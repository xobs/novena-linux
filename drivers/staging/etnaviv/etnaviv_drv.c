/*
 * Copyright (C) 2015 Etnaviv Project
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/component.h>
#include <linux/of_platform.h>

#include "etnaviv_drv.h"
#include "etnaviv_gpu.h"
#include "etnaviv_gem.h"
#include "etnaviv_mmu.h"

#ifdef CONFIG_DRM_ETNAVIV_REGISTER_LOGGING
static bool reglog;
MODULE_PARM_DESC(reglog, "Enable register read/write logging");
module_param(reglog, bool, 0600);
#else
#define reglog 0
#endif

void __iomem *etnaviv_ioremap(struct platform_device *pdev, const char *name,
		const char *dbgname)
{
	struct resource *res;
	void __iomem *ptr;

	if (name)
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	else
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	ptr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ptr)) {
		dev_err(&pdev->dev, "failed to ioremap %s: %ld\n", name,
			PTR_ERR(ptr));
		return ptr;
	}

	if (reglog)
		dev_printk(KERN_DEBUG, &pdev->dev, "IO:region %s 0x%p %08zx\n",
			   dbgname, ptr, (size_t)resource_size(res));

	return ptr;
}

void etnaviv_writel(u32 data, void __iomem *addr)
{
	if (reglog)
		printk(KERN_DEBUG "IO:W %p %08x\n", addr, data);

	writel(data, addr);
}

u32 etnaviv_readl(const void __iomem *addr)
{
	u32 val = readl(addr);

	if (reglog)
		printk(KERN_DEBUG "IO:R %p %08x\n", addr, val);

	return val;
}

/*
 * DRM operations:
 */

static int etnaviv_unload(struct drm_device *dev)
{
	struct etnaviv_drm_private *priv = dev->dev_private;

	flush_workqueue(priv->wq);
	destroy_workqueue(priv->wq);

	component_unbind_all(dev->dev, dev);

	dev->dev_private = NULL;

	kfree(priv);

	return 0;
}


static void load_gpu(struct drm_device *dev)
{
	struct etnaviv_drm_private *priv = dev->dev_private;
	unsigned int i;

	for (i = 0; i < ETNA_MAX_PIPES; i++) {
		struct etnaviv_gpu *g = priv->gpu[i];

		if (g) {
			int ret;

			ret = etnaviv_gpu_init(g);
			if (ret) {
				dev_err(g->dev, "hw init failed: %d\n", ret);
				priv->gpu[i] = NULL;
			}
		}
	}
}

static int etnaviv_load(struct drm_device *dev, unsigned long flags)
{
	struct platform_device *pdev = dev->platformdev;
	struct etnaviv_drm_private *priv;
	int err;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev->dev, "failed to allocate private data\n");
		return -ENOMEM;
	}

	dev->dev_private = priv;

	priv->wq = alloc_ordered_workqueue("etnaviv", 0);
	if (!priv->wq) {
		err = -ENOMEM;
		goto err_wq;
	}

	INIT_LIST_HEAD(&priv->inactive_list);
	priv->num_gpus = 0;

	platform_set_drvdata(pdev, dev);

	err = component_bind_all(dev->dev, dev);
	if (err < 0)
		goto err_bind;

	load_gpu(dev);

	return 0;

err_bind:
	flush_workqueue(priv->wq);
	destroy_workqueue(priv->wq);
err_wq:
	kfree(priv);
	return err;
}

static int etnaviv_open(struct drm_device *dev, struct drm_file *file)
{
	struct etnaviv_file_private *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	file->driver_priv = ctx;

	return 0;
}

static void etnaviv_preclose(struct drm_device *dev, struct drm_file *file)
{
	struct etnaviv_drm_private *priv = dev->dev_private;
	struct etnaviv_file_private *ctx = file->driver_priv;
	unsigned int i;

	mutex_lock(&dev->struct_mutex);
	for (i = 0; i < ETNA_MAX_PIPES; i++) {
		struct etnaviv_gpu *gpu = priv->gpu[i];

		if (gpu && gpu->lastctx == ctx)
			gpu->lastctx = NULL;
	}
	mutex_unlock(&dev->struct_mutex);

	kfree(ctx);
}

/*
 * DRM debugfs:
 */

#ifdef CONFIG_DEBUG_FS
static int etnaviv_gem_show(struct drm_device *dev, struct seq_file *m)
{
	struct etnaviv_drm_private *priv = dev->dev_private;
	struct etnaviv_gpu *gpu;
	unsigned int i;

	for (i = 0; i < ETNA_MAX_PIPES; i++) {
		gpu = priv->gpu[i];
		if (gpu) {
			seq_printf(m, "Active Objects (%s):\n",
				   dev_name(gpu->dev));
			etnaviv_gem_describe_objects(&gpu->active_list, m);
		}
	}

	seq_puts(m, "Inactive Objects:\n");
	etnaviv_gem_describe_objects(&priv->inactive_list, m);

	return 0;
}

static int etnaviv_mm_show(struct drm_device *dev, struct seq_file *m)
{
	return drm_mm_dump_table(m, &dev->vma_offset_manager->vm_addr_space_mm);
}

static int etnaviv_mmu_show(struct drm_device *dev, struct seq_file *m)
{
	struct etnaviv_drm_private *priv = dev->dev_private;
	struct etnaviv_gpu *gpu;
	unsigned int i;

	for (i = 0; i < ETNA_MAX_PIPES; i++) {
		gpu = priv->gpu[i];
		if (gpu) {
			seq_printf(m, "Active Objects (%s):\n",
				   dev_name(gpu->dev));
			drm_mm_dump_table(m, &gpu->mmu->mm);
		}
	}
	return 0;
}

static int show_locked(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	int (*show)(struct drm_device *dev, struct seq_file *m) =
			node->info_ent->data;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	ret = show(dev, m);

	mutex_unlock(&dev->struct_mutex);

	return ret;
}

static int show_each_gpu(struct seq_file *m, void *arg)
{
	struct drm_info_node *node = (struct drm_info_node *) m->private;
	struct drm_device *dev = node->minor->dev;
	struct etnaviv_drm_private *priv = dev->dev_private;
	struct etnaviv_gpu *gpu;
	int (*show)(struct etnaviv_gpu *gpu, struct seq_file *m) =
			node->info_ent->data;
	unsigned int i;
	int ret = 0;

	for (i = 0; i < ETNA_MAX_PIPES; i++) {
		gpu = priv->gpu[i];
		if (!gpu)
			continue;

		ret = show(gpu, m);
		if (ret < 0)
			break;
	}

	return ret;
}

static struct drm_info_list etnaviv_debugfs_list[] = {
		{"gpu", show_each_gpu, 0, etnaviv_gpu_debugfs},
		{"gem", show_locked, 0, etnaviv_gem_show},
		{ "mm", show_locked, 0, etnaviv_mm_show },
		{"mmu", show_locked, 0, etnaviv_mmu_show},
};

static int etnaviv_debugfs_init(struct drm_minor *minor)
{
	struct drm_device *dev = minor->dev;
	int ret;

	ret = drm_debugfs_create_files(etnaviv_debugfs_list,
			ARRAY_SIZE(etnaviv_debugfs_list),
			minor->debugfs_root, minor);

	if (ret) {
		dev_err(dev->dev, "could not install etnaviv_debugfs_list\n");
		return ret;
	}

	return ret;
}

static void etnaviv_debugfs_cleanup(struct drm_minor *minor)
{
	drm_debugfs_remove_files(etnaviv_debugfs_list,
			ARRAY_SIZE(etnaviv_debugfs_list), minor);
}
#endif

/*
 * DRM ioctls:
 */

static int etnaviv_ioctl_get_param(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct etnaviv_drm_private *priv = dev->dev_private;
	struct drm_etnaviv_param *args = data;
	struct etnaviv_gpu *gpu;

	if (args->pipe >= ETNA_MAX_PIPES)
		return -EINVAL;

	gpu = priv->gpu[args->pipe];
	if (!gpu)
		return -ENXIO;

	return etnaviv_gpu_get_param(gpu, args->param, &args->value);
}

static int etnaviv_ioctl_gem_new(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_etnaviv_gem_new *args = data;

	return etnaviv_gem_new_handle(dev, file, args->size,
			args->flags, &args->handle);
}

#define TS(t) ((struct timespec){ \
	.tv_sec = (t).tv_sec, \
	.tv_nsec = (t).tv_nsec \
})

static int etnaviv_ioctl_gem_cpu_prep(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_etnaviv_gem_cpu_prep *args = data;
	struct drm_gem_object *obj;
	int ret;

	obj = drm_gem_object_lookup(dev, file, args->handle);
	if (!obj)
		return -ENOENT;

	ret = etnaviv_gem_cpu_prep(obj, args->op, &TS(args->timeout));

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static int etnaviv_ioctl_gem_cpu_fini(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_etnaviv_gem_cpu_fini *args = data;
	struct drm_gem_object *obj;
	int ret;

	obj = drm_gem_object_lookup(dev, file, args->handle);
	if (!obj)
		return -ENOENT;

	ret = etnaviv_gem_cpu_fini(obj);

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static int etnaviv_ioctl_gem_info(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_etnaviv_gem_info *args = data;
	struct drm_gem_object *obj;
	int ret;

	if (args->pad)
		return -EINVAL;

	obj = drm_gem_object_lookup(dev, file, args->handle);
	if (!obj)
		return -ENOENT;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret == 0) {
		ret = etnaviv_gem_mmap_offset(obj, &args->offset);

		mutex_unlock(&dev->struct_mutex);
	}

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static int etnaviv_ioctl_wait_fence(struct drm_device *dev, void *data,
		struct drm_file *file)
{
	struct drm_etnaviv_wait_fence *args = data;
	struct etnaviv_drm_private *priv = dev->dev_private;
	struct etnaviv_gpu *gpu;

	if (args->pipe >= ETNA_MAX_PIPES)
		return -EINVAL;

	gpu = priv->gpu[args->pipe];
	if (!gpu)
		return -ENXIO;

	return etnaviv_gpu_wait_fence_interruptible(gpu, args->fence,
						    &TS(args->timeout));
}

static int etnaviv_ioctl_gem_userptr(struct drm_device *dev, void *data,
	struct drm_file *file)
{
	struct drm_etnaviv_gem_userptr *args = data;
	int access;

	if (args->flags & ~(ETNA_USERPTR_READ|ETNA_USERPTR_WRITE) ||
	    args->flags == 0)
		return -EINVAL;

	if (offset_in_page(args->user_ptr | args->user_size) ||
	    (uintptr_t)args->user_ptr != args->user_ptr ||
	    (u32)args->user_size != args->user_size)
		return -EINVAL;

	if (args->flags & ETNA_USERPTR_WRITE)
		access = VERIFY_WRITE;
	else
		access = VERIFY_READ;

	if (!access_ok(access, (void __user *)(unsigned long)args->user_ptr,
		       args->user_size))
		return -EFAULT;

	return etnaviv_gem_new_userptr(dev, file, args->user_ptr,
				       args->user_size, args->flags,
				       &args->handle);
}

static int etnaviv_ioctl_gem_wait(struct drm_device *dev, void *data,
	struct drm_file *file)
{
	struct etnaviv_drm_private *priv = dev->dev_private;
	struct drm_etnaviv_gem_wait *args = data;
	struct drm_gem_object *obj;
	struct etnaviv_gpu *gpu;
	int ret;

	if (args->pipe >= ETNA_MAX_PIPES)
		return -EINVAL;

	gpu = priv->gpu[args->pipe];
	if (!gpu)
		return -ENXIO;

	obj = drm_gem_object_lookup(dev, file, args->handle);
	if (!obj)
		return -ENOENT;

	ret = etnaviv_gem_wait_bo(gpu, obj, &TS(args->timeout));

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static const struct drm_ioctl_desc etnaviv_ioctls[] = {
#define ETNA_IOCTL(n, func, flags) \
	DRM_IOCTL_DEF_DRV(ETNAVIV_##n, etnaviv_ioctl_##func, flags)
	ETNA_IOCTL(GET_PARAM,    get_param,    DRM_UNLOCKED|DRM_RENDER_ALLOW),
	ETNA_IOCTL(GEM_NEW,      gem_new,      DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	ETNA_IOCTL(GEM_INFO,     gem_info,     DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	ETNA_IOCTL(GEM_CPU_PREP, gem_cpu_prep, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	ETNA_IOCTL(GEM_CPU_FINI, gem_cpu_fini, DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	ETNA_IOCTL(GEM_SUBMIT,   gem_submit,   DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	ETNA_IOCTL(WAIT_FENCE,   wait_fence,   DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
	ETNA_IOCTL(GEM_USERPTR,  gem_userptr,  DRM_UNLOCKED|DRM_RENDER_ALLOW),
	ETNA_IOCTL(GEM_WAIT,     gem_wait,     DRM_UNLOCKED|DRM_AUTH|DRM_RENDER_ALLOW),
};

static const struct vm_operations_struct vm_ops = {
	.fault = etnaviv_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct file_operations fops = {
	.owner              = THIS_MODULE,
	.open               = drm_open,
	.release            = drm_release,
	.unlocked_ioctl     = drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl       = drm_compat_ioctl,
#endif
	.poll               = drm_poll,
	.read               = drm_read,
	.llseek             = no_llseek,
	.mmap               = etnaviv_gem_mmap,
};

static struct drm_driver etnaviv_drm_driver = {
	.driver_features    = DRIVER_HAVE_IRQ |
				DRIVER_GEM |
				DRIVER_PRIME |
				DRIVER_RENDER,
	.load               = etnaviv_load,
	.unload             = etnaviv_unload,
	.open               = etnaviv_open,
	.preclose           = etnaviv_preclose,
	.set_busid          = drm_platform_set_busid,
	.gem_free_object    = etnaviv_gem_free_object,
	.gem_vm_ops         = &vm_ops,
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export   = drm_gem_prime_export,
	.gem_prime_import   = drm_gem_prime_import,
	.gem_prime_pin      = etnaviv_gem_prime_pin,
	.gem_prime_unpin    = etnaviv_gem_prime_unpin,
	.gem_prime_get_sg_table = etnaviv_gem_prime_get_sg_table,
	.gem_prime_import_sg_table = etnaviv_gem_prime_import_sg_table,
	.gem_prime_vmap     = etnaviv_gem_prime_vmap,
	.gem_prime_vunmap   = etnaviv_gem_prime_vunmap,
#ifdef CONFIG_DEBUG_FS
	.debugfs_init       = etnaviv_debugfs_init,
	.debugfs_cleanup    = etnaviv_debugfs_cleanup,
#endif
	.ioctls             = etnaviv_ioctls,
	.num_ioctls         = DRM_ETNAVIV_NUM_IOCTLS,
	.fops               = &fops,
	.name               = "etnaviv",
	.desc               = "etnaviv DRM",
	.date               = "20150302",
	.major              = 1,
	.minor              = 0,
};

/*
 * Platform driver:
 */
static int etnaviv_bind(struct device *dev)
{
	return drm_platform_init(&etnaviv_drm_driver, to_platform_device(dev));
}

static void etnaviv_unbind(struct device *dev)
{
	drm_put_dev(dev_get_drvdata(dev));
}

static const struct component_master_ops etnaviv_master_ops = {
	.bind = etnaviv_bind,
	.unbind = etnaviv_unbind,
};

static int compare_of(struct device *dev, void *data)
{
	struct device_node *np = data;

	return dev->of_node == np;
}

static int compare_str(struct device *dev, void *data)
{
	return !strcmp(dev_name(dev), data);
}

static int etnaviv_pdev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct component_match *match = NULL;

	dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));

	if (node) {
		struct device_node *core_node;
		int i;

		for (i = 0; ; i++) {
			core_node = of_parse_phandle(node, "cores", i);
			if (!core_node)
				break;

			component_match_add(&pdev->dev, &match, compare_of,
					    core_node);
			of_node_put(core_node);
		}
	} else if (dev->platform_data) {
		char **names = dev->platform_data;
		unsigned i;

		for (i = 0; names[i]; i++)
			component_match_add(dev, &match, compare_str, names[i]);
	}

	return component_master_add_with_match(dev, &etnaviv_master_ops, match);
}

static int etnaviv_pdev_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &etnaviv_master_ops);

	return 0;
}

static const struct of_device_id dt_match[] = {
	{ .compatible = "fsl,imx-gpu-subsystem" },
	{ .compatible = "marvell,dove-gpu-subsystem" },
	{}
};
MODULE_DEVICE_TABLE(of, dt_match);

static struct platform_driver etnaviv_platform_driver = {
	.probe      = etnaviv_pdev_probe,
	.remove     = etnaviv_pdev_remove,
	.driver     = {
		.owner  = THIS_MODULE,
		.name   = "etnaviv",
		.of_match_table = dt_match,
	},
};

static int __init etnaviv_init(void)
{
	int ret;

	ret = platform_driver_register(&etnaviv_gpu_driver);
	if (ret != 0)
		return ret;

	ret = platform_driver_register(&etnaviv_platform_driver);
	if (ret != 0)
		platform_driver_unregister(&etnaviv_gpu_driver);

	return ret;
}
module_init(etnaviv_init);

static void __exit etnaviv_exit(void)
{
	platform_driver_unregister(&etnaviv_gpu_driver);
	platform_driver_unregister(&etnaviv_platform_driver);
}
module_exit(etnaviv_exit);

MODULE_AUTHOR("Christian Gmeiner <christian.gmeiner@gmail.com>");
MODULE_AUTHOR("Russell King <rmk+kernel@arm.linux.org.uk>");
MODULE_AUTHOR("Lucas Stach <l.stach@pengutronix.de>");
MODULE_DESCRIPTION("etnaviv DRM Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:etnaviv");
