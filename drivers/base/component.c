/*
 * Componentized device handling.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This is work in progress.  We gather up the component devices into a list,
 * and bind them when instructed.  At the moment, we're specific to the DRM
 * subsystem, and only handles one master device, but this doesn't have to be
 * the case.
 */
#include <linux/component.h>
#include <linux/device.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>

struct master {
	struct list_head node;
	struct list_head components;
	struct kref kref;
	struct mutex mutex;
	bool bound;
	bool dead;

	const struct component_master_ops *ops;
	struct device *dev;
};

struct component {
	struct list_head node;
	struct list_head master_node;
	struct master *master;
	bool bound;

	const struct component_ops *ops;
	struct device *dev;
};

static DEFINE_MUTEX(component_mutex);
static LIST_HEAD(component_list);
static DEFINE_MUTEX(master_mutex);
static LIST_HEAD(masters);

static void master_release(struct kref *kref) __releases(master_mutex)
{
	struct master *master = container_of(kref, struct master, kref);

	list_del(&master->node);
	mutex_unlock(&master_mutex);
	WARN_ON(!list_empty(&master->components));
	kfree(master);
}

static void master_release_nolock(struct kref *kref)
{
	struct master *master = container_of(kref, struct master, kref);

	list_del(&master->node);
	WARN_ON(!list_empty(&master->components));
	kfree(master);
}

static void master_norelease(struct kref *kref)
{
	WARN_ON(kref);
}

static struct master *__master_find(struct device *dev, const struct component_master_ops *ops)
{
	struct master *m;

	list_for_each_entry(m, &masters, node)
		if (m->dev == dev && (!ops || m->ops == ops))
			return m;

	return NULL;
}

/*
 * Find the master structure for this device, and make sure that the
 * per-master lock is held.  This is used from within the master device
 * drivers bind and unbind callbacks, which should only be called from
 * paths where the per-master lock is already held.
 */
static struct master *master_find_locked(struct device *dev, const struct component_master_ops *ops)
{
	struct master *master;

	mutex_lock(&master_mutex);
	master = __master_find(dev, NULL);
	WARN_ON(master && !mutex_is_locked(&master->mutex));
	mutex_unlock(&master_mutex);

	return master;
}

/* Attach an unattached component to a master. */
static void component_attach_master(struct master *master, struct component *c)
{
	kref_get(&master->kref);

	c->master = master;

	list_add_tail(&c->master_node, &master->components);
}

/* Detach a component from a master. */
static void component_detach_master(struct master *master, struct component *c)
{
	list_del(&c->master_node);

	c->master = NULL;

	/* This kref_put should never release the master */
	kref_put(&master->kref, master_norelease);
}

int component_master_add_child(struct master *master,
	int (*compare)(struct device *, void *), void *compare_data)
{
	struct component *c;
	int ret = -ENXIO;

	mutex_lock(&component_mutex);
	list_for_each_entry(c, &component_list, node) {
		if (c->master)
			continue;

		if (compare(c->dev, compare_data)) {
			component_attach_master(master, c);
			ret = 0;
			break;
		}
	}
	mutex_unlock(&component_mutex);

	return ret;
}
EXPORT_SYMBOL(component_master_add_child);

/* Detach all attached components from this master */
static void master_remove_components(struct master *master)
{
	mutex_lock(&component_mutex);
	while (!list_empty(&master->components)) {
		struct component *c = list_first_entry(&master->components,
					struct component, master_node);

		WARN_ON(c->master != master);

		component_detach_master(master, c);
	}
	mutex_unlock(&component_mutex);
}

/*
 * Try to bring up a master.  If component is NULL, we're interested in
 * this master, otherwise it's a component which must be present to try
 * and bring up the master.
 *
 * Returns 1 for successful bringup, 0 if not ready, or -ve errno.
 */
static int try_to_bring_up_master(struct master *master,
	struct component *component)
{
	int ret = 0;

	mutex_lock(&master->mutex);
	if (!master->dead && !master->bound) {
		/*
		 * Search the list of components, looking for components that
		 * belong to this master, and attach them to the master.
		 */
		if (master->ops->add_components(master->dev, master)) {
			/* Failed to find all components */
			master_remove_components(master);
			ret = 0;
			goto out;
		}

		if (component && component->master != master) {
			master_remove_components(master);
			ret = 0;
			goto out;
		}

		/* Found all components */
		ret = master->ops->bind(master->dev);
		if (ret < 0) {
			master_remove_components(master);
			goto out;
		}

		master->bound = true;
		ret = 1;
	}
out:
	mutex_unlock(&master->mutex);

	return ret;
}

static int try_to_bring_up_masters(struct component *component)
{
	struct master *m, *last = NULL;
	int ret = 0;

	mutex_lock(&master_mutex);
	list_for_each_entry(m, &masters, node) {
		if (last) {
			kref_put(&last->kref, master_release_nolock);
			last = NULL;
		}

		if (m->dead || m->bound)
			continue;

		kref_get(&m->kref);
		mutex_unlock(&master_mutex);

		ret = try_to_bring_up_master(m, component);

		mutex_lock(&master_mutex);
		last = m;

		if (ret != 0)
			break;
	}

	if (last)
		kref_put(&last->kref, master_release_nolock);
	mutex_unlock(&master_mutex);

	return ret;
}

static void take_down_master(struct master *master)
{
	mutex_lock(&master->mutex);
	if (master->bound) {
		master->ops->unbind(master->dev);
		master->bound = false;
	}

	master_remove_components(master);
	mutex_unlock(&master->mutex);
}

int component_master_add(struct device *dev, const struct component_master_ops *ops)
{
	struct master *master;
	int ret;

	master = kzalloc(sizeof(*master), GFP_KERNEL);
	if (!master)
		return -ENOMEM;

	master->dev = dev;
	master->ops = ops;
	mutex_init(&master->mutex);
	INIT_LIST_HEAD(&master->components);
	kref_init(&master->kref);

	/*
	 * Add to the list of available masters.  This is so
	 * the component code below can find this master.
	 */
	mutex_lock(&master_mutex);
	list_add(&master->node, &masters);
	mutex_unlock(&master_mutex);

	ret = try_to_bring_up_master(master, NULL);

	/* Delete off the list if we weren't successful */
	if (ret < 0)
		kref_put_mutex(&master->kref, master_release, &master_mutex);

	return ret < 0 ? ret : 0;
}
EXPORT_SYMBOL_GPL(component_master_add);

void component_master_del(struct device *dev, const struct component_master_ops *ops)
{
	struct master *master;

	mutex_lock(&master_mutex);
	master = __master_find(dev, ops);
	if (master)
		master->dead = true;
	mutex_unlock(&master_mutex);

	if (master) {
		take_down_master(master);

		kref_put_mutex(&master->kref, master_release, &master_mutex);
	}
}
EXPORT_SYMBOL_GPL(component_master_del);

static void component_unbind(struct component *component,
	struct master *master, void *data)
{
	WARN_ON(!component->bound);

	component->ops->unbind(component->dev, master->dev, data);
	component->bound = false;

	/* Release all resources claimed in the binding of this component */
	devres_release_group(component->dev, component);
}

void component_unbind_all(struct device *master_dev, void *data)
{
	struct master *master = master_find_locked(master_dev, NULL);
	struct component *c;

	if (!master)
		return;

	list_for_each_entry_reverse(c, &master->components, master_node)
		component_unbind(c, master, data);
}
EXPORT_SYMBOL_GPL(component_unbind_all);

static int component_bind(struct component *component, struct master *master,
	void *data)
{
	int ret;

	/*
	 * Each component initialises inside its own devres group.
	 * This allows us to roll-back a failed component without
	 * affecting anything else.
	 */
	if (!devres_open_group(master->dev, NULL, GFP_KERNEL))
		return -ENOMEM;

	/*
	 * Also open a group for the device itself: this allows us
	 * to release the resources claimed against the sub-device
	 * at the appropriate moment.
	 */
	if (!devres_open_group(component->dev, component, GFP_KERNEL)) {
		devres_release_group(master->dev, NULL);
		return -ENOMEM;
	}

	dev_dbg(master->dev, "binding %s (ops %ps)\n",
		dev_name(component->dev), component->ops);

	ret = component->ops->bind(component->dev, master->dev, data);
	if (!ret) {
		component->bound = true;

		/*
		 * Close the component device's group so that resources
		 * allocated in the binding are encapsulated for removal
		 * at unbind.  Remove the group on the DRM device as we
		 * can clean those resources up independently.
		 */
		devres_close_group(component->dev, NULL);
		devres_remove_group(master->dev, NULL);

		dev_info(master->dev, "bound %s (ops %ps)\n",
			 dev_name(component->dev), component->ops);
	} else {
		devres_release_group(component->dev, NULL);
		devres_release_group(master->dev, NULL);

		dev_err(master->dev, "failed to bind %s (ops %ps): %d\n",
			dev_name(component->dev), component->ops, ret);
	}

	return ret;
}

int component_bind_all(struct device *master_dev, void *data)
{
	struct master *master = master_find_locked(master_dev, NULL);
	struct component *c;
	int ret = 0;

	if (!master)
		return -EINVAL;

	list_for_each_entry(c, &master->components, master_node) {
		ret = component_bind(c, master, data);
		if (ret)
			break;
	}

	if (ret != 0) {
		list_for_each_entry_continue_reverse(c, &master->components,
						     master_node)
			component_unbind(c, master, data);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(component_bind_all);

int component_add(struct device *dev, const struct component_ops *ops)
{
	struct component *component;
	int ret;

	component = kzalloc(sizeof(*component), GFP_KERNEL);
	if (!component)
		return -ENOMEM;

	component->ops = ops;
	component->dev = dev;

	dev_dbg(dev, "adding component (ops %ps)\n", ops);

	mutex_lock(&component_mutex);
	list_add_tail(&component->node, &component_list);
	mutex_unlock(&component_mutex);

	ret = try_to_bring_up_masters(component);
	if (ret < 0) {
		mutex_lock(&component_mutex);
		list_del(&component->node);
		mutex_unlock(&component_mutex);

		kfree(component);
	}

	return ret < 0 ? ret : 0;
}
EXPORT_SYMBOL_GPL(component_add);

void component_del(struct device *dev, const struct component_ops *ops)
{
	struct component *c, *component = NULL;
	struct master *master = NULL;

	mutex_lock(&component_mutex);
	list_for_each_entry(c, &component_list, node)
		if (c->dev == dev && c->ops == ops) {
			master = c->master;
			if (master)
				kref_get(&master->kref);
			list_del(&c->node);
			component = c;
			break;
		}
	mutex_unlock(&component_mutex);

	if (WARN_ON(!component))
		return;

	if (master) {
		take_down_master(master);
		kref_put_mutex(&master->kref, master_release, &master_mutex);
	}

	kfree(component);
}
EXPORT_SYMBOL_GPL(component_del);

MODULE_LICENSE("GPL v2");
