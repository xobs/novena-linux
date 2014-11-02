ccflags-y := -Iinclude/drm -Idrivers/staging/vivante
ifeq (, $(findstring -W,$(EXTRA_CFLAGS)))
	ccflags-y += -Werror
endif

etnaviv-y := \
	etnaviv_drv.o \
	etnaviv_gem.o \
	etnaviv_gem_prime.o \
	etnaviv_gem_submit.o \
	etnaviv_gpu.o \
	etnaviv_iommu.o \
	etnaviv_iommu_v2.o \
	etnaviv_mmu.o \
	etnaviv_buffer.o

obj-$(CONFIG_DRM_ETNAVIV)	+= etnaviv.o
