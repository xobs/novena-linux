#ifndef DRM_DDC_CONNECTOR_H
#define DRM_DDC_CONNECTOR_H

#include <drm/drm_crtc.h>

struct drm_ddc_connector {
	struct i2c_adapter *ddc;
	struct drm_connector connector;
	void *private;
};

#define to_ddc_conn(c) container_of(c, struct drm_ddc_connector, connector)

enum drm_connector_status drm_ddc_connector_always_connected(
	struct drm_connector *connector, bool force);
int drm_ddc_connector_get_modes(struct drm_connector *connector);
void drm_ddc_connector_add(struct drm_device *drm,
	struct drm_ddc_connector *ddc_conn,
	struct drm_connector_funcs *funcs, int connector_type);
void drm_ddc_connector_destroy(struct drm_connector *connector);
struct drm_ddc_connector *drm_ddc_connector_create(struct drm_device *drm,
	struct device_node *np, void *private);

static inline void *drm_ddc_private(struct drm_connector *connector)
{
	struct drm_ddc_connector *ddc_conn = to_ddc_conn(connector);

	return ddc_conn->private;
}

#endif
