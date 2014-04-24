#ifndef DRM_DDC_CONNECTOR_H
#define DRM_DDC_CONNECTOR_H

struct drm_ddc_connector {
	struct i2c_adapter *ddc;
	struct drm_connector connector;
	enum drm_connector_status (*detect)(struct drm_connector *, bool);
	void *private;
};

#define to_ddc_conn(c) container_of(c, struct drm_ddc_connector, connector)

int drm_ddc_connector_get_modes(struct drm_connector *connector);
int drm_ddc_connector_add(struct drm_device *drm,
	struct drm_ddc_connector *ddc_conn, int connector_type);
struct drm_ddc_connector *drm_ddc_connector_create(struct drm_device *drm,
	struct device_node *np, void *private);

static inline void *drm_ddc_private(struct drm_connector *connector)
{
	struct drm_ddc_connector *ddc_conn = to_ddc_conn(connector);

	return ddc_conn->private;
}

#endif
