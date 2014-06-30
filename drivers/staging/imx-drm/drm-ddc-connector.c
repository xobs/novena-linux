#include <linux/i2c.h>
#include <linux/module.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>

#include "drm-ddc-connector.h"

static enum drm_connector_status
drm_ddc_connector_detect(struct drm_connector *connector, bool force)
{
	struct drm_ddc_connector *ddc_conn = to_ddc_conn(connector);

	return ddc_conn->detect ? ddc_conn->detect(connector, force) :
		connector_status_connected;
}

int drm_ddc_connector_get_modes(struct drm_connector *connector)
{
	struct drm_ddc_connector *ddc_conn = to_ddc_conn(connector);
	struct edid *edid;
	int ret = 0;

	if (!ddc_conn->ddc)
		return 0;

	edid = drm_get_edid(connector, ddc_conn->ddc);
	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		/* Store the ELD */
		drm_edid_to_eld(connector, edid);
		kfree(edid);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(drm_ddc_connector_get_modes);

static void drm_ddc_connector_destroy(struct drm_connector *connector)
{
	struct drm_ddc_connector *ddc_conn = to_ddc_conn(connector);

	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	if (ddc_conn->ddc)
		i2c_put_adapter(ddc_conn->ddc);
}

static const struct drm_connector_funcs drm_ddc_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = drm_ddc_connector_detect,
	.destroy = drm_ddc_connector_destroy,
};

int drm_ddc_connector_add(struct drm_device *drm,
	struct drm_ddc_connector *ddc_conn, int connector_type)
{
	drm_connector_init(drm, &ddc_conn->connector, &drm_ddc_connector_funcs,
			   connector_type);
	return 0;
}
EXPORT_SYMBOL_GPL(drm_ddc_connector_add);

struct drm_ddc_connector *drm_ddc_connector_create(struct drm_device *drm,
	struct device_node *np, void *private)
{
	struct drm_ddc_connector *ddc_conn;
	struct device_node *ddc_node;

	ddc_conn = devm_kzalloc(drm->dev, sizeof(*ddc_conn), GFP_KERNEL);
	if (!ddc_conn)
		return ERR_PTR(-ENOMEM);

	ddc_conn->private = private;

	ddc_node = of_parse_phandle(np, "ddc-i2c-bus", 0);
	if (ddc_node) {
		ddc_conn->ddc = of_find_i2c_adapter_by_node(ddc_node);
		of_node_put(ddc_node);
		if (!ddc_conn->ddc)
			return ERR_PTR(-EPROBE_DEFER);
	}

	return ddc_conn;
}
EXPORT_SYMBOL_GPL(drm_ddc_connector_create);

MODULE_AUTHOR("Russell King <rmk+kernel@arm.linux.org.uk>");
MODULE_DESCRIPTION("Generic DRM DDC connector module");
MODULE_LICENSE("GPL v2");
