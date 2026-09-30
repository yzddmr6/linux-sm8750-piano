// SPDX-License-Identifier: GPL-2.0-only
/* Included with the private helper; consumers use the real Type-C APIs. */
#include <kunit/device.h>
#include <kunit/test.h>
#include <drm/drm_bridge.h>

struct altmode_test_observation {
	unsigned long mode;
	u16 svid;
	struct typec_displayport_data data;
	unsigned int calls;
};

struct altmode_test_ctx {
	struct pmic_glink_altmode altmode;
	struct pmic_glink_altmode_port port;
	struct device_node bridge_node;
	struct drm_bridge *bridge;
	struct typec_switch_dev *sw;
	struct typec_mux_dev *mux;
	struct typec_retimer *retimer;
	struct altmode_test_observation mux_seen;
	struct altmode_test_observation retimer_seen;
	int mux_error;
	int switch_error;
	int retimer_error;
	unsigned int switch_calls;
	unsigned int hpd_calls;
	unsigned int ack_calls;
	enum drm_connector_status hpd_status;
};

static const struct property_entry altmode_test_sink_props[] = {
	PROPERTY_ENTRY_BOOL("orientation-switch"),
	PROPERTY_ENTRY_BOOL("mode-switch"),
	PROPERTY_ENTRY_BOOL("retimer-switch"),
	{ }
};

static const struct software_node altmode_test_sink = {
	.name = "pmic-glink-dp-test-sink",
	.properties = altmode_test_sink_props,
};

static const struct property_entry altmode_test_port_props[] = {
	PROPERTY_ENTRY_REF("orientation-switch", &altmode_test_sink),
	PROPERTY_ENTRY_REF("mode-switch", &altmode_test_sink),
	PROPERTY_ENTRY_REF("retimer-switch", &altmode_test_sink),
	{ }
};

static const struct software_node altmode_test_port_node = {
	.name = "pmic-glink-dp-test-port",
	.properties = altmode_test_port_props,
};

static const struct software_node * const altmode_test_nodes[] = {
	&altmode_test_sink, &altmode_test_port_node, NULL,
};

static int altmode_test_mux_set(struct typec_mux_dev *mux, struct typec_mux_state *state)
{
	struct altmode_test_ctx *ctx = typec_mux_get_drvdata(mux);

	ctx->mux_seen.mode = state->mode;
	ctx->mux_seen.svid = state->alt ? state->alt->svid : 0;
	if (state->data)
		ctx->mux_seen.data = *(struct typec_displayport_data *)state->data;
	ctx->mux_seen.calls++;
	return ctx->mux_error;
}

static int altmode_test_retimer_set(struct typec_retimer *retimer,
				  struct typec_retimer_state *state)
{
	struct altmode_test_ctx *ctx = typec_retimer_get_drvdata(retimer);

	ctx->retimer_seen.mode = state->mode;
	ctx->retimer_seen.svid = state->alt ? state->alt->svid : 0;
	if (state->data)
		ctx->retimer_seen.data = *(struct typec_displayport_data *)state->data;
	ctx->retimer_seen.calls++;
	return ctx->retimer_error;
}

static int altmode_test_switch_set(struct typec_switch_dev *sw,
				   enum typec_orientation orientation)
{
	struct altmode_test_ctx *ctx = typec_switch_get_drvdata(sw);

	ctx->switch_calls++;
	return ctx->switch_error;
}

static int altmode_test_ack(struct pmic_glink_altmode *altmode, u32 cmd, u32 arg)
{
	struct altmode_test_ctx *ctx = container_of(altmode, struct altmode_test_ctx, altmode);

	KUNIT_EXPECT_EQ(kunit_get_current_test(), cmd, ALTMODE_PAN_ACK);
	KUNIT_EXPECT_EQ(kunit_get_current_test(), arg, ctx->port.index);
	ctx->ack_calls++;
	return 0;
}

static void altmode_test_hpd(void *data, enum drm_connector_status status)
{
	struct altmode_test_ctx *ctx = data;

	ctx->hpd_calls++;
	ctx->hpd_status = status;
}

static void altmode_test_put_node(void *node)
{
	of_node_put(node);
}

static void altmode_test_put_bridge(void *bridge)
{
	drm_bridge_hpd_disable(bridge);
	drm_bridge_put(bridge);
}

static void altmode_test_remove_switch(void *sw)
{
	typec_switch_unregister(sw);
}

static void altmode_test_remove_nodes(void *unused)
{
	software_node_unregister_node_group(altmode_test_nodes);
}

static void altmode_test_remove_mux(void *mux)
{
	typec_mux_unregister(mux);
}

static void altmode_test_remove_retimer(void *retimer)
{
	typec_retimer_unregister(retimer);
}

static void altmode_test_put_handles(void *data)
{
	struct altmode_test_ctx *ctx = data;

	typec_retimer_put(ctx->port.typec_retimer);
	typec_mux_put(ctx->port.typec_mux);
	typec_switch_put(ctx->port.typec_switch);
}

static int altmode_test_init(struct kunit *test)
{
	struct typec_mux_desc mux_desc = { .set = altmode_test_mux_set };
	struct typec_retimer_desc retimer_desc = { .set = altmode_test_retimer_set };
	struct typec_switch_desc switch_desc = { .set = altmode_test_switch_set };
	struct altmode_test_ctx *ctx;
	struct device *dev;
	int ret;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	test->priv = ctx;
	/* Unattached identity node; no hardware tree is modified. */
	of_node_init(&ctx->bridge_node);
	ctx->bridge_node.full_name = "/kunit-pmic-glink-dp";
	kunit_add_action_or_reset(test, altmode_test_put_node, &ctx->bridge_node);
	dev = kunit_device_register(test, "pmic-glink-dp-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	ctx->altmode.dev = dev;
	ctx->port.altmode = &ctx->altmode;
	ctx->port.dp_alt.svid = USB_TYPEC_DP_SID;
	ctx->port.svid = USB_TYPEC_DP_SID;
	ctx->port.mode = DP_PIN_ASSIGN_C;
	ctx->port.mux_ctrl = MUX_CTRL_STATE_DP4LN;
	ctx->port.orientation = TYPEC_ORIENTATION_NORMAL;
	ctx->port.hpd_state = 1;
	kunit_activate_static_stub(test, pmic_glink_altmode_request, altmode_test_ack);
	ctx->port.bridge = devm_drm_dp_hpd_bridge_alloc(dev, &ctx->bridge_node);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->port.bridge);
	ret = devm_drm_dp_hpd_bridge_add(dev, ctx->port.bridge);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ctx->bridge = of_drm_find_and_get_bridge(&ctx->bridge_node);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->bridge);
	drm_bridge_hpd_enable(ctx->bridge, altmode_test_hpd, ctx);
	kunit_add_action_or_reset(test, altmode_test_put_bridge, ctx->bridge);
	ret = software_node_register_node_group(altmode_test_nodes);
	KUNIT_ASSERT_EQ(test, ret, 0);
	kunit_add_action_or_reset(test, altmode_test_remove_nodes, NULL);
	mux_desc.fwnode = software_node_fwnode(&altmode_test_sink);
	mux_desc.drvdata = ctx;
	switch_desc.fwnode = mux_desc.fwnode;
	switch_desc.drvdata = ctx;
	ctx->sw = typec_switch_register(dev, &switch_desc);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->sw);
	kunit_add_action_or_reset(test, altmode_test_remove_switch, ctx->sw);
	ctx->mux = typec_mux_register(dev, &mux_desc);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->mux);
	kunit_add_action_or_reset(test, altmode_test_remove_mux, ctx->mux);
	retimer_desc.fwnode = mux_desc.fwnode;
	retimer_desc.drvdata = ctx;
	ctx->retimer = typec_retimer_register(dev, &retimer_desc);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->retimer);
	kunit_add_action_or_reset(test, altmode_test_remove_retimer, ctx->retimer);
	ctx->port.typec_mux = fwnode_typec_mux_get(software_node_fwnode(&altmode_test_port_node));
	kunit_add_action_or_reset(test, altmode_test_put_handles, ctx);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->port.typec_mux);
	ctx->port.typec_retimer =
		fwnode_typec_retimer_get(software_node_fwnode(&altmode_test_port_node));
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->port.typec_retimer);
	ctx->port.typec_switch =
		fwnode_typec_switch_get(software_node_fwnode(&altmode_test_port_node));
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->port.typec_switch);
	return 0;
}

struct altmode_test_pin {
	const char *name;
	u8 pin;
	unsigned long mode;
	u32 conf;
};

/* The published Configure VDO carries one bit per pin assignment, not its index. */
static const struct altmode_test_pin altmode_test_pins[] = {
	{ "A", DP_PIN_ASSIGN_A, TYPEC_DP_STATE_A, 0x0100 },
	{ "B", DP_PIN_ASSIGN_B, TYPEC_DP_STATE_B, 0x0200 },
	{ "C", DP_PIN_ASSIGN_C, TYPEC_DP_STATE_C, 0x0400 },
	{ "D", DP_PIN_ASSIGN_D, TYPEC_DP_STATE_D, 0x0800 },
	{ "E", DP_PIN_ASSIGN_E, TYPEC_DP_STATE_E, 0x1000 },
	{ "F", DP_PIN_ASSIGN_F, TYPEC_DP_STATE_F, 0x2000 },
};

static void altmode_test_pin_desc(const struct altmode_test_pin *pin, char *desc)
{
	snprintf(desc, KUNIT_PARAM_DESC_SIZE, "%s", pin->name);
}
KUNIT_ARRAY_PARAM(altmode_test_pin, altmode_test_pins, altmode_test_pin_desc);

static void altmode_test_dp_notification(struct kunit *test)
{
	struct altmode_test_ctx *ctx = test->priv;
	const struct altmode_test_pin *pin = test->param_value;
	struct altmode_test_observation *seen[] = { &ctx->mux_seen, &ctx->retimer_seen };
	unsigned int i;

	pmic_glink_altmode_enable_dp(&ctx->altmode, &ctx->port, pin->pin, true, true);
	for (i = 0; i < ARRAY_SIZE(seen); i++) {
		KUNIT_EXPECT_EQ(test, seen[i]->calls, 1);
		KUNIT_EXPECT_EQ(test, seen[i]->svid, USB_TYPEC_DP_SID);
		KUNIT_EXPECT_EQ(test, seen[i]->mode, pin->mode);
		KUNIT_EXPECT_EQ(test, seen[i]->data.conf, pin->conf);
		KUNIT_EXPECT_EQ(test, seen[i]->data.status,
				DP_STATUS_ENABLED | DP_STATUS_HPD_STATE | DP_STATUS_IRQ_HPD);
	}
}

static void altmode_test_invalid_pins(struct kunit *test)
{
	struct altmode_test_ctx *ctx = test->priv;

	pmic_glink_altmode_enable_dp(&ctx->altmode, &ctx->port, 6, false, false);
	pmic_glink_altmode_enable_dp(&ctx->altmode, &ctx->port, 255, false, false);
	KUNIT_EXPECT_EQ(test, ctx->mux_seen.calls, 0);
	KUNIT_EXPECT_EQ(test, ctx->retimer_seen.calls, 0);
}

static void altmode_test_mux_error(struct kunit *test)
{
	struct altmode_test_ctx *ctx = test->priv;

	ctx->mux_error = -EIO;
	pmic_glink_altmode_enable_dp(&ctx->altmode, &ctx->port, DP_PIN_ASSIGN_C, true, false);
	KUNIT_EXPECT_EQ(test, ctx->mux_seen.calls, 1);
	KUNIT_EXPECT_EQ(test, ctx->retimer_seen.calls, 0);
}

static void altmode_test_worker_connected(struct kunit *test)
{
	struct altmode_test_ctx *ctx = test->priv;

	pmic_glink_altmode_worker(&ctx->port.work);
	KUNIT_EXPECT_EQ(test, ctx->hpd_calls, 1);
	KUNIT_EXPECT_EQ(test, ctx->hpd_status, connector_status_connected);
	KUNIT_EXPECT_EQ(test, ctx->ack_calls, 1);
}

static void altmode_test_worker_switch_error(struct kunit *test)
{
	struct altmode_test_ctx *ctx = test->priv;

	ctx->switch_error = -EIO;
	pmic_glink_altmode_worker(&ctx->port.work);
	KUNIT_EXPECT_EQ(test, ctx->mux_seen.calls, 0);
	KUNIT_EXPECT_EQ(test, ctx->retimer_seen.calls, 0);
	KUNIT_EXPECT_EQ(test, ctx->hpd_calls, 0);
	KUNIT_EXPECT_EQ(test, ctx->ack_calls, 1);
}

static void altmode_test_worker_mux_error(struct kunit *test)
{
	struct altmode_test_ctx *ctx = test->priv;

	ctx->mux_error = -EIO;
	pmic_glink_altmode_worker(&ctx->port.work);
	KUNIT_EXPECT_EQ(test, ctx->retimer_seen.calls, 0);
	KUNIT_EXPECT_EQ(test, ctx->hpd_calls, 0);
	KUNIT_EXPECT_EQ(test, ctx->ack_calls, 1);
}

static void altmode_test_worker_retimer_error(struct kunit *test)
{
	struct altmode_test_ctx *ctx = test->priv;

	ctx->retimer_error = -EIO;
	pmic_glink_altmode_worker(&ctx->port.work);
	KUNIT_EXPECT_EQ(test, ctx->hpd_calls, 0);
	KUNIT_EXPECT_EQ(test, ctx->ack_calls, 1);
}

static void altmode_test_disconnect_with_error(struct kunit *test)
{
	struct altmode_test_ctx *ctx = test->priv;

	ctx->switch_error = -EIO;
	ctx->mux_error = -EIO;
	ctx->port.hpd_state = 0;
	ctx->port.mux_ctrl = MUX_CTRL_STATE_NO_CONN;
	pmic_glink_altmode_worker(&ctx->port.work);
	KUNIT_EXPECT_EQ(test, ctx->hpd_calls, 1);
	KUNIT_EXPECT_EQ(test, ctx->hpd_status, connector_status_disconnected);
	KUNIT_EXPECT_EQ(test, ctx->ack_calls, 1);
}

static struct kunit_case altmode_test_cases[] = {
	KUNIT_CASE_PARAM(altmode_test_dp_notification, altmode_test_pin_gen_params),
	KUNIT_CASE(altmode_test_invalid_pins),
	KUNIT_CASE(altmode_test_mux_error),
	KUNIT_CASE(altmode_test_worker_connected),
	KUNIT_CASE(altmode_test_worker_switch_error),
	KUNIT_CASE(altmode_test_worker_mux_error),
	KUNIT_CASE(altmode_test_worker_retimer_error),
	KUNIT_CASE(altmode_test_disconnect_with_error),
	{ }
};

static struct kunit_suite altmode_test_suite = {
	.name = "pmic-glink-dp-notification",
	.init = altmode_test_init,
	.test_cases = altmode_test_cases,
};
kunit_test_suite(altmode_test_suite);
