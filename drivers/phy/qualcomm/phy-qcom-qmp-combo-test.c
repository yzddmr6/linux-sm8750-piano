// SPDX-License-Identifier: GPL-2.0-only
/* Included by the driver to exercise its actual deferred mode transitions. */
#include <kunit/device.h>
#include <kunit/test.h>

struct qmp_combo_test {
	struct qmp_combo qmp;
	struct phy dp_phy;
	struct phy usb_phy;
	int usb_error;
	int common_error;
	unsigned int aux_inits;
	int prepare_error;
	int complete_error;
	unsigned int prepares;
	unsigned int completes;
	bool available;
};

static int qmp_combo_test_prepare(void *data)
{
	struct qmp_combo_test *ctx = data;

	ctx->prepares++;
	return ctx->prepare_error;
}

static int qmp_combo_test_complete(void *data, bool available)
{
	struct qmp_combo_test *ctx = data;

	ctx->completes++;
	ctx->available = available;
	return ctx->complete_error;
}

static const struct qmp_combo_usb_mode_ops qmp_combo_test_ops = {
	.prepare = qmp_combo_test_prepare,
	.complete = qmp_combo_test_complete,
};

static int qmp_combo_test_init(struct kunit *test)
{
	struct qmp_combo_test *ctx;
	struct qmp_combo *qmp;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	test->priv = ctx;
	qmp = &ctx->qmp;
	qmp->dev = kunit_device_register(test, "qmp-combo-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, qmp->dev);
	mutex_init(&qmp->phy_mutex);
	mutex_init(&qmp->typec_mutex);
	qmp->qmpphy_mode = QMPPHY_MODE_USB3_ONLY;
	qmp->orientation = TYPEC_ORIENTATION_NORMAL;
	qmp->requested_orientation = TYPEC_ORIENTATION_NORMAL;
	qmp->usb3_available = true;
	qmp->usb_mode_ops = &qmp_combo_test_ops;
	qmp->usb_mode_data = ctx;
	phy_set_drvdata(&ctx->dp_phy, qmp);
	phy_set_drvdata(&ctx->usb_phy, qmp);
	qmp->usb_phy = &ctx->usb_phy;
	return 0;
}

static int qmp_combo_test_request(struct qmp_combo *qmp, enum qmpphy_mode mode)
{
	guard(mutex)(&qmp->typec_mutex);
	return qmp_combo_request_mode(qmp, mode);
}

static void qmp_combo_test_dp_to_usb(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;

	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_DP_ONLY), 0);
	KUNIT_EXPECT_FALSE(test, ctx->available);
	qmp->dp_powered_on = true;

	/* SAFE, then a quick USB connection, must not publish PIPE early. */
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3_ONLY), 0);
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3_ONLY), 0);
	KUNIT_EXPECT_EQ(test, ctx->completes, 1U);
	KUNIT_EXPECT_EQ(test, qmp->qmpphy_mode, QMPPHY_MODE_DP_ONLY);
	KUNIT_EXPECT_TRUE(test, qmp->mode_pending);

	/* Opposite orientation is applied with the saved mode after DP stops. */
	qmp->requested_orientation = TYPEC_ORIENTATION_REVERSE;
	KUNIT_ASSERT_EQ(test, qmp_combo_dp_power_off(&ctx->dp_phy), 0);
	KUNIT_EXPECT_EQ(test, qmp->qmpphy_mode, QMPPHY_MODE_USB3_ONLY);
	KUNIT_EXPECT_EQ(test, qmp->orientation, TYPEC_ORIENTATION_REVERSE);
	KUNIT_EXPECT_FALSE(test, qmp->mode_pending);
	KUNIT_EXPECT_TRUE(test, ctx->available);
	KUNIT_EXPECT_EQ(test, ctx->prepares, 2U);
	KUNIT_EXPECT_EQ(test, ctx->completes, 2U);

	/* A second DP connection starts from the USB baseline. */
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_DP_ONLY), 0);
	KUNIT_EXPECT_FALSE(test, ctx->available);
	KUNIT_EXPECT_EQ(test, ctx->completes, 3U);
}

static void qmp_combo_test_latest_request(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;

	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_DP_ONLY), 0);
	qmp->dp_powered_on = true;
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3_ONLY), 0);
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3DP), 0);
	KUNIT_EXPECT_EQ(test, ctx->completes, 1U);
	KUNIT_ASSERT_EQ(test, qmp_combo_dp_power_off(&ctx->dp_phy), 0);
	KUNIT_EXPECT_EQ(test, qmp->qmpphy_mode, QMPPHY_MODE_USB3DP);
	KUNIT_EXPECT_TRUE(test, ctx->available);
	KUNIT_EXPECT_EQ(test, ctx->completes, 2U);
}

static void qmp_combo_test_prepare_failure(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;

	ctx->prepare_error = -EBUSY;
	KUNIT_EXPECT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_DP_ONLY), 0);
	KUNIT_EXPECT_EQ(test, qmp->qmpphy_mode, QMPPHY_MODE_DP_ONLY);
	KUNIT_EXPECT_FALSE(test, qmp->usb3_available);
	KUNIT_EXPECT_TRUE(test, qmp->mode_pending);
	KUNIT_EXPECT_EQ(test, ctx->completes, 0U);

	/* A disconnect supersedes the failed request, even at the same mode. */
	ctx->prepare_error = 0;
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3_ONLY), 0);
	KUNIT_EXPECT_FALSE(test, qmp->mode_pending);
	KUNIT_EXPECT_TRUE(test, ctx->available);
	KUNIT_EXPECT_EQ(test, ctx->prepares, 2U);
	KUNIT_EXPECT_EQ(test, ctx->completes, 1U);
}

static void qmp_combo_test_unchanged_mode(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;

	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(&ctx->qmp, QMPPHY_MODE_USB3_ONLY), 0);
	KUNIT_EXPECT_EQ(test, ctx->prepares, 0U);
	KUNIT_EXPECT_EQ(test, ctx->completes, 0U);
}

static void qmp_combo_test_release_failure(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;

	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_DP_ONLY), 0);
	qmp->dp_powered_on = true;
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3_ONLY), 0);
	ctx->prepare_error = -EBUSY;
	/* Failure to start USB must not retain the DP core's power reference. */
	KUNIT_EXPECT_EQ(test, qmp_combo_dp_power_off(&ctx->dp_phy), 0);
	KUNIT_EXPECT_FALSE(test, qmp->dp_powered_on);
	KUNIT_EXPECT_TRUE(test, qmp->mode_pending);
	KUNIT_EXPECT_EQ(test, ctx->completes, 1U);
	ctx->prepare_error = 0;
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3_ONLY), 0);
	KUNIT_EXPECT_TRUE(test, ctx->available);
}

static int qmp_test_common_init(struct qmp_combo *qmp, bool force)
{
	struct qmp_combo_test *ctx = container_of(qmp, struct qmp_combo_test, qmp);

	qmp->com_enabled = !ctx->common_error;
	return ctx->common_error;
}

static int qmp_test_common_exit(struct qmp_combo *qmp, bool force)
{
	qmp->com_enabled = false;
	return 0;
}

static int qmp_test_usb_on(struct phy *phy)
{
	struct qmp_combo *qmp = phy_get_drvdata(phy);
	struct qmp_combo_test *ctx = container_of(qmp, struct qmp_combo_test, qmp);

	return ctx->usb_error;
}

static int qmp_test_usb_off(struct phy *phy)
{
	return 0;
}

static void qmp_test_aux_init(struct qmp_combo *qmp)
{
	struct qmp_combo_test *ctx = container_of(qmp, struct qmp_combo_test, qmp);

	ctx->aux_inits++;
}

static void qmp_test_stub_resources(struct kunit *test)
{
	static const struct qmp_phy_cfg cfg = { .dp_aux_init = qmp_test_aux_init };
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;

	qmp->cfg = &cfg;
	qmp->init_count = 2;
	qmp->usb_init_count = 1;
	qmp->dp_init_count = 1;
	qmp->com_enabled = true;
	qmp->dp_dp_phy = (void __iomem *)kunit_kzalloc(test, SZ_4K, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, qmp->dp_dp_phy);
	kunit_activate_static_stub(test, qmp_combo_com_init, qmp_test_common_init);
	kunit_activate_static_stub(test, qmp_combo_com_exit, qmp_test_common_exit);
	kunit_activate_static_stub(test, qmp_combo_usb_power_on, qmp_test_usb_on);
	kunit_activate_static_stub(test, qmp_combo_usb_power_off, qmp_test_usb_off);
}

static void qmp_combo_test_usb_failure_keeps_aux(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;

	qmp_test_stub_resources(test);
	ctx->usb_error = -ETIMEDOUT;
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3DP), 0);
	KUNIT_EXPECT_EQ(test, ctx->aux_inits, 1U);
	KUNIT_EXPECT_FALSE(test, ctx->available);
	KUNIT_EXPECT_TRUE(test, qmp->mode_pending);
	KUNIT_EXPECT_EQ(test, ctx->completes, 1U);
	KUNIT_EXPECT_EQ(test, qmp->init_count, 2);

	/* A same-direction connection retries the saved request, not the old mode. */
	ctx->usb_error = 0;
	KUNIT_ASSERT_EQ(test, qmp_combo_request_orientation(qmp, TYPEC_ORIENTATION_NORMAL), 0);
	KUNIT_EXPECT_TRUE(test, ctx->available);
	KUNIT_EXPECT_FALSE(test, qmp->mode_pending);
}

static void qmp_combo_test_prepare_failure_keeps_aux(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;

	qmp_test_stub_resources(test);
	ctx->prepare_error = -EIO;
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3DP), 0);
	KUNIT_EXPECT_EQ(test, ctx->aux_inits, 1U);
	KUNIT_EXPECT_FALSE(test, qmp->usb3_available);
	KUNIT_EXPECT_EQ(test, ctx->completes, 0U);
}

static void qmp_combo_test_pending_orientation(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;
	enum typec_orientation orientations[] = {
		TYPEC_ORIENTATION_NORMAL, TYPEC_ORIENTATION_REVERSE,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(orientations); i++) {
		qmp->qmpphy_mode = QMPPHY_MODE_DP_ONLY;
		qmp->mode_pending = true;
		qmp->pending_mode = QMPPHY_MODE_USB3_ONLY;
		KUNIT_ASSERT_EQ(test, qmp_combo_request_orientation(qmp, orientations[i]), 0);
		KUNIT_EXPECT_EQ(test, qmp->qmpphy_mode, QMPPHY_MODE_USB3_ONLY);
		KUNIT_EXPECT_EQ(test, qmp->orientation, orientations[i]);
		KUNIT_EXPECT_FALSE(test, qmp->mode_pending);
	}
	KUNIT_EXPECT_EQ(test, ctx->completes, 2U);
}

static void qmp_combo_test_redundant_pending(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;

	qmp->mode_pending = true;
	qmp->pending_mode = QMPPHY_MODE_USB3_ONLY;
	KUNIT_ASSERT_EQ(test, qmp_combo_request_orientation(qmp, TYPEC_ORIENTATION_NORMAL), 0);
	KUNIT_EXPECT_EQ(test, ctx->prepares, 0U);
	KUNIT_EXPECT_FALSE(test, qmp->mode_pending);
}

static void qmp_combo_test_common_failure(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	union phy_configure_opts opts = {};

	qmp_test_stub_resources(test);
	ctx->common_error = -EIO;
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(&ctx->qmp, QMPPHY_MODE_USB3DP), -EIO);
	KUNIT_EXPECT_FALSE(test, ctx->qmp.com_enabled);
	KUNIT_EXPECT_EQ(test, ctx->aux_inits, 0U);
	KUNIT_EXPECT_EQ(test, ctx->completes, 1U);
	KUNIT_EXPECT_FALSE(test, ctx->available);
	/* These must return before any unclocked register access or cfg callback. */
	opts.dp.set_voltages = true;
	KUNIT_EXPECT_EQ(test, qmp_combo_dp_configure(&ctx->dp_phy, &opts), -EIO);
	KUNIT_EXPECT_EQ(test, qmp_combo_dp_calibrate(&ctx->dp_phy), -EIO);
	KUNIT_EXPECT_EQ(test, qmp_combo_dp_power_on(&ctx->dp_phy), -EIO);
}

static void qmp_combo_test_restart_failure(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;

	/* A failed device soft-connect or host probe must survive the no-op path. */
	ctx->complete_error = -EIO;
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3DP), 0);
	KUNIT_EXPECT_TRUE(test, qmp->mode_pending);
	ctx->complete_error = 0;
	KUNIT_ASSERT_EQ(test, qmp_combo_request_orientation(qmp, TYPEC_ORIENTATION_NORMAL), 0);
	KUNIT_EXPECT_EQ(test, ctx->completes, 2U);
	KUNIT_EXPECT_FALSE(test, qmp->mode_pending);
}

static void qmp_combo_test_sleep_defers_apply(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;

	ctx->prepare_error = -EAGAIN;
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_DP_ONLY), 0);
	KUNIT_EXPECT_EQ(test, qmp->qmpphy_mode, QMPPHY_MODE_USB3_ONLY);
	KUNIT_EXPECT_TRUE(test, qmp->mode_pending);
	KUNIT_EXPECT_EQ(test, ctx->completes, 0U);
	ctx->prepare_error = 0;
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, qmp->pending_mode), 0);
	KUNIT_EXPECT_EQ(test, qmp->qmpphy_mode, QMPPHY_MODE_DP_ONLY);
	KUNIT_EXPECT_EQ(test, ctx->completes, 1U);
	KUNIT_EXPECT_FALSE(test, qmp->mode_pending);
}

static void qmp_combo_test_deferred_lanes_limit_usb(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;

	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3DP), 0);
	qmp->dp_powered_on = true;
	/* A new peer is opposite to the DP lanes still owned by the old peer. */
	KUNIT_ASSERT_EQ(test, qmp_combo_request_orientation(qmp, TYPEC_ORIENTATION_REVERSE), 0);
	KUNIT_EXPECT_FALSE(test, ctx->available);
	KUNIT_EXPECT_EQ(test, qmp->orientation, TYPEC_ORIENTATION_NORMAL);
	KUNIT_EXPECT_EQ(test, qmp->qmpphy_mode, QMPPHY_MODE_USB3DP);
	KUNIT_EXPECT_EQ(test, ctx->completes, 2U);
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3_ONLY), 0);
	KUNIT_EXPECT_FALSE(test, ctx->available);
	KUNIT_EXPECT_EQ(test, ctx->completes, 2U);
	KUNIT_ASSERT_EQ(test, qmp_combo_dp_power_off(&ctx->dp_phy), 0);
	KUNIT_EXPECT_TRUE(test, ctx->available);
	KUNIT_EXPECT_EQ(test, qmp->orientation, TYPEC_ORIENTATION_REVERSE);
	KUNIT_EXPECT_EQ(test, qmp->qmpphy_mode, QMPPHY_MODE_USB3_ONLY);
	KUNIT_EXPECT_EQ(test, ctx->completes, 3U);
}

static void qmp_combo_test_sleep_cancel(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;

	ctx->prepare_error = -EAGAIN;
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_DP_ONLY), 0);
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3_ONLY), 0);
	KUNIT_EXPECT_TRUE(test, qmp->mode_pending);
	ctx->prepare_error = 0;
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, qmp->pending_mode), 0);
	/* Restore the consumer's PIPE even though the PHY itself never changed. */
	KUNIT_EXPECT_EQ(test, ctx->completes, 1U);
	KUNIT_EXPECT_TRUE(test, ctx->available);
	KUNIT_EXPECT_FALSE(test, qmp->mode_pending);
}

static void qmp_combo_test_deferred_cancel(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;

	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3DP), 0);
	qmp->dp_powered_on = true;
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_DP_ONLY), 0);
	KUNIT_EXPECT_FALSE(test, ctx->available);
	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3DP), 0);
	KUNIT_EXPECT_TRUE(test, ctx->available);
	KUNIT_EXPECT_EQ(test, qmp->qmpphy_mode, QMPPHY_MODE_USB3DP);
	KUNIT_EXPECT_TRUE(test, qmp->dp_powered_on);
}

static void qmp_combo_test_deferred_restart_failure(struct kunit *test)
{
	struct qmp_combo_test *ctx = test->priv;
	struct qmp_combo *qmp = &ctx->qmp;

	KUNIT_ASSERT_EQ(test, qmp_combo_test_request(qmp, QMPPHY_MODE_USB3DP), 0);
	qmp->dp_powered_on = true;
	/* An unbound replacement host returns -ENODEV from complete, not prepare. */
	ctx->complete_error = -ENODEV;
	KUNIT_ASSERT_EQ(test, qmp_combo_request_orientation(qmp, TYPEC_ORIENTATION_REVERSE), 0);
	KUNIT_EXPECT_EQ(test, ctx->completes, 2U);
	ctx->complete_error = 0;
	KUNIT_ASSERT_EQ(test, qmp_combo_request_orientation(qmp, TYPEC_ORIENTATION_REVERSE), 0);
	KUNIT_EXPECT_EQ(test, ctx->completes, 3U);
	KUNIT_EXPECT_FALSE(test, ctx->available);
	KUNIT_ASSERT_EQ(test, qmp_combo_dp_power_off(&ctx->dp_phy), 0);
	KUNIT_EXPECT_TRUE(test, ctx->available);
}

static struct kunit_case qmp_combo_test_cases[] = {
	KUNIT_CASE(qmp_combo_test_restart_failure),
	KUNIT_CASE(qmp_combo_test_sleep_defers_apply),
	KUNIT_CASE(qmp_combo_test_sleep_cancel),
	KUNIT_CASE(qmp_combo_test_deferred_lanes_limit_usb),
	KUNIT_CASE(qmp_combo_test_deferred_cancel),
	KUNIT_CASE(qmp_combo_test_deferred_restart_failure),
	KUNIT_CASE(qmp_combo_test_dp_to_usb),
	KUNIT_CASE(qmp_combo_test_latest_request),
	KUNIT_CASE(qmp_combo_test_prepare_failure),
	KUNIT_CASE(qmp_combo_test_unchanged_mode),
	KUNIT_CASE(qmp_combo_test_release_failure),
	KUNIT_CASE(qmp_combo_test_usb_failure_keeps_aux),
	KUNIT_CASE(qmp_combo_test_prepare_failure_keeps_aux),
	KUNIT_CASE(qmp_combo_test_pending_orientation),
	KUNIT_CASE(qmp_combo_test_redundant_pending),
	KUNIT_CASE(qmp_combo_test_common_failure),
	{}
};

static struct kunit_suite qmp_combo_test_suite = {
	.name = "qmp-combo-typec",
	.init = qmp_combo_test_init,
	.test_cases = qmp_combo_test_cases,
};

kunit_test_suite(qmp_combo_test_suite);
