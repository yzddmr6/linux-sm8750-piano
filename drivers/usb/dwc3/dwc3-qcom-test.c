// SPDX-License-Identifier: GPL-2.0-only
/* Included by the glue; only hardware-facing host/gadget operations are stubbed. */
#include <kunit/device.h>
#include <kunit/static_stub.h>
#include <kunit/test.h>
#include <linux/sizes.h>

struct dwc3_qcom_test {
	struct dwc3_qcom qcom;
	struct platform_device *xhci;
	unsigned int stops;
	unsigned int starts;
	int start_error;
};

static void dwc3_test_unregister(void *data)
{
	platform_device_unregister(data);
}

static void dwc3_test_disable_pm(void *data)
{
	struct device *dev = data;

	pm_runtime_put_noidle(dev);
	pm_runtime_disable(dev);
}

static int dwc3_test_host_start(struct dwc3 *dwc)
{
	struct dwc3_qcom_test *ctx = container_of(to_dwc3_qcom(dwc),
						struct dwc3_qcom_test, qcom);

	ctx->starts++;
	/* Model device_add succeeding while probing is blocked or has failed. */
	dwc->xhci = ctx->start_error ? NULL : ctx->xhci;
	return ctx->start_error;
}

static void dwc3_test_host_stop(struct dwc3 *dwc)
{
	struct dwc3_qcom_test *ctx = container_of(to_dwc3_qcom(dwc),
						struct dwc3_qcom_test, qcom);

	ctx->stops++;
	dwc->xhci = NULL;
}

static int dwc3_test_device_stop(struct dwc3 *dwc)
{
	struct dwc3_qcom_test *ctx = container_of(to_dwc3_qcom(dwc),
						struct dwc3_qcom_test, qcom);

	ctx->stops++;
	return 0;
}

static int dwc3_test_device_start(struct dwc3 *dwc)
{
	struct dwc3_qcom_test *ctx = container_of(to_dwc3_qcom(dwc),
						struct dwc3_qcom_test, qcom);

	ctx->starts++;
	return ctx->start_error;
}

static int dwc3_qcom_test_init(struct kunit *test)
{
	struct dwc3_qcom_test *ctx;
	struct dwc3_qcom *qcom;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	test->priv = ctx;
	qcom = &ctx->qcom;
	qcom->dev = kunit_device_register(test, "dwc3-qcom-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, qcom->dev);
	qcom->dwc.dev = qcom->dev;
	dev_set_drvdata(qcom->dev, &qcom->dwc);
	KUNIT_ASSERT_EQ(test, pm_runtime_set_active(qcom->dev), 0);
	pm_runtime_get_noresume(qcom->dev);
	pm_runtime_enable(qcom->dev);
	kunit_add_action_or_reset(test, dwc3_test_disable_pm, qcom->dev);
	mutex_init(&qcom->dwc.mutex);
	mutex_init(&qcom->dwc.connect_mutex);
	mutex_init(&qcom->pipe_clk_lock);
	init_completion(&qcom->mode_probe_done);
	complete_all(&qcom->mode_probe_done);
	qcom->mode_ready = true;
	qcom->mode_registered = true;
	qcom->qscratch_base = (void __iomem *)kunit_kzalloc(test, SZ_4K, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, qcom->qscratch_base);
	ctx->xhci = platform_device_register_simple("dwc3-unbound-test", -1, NULL, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->xhci);
	kunit_add_action_or_reset(test, dwc3_test_unregister, ctx->xhci);
	qcom->dwc.xhci = ctx->xhci;
	qcom->dwc.current_dr_role = DWC3_GCTL_PRTCAP_HOST;
	kunit_activate_static_stub(test, dwc3_host_init, dwc3_test_host_start);
	kunit_activate_static_stub(test, dwc3_host_exit, dwc3_test_host_stop);
	kunit_activate_static_stub(test, dwc3_gadget_prepare_phy_change, dwc3_test_device_stop);
	kunit_activate_static_stub(test, dwc3_gadget_resume, dwc3_test_device_start);
	return 0;
}

static void dwc3_test_sleep_keeps_xhci(struct kunit *test)
{
	struct dwc3_qcom_test *ctx = test->priv;
	struct dwc3_qcom *qcom = &ctx->qcom;
	int ret;

	KUNIT_ASSERT_EQ(test, dwc3_qcom_prepare(qcom->dev), 0);
	/* DRM's prepare may release DP before this controller's suspend. */
	KUNIT_ASSERT_FALSE(test, qcom->pm_suspended);
	ret = dwc3_qcom_phy_mode_prepare(qcom);
	KUNIT_EXPECT_EQ(test, ret, -EAGAIN);
	if (!ret)
		dwc3_qcom_phy_mode_complete(qcom, true);
	kunit_info(test, "prepare=%d host_stop=%u host_start=%u unbound=%d hcd_null=%d\n",
		   ret, ctx->stops, ctx->starts, !device_is_bound(&ctx->xhci->dev),
		   !platform_get_drvdata(ctx->xhci));
	KUNIT_EXPECT_EQ(test, ctx->stops, 0U);
	KUNIT_EXPECT_EQ(test, ctx->starts, 0U);
	KUNIT_EXPECT_PTR_EQ(test, qcom->dwc.xhci, ctx->xhci);
	KUNIT_EXPECT_TRUE(test, qcom->dp_only);
	KUNIT_EXPECT_TRUE(test, qcom->dwc.usb3_disabled);
	qcom->num_ports = 1;
	writel(PWR_EVNT_LPM_IN_L2_MASK, qcom->qscratch_base + pwr_evnt_irq_stat_reg[0]);
	/* Continue into the real wakeup-enabled suspend with a NULL HCD. */
	KUNIT_EXPECT_EQ(test, dwc3_qcom_suspend(qcom, true), 0);
}

static void dwc3_test_unbound_host(struct kunit *test)
{
	struct dwc3_qcom_test *ctx = test->priv;

	KUNIT_EXPECT_FALSE(test, dwc3_qcom_is_host(&ctx->qcom));
	/* Exercise the actual null-HCD access path with an unbound platform device. */
	KUNIT_EXPECT_EQ(test, dwc3_qcom_read_usb2_speed(&ctx->qcom, 0), USB_SPEED_UNKNOWN);
}

static void dwc3_test_failed_host_restart(struct kunit *test)
{
	struct dwc3_qcom_test *ctx = test->priv;
	struct dwc3 *dwc = &ctx->qcom.dwc;

	guard(mutex)(&dwc->mutex);
	KUNIT_ASSERT_EQ(test, dwc3_prepare_phy_change(dwc), 0);
	KUNIT_EXPECT_LT(test, dwc3_complete_phy_change(dwc), 0);
	KUNIT_EXPECT_PTR_EQ(test, dwc->xhci, NULL);
	ctx->start_error = -ENOMEM;
	KUNIT_ASSERT_EQ(test, dwc3_prepare_phy_change(dwc), 0);
	KUNIT_EXPECT_EQ(test, dwc3_complete_phy_change(dwc), -ENOMEM);
	KUNIT_EXPECT_PTR_EQ(test, dwc->xhci, NULL);
}

static void dwc3_test_failed_device_restart(struct kunit *test)
{
	struct dwc3_qcom_test *ctx = test->priv;
	struct dwc3 *dwc = &ctx->qcom.dwc;

	dwc->current_dr_role = DWC3_GCTL_PRTCAP_DEVICE;
	ctx->start_error = -ETIMEDOUT;
	guard(mutex)(&dwc->mutex);
	KUNIT_ASSERT_EQ(test, dwc3_prepare_phy_change(dwc), 0);
	KUNIT_EXPECT_EQ(test, dwc3_complete_phy_change(dwc), -ETIMEDOUT);
	ctx->start_error = 0;
	KUNIT_ASSERT_EQ(test, dwc3_prepare_phy_change(dwc), 0);
	KUNIT_EXPECT_EQ(test, dwc3_complete_phy_change(dwc), 0);
	KUNIT_EXPECT_EQ(test, ctx->starts, 2U);
	KUNIT_EXPECT_FALSE(test, mutex_is_locked(&dwc->connect_mutex));
}

static struct kunit_case dwc3_qcom_cases[] = {
	KUNIT_CASE(dwc3_test_sleep_keeps_xhci),
	KUNIT_CASE(dwc3_test_failed_host_restart),
	KUNIT_CASE(dwc3_test_failed_device_restart),
	KUNIT_CASE(dwc3_test_unbound_host),
	{}
};

static struct kunit_suite dwc3_qcom_suite = {
	.name = "dwc3-qcom-phy-mode",
	.init = dwc3_qcom_test_init,
	.test_cases = dwc3_qcom_cases,
};

kunit_test_suite(dwc3_qcom_suite);
