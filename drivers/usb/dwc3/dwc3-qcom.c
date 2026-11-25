// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * Inspired by dwc3-of-simple.c
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/irq.h>
#include <linux/of_clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interconnect.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-qcom-qmp-combo.h>
#include <linux/usb/of.h>
#include <linux/reset.h>
#include <linux/iopoll.h>
#include <linux/usb/hcd.h>
#include <linux/usb.h>
#include "core.h"
#include "glue.h"

/* USB QSCRATCH Hardware registers */
#define QSCRATCH_HS_PHY_CTRL			0x10
#define UTMI_OTG_VBUS_VALID			BIT(20)
#define SW_SESSVLD_SEL				BIT(28)

#define QSCRATCH_SS_PHY_CTRL			0x30
#define LANE0_PWR_PRESENT			BIT(24)

#define QSCRATCH_GENERAL_CFG			0x08
#define PIPE_UTMI_CLK_SEL			BIT(0)
#define PIPE3_PHYSTATUS_SW			BIT(3)
#define PIPE_UTMI_CLK_DIS			BIT(8)

#define PWR_EVNT_LPM_IN_L2_MASK			BIT(4)
#define PWR_EVNT_LPM_OUT_L2_MASK		BIT(5)

#define SDM845_QSCRATCH_BASE_OFFSET		0xf8800
#define SDM845_QSCRATCH_SIZE			0x400
#define SDM845_DWC3_CORE_SIZE			0xcd00

/* Interconnect path bandwidths in MBps */
#define USB_MEMORY_AVG_HS_BW MBps_to_icc(240)
#define USB_MEMORY_PEAK_HS_BW MBps_to_icc(700)
#define USB_MEMORY_AVG_SS_BW  MBps_to_icc(1000)
#define USB_MEMORY_PEAK_SS_BW MBps_to_icc(2500)
#define APPS_USB_AVG_BW 0
#define APPS_USB_PEAK_BW MBps_to_icc(40)

/* Qualcomm SoCs with multiport support has up to 4 ports */
#define QSCRATCH_EXTRA_INP_REG	0x1e4
#define EXTRA_INP_SS_DISABLE	BIT(5)

#define DWC3_QCOM_MAX_PORTS	4

static const u32 pwr_evnt_irq_stat_reg[DWC3_QCOM_MAX_PORTS] = {
	0x58,
	0x1dc,
	0x228,
	0x238,
};

struct dwc3_qcom_port {
	int			qusb2_phy_irq;
	int			dp_hs_phy_irq;
	int			dm_hs_phy_irq;
	int			ss_phy_irq;
	enum usb_device_speed	usb2_speed;
};

struct dwc3_qcom {
	struct device		*dev;
	void __iomem		*qscratch_base;
	struct dwc3		dwc;
	struct clk_bulk_data	*clks;
	int			num_clocks;
	struct reset_control	*resets;
	struct dwc3_qcom_port	ports[DWC3_QCOM_MAX_PORTS];
	u8			num_ports;

	enum usb_dr_mode	mode;
	bool			is_suspended;
	bool			pm_suspended;
	bool			ignore_pipe_clk;
	bool			dp_only;
	bool			utmi_pipe_clk;
	/* Serializes PHY changes with runtime and system PM. */
	struct mutex		pipe_clk_lock;
	struct completion	mode_probe_done;
	bool			mode_registered;
	bool			mode_ready;
	bool			mode_pm_ref;
	bool			mode_quiesced;
	struct icc_path		*icc_path_ddr;
	struct icc_path		*icc_path_apps;

	enum usb_role		current_role;
};

#define to_dwc3_qcom(d) container_of((d), struct dwc3_qcom, dwc)

static inline void dwc3_qcom_setbits(void __iomem *base, u32 offset, u32 val)
{
	u32 reg;

	reg = readl(base + offset);
	reg |= val;
	writel(reg, base + offset);

	/* ensure that above write is through */
	readl(base + offset);
}

static inline void dwc3_qcom_clrbits(void __iomem *base, u32 offset, u32 val)
{
	u32 reg;

	reg = readl(base + offset);
	reg &= ~val;
	writel(reg, base + offset);

	/* ensure that above write is through */
	readl(base + offset);
}

static void dwc3_qcom_vbus_override_enable(struct dwc3_qcom *qcom, bool enable)
{
	if (enable) {
		dwc3_qcom_setbits(qcom->qscratch_base, QSCRATCH_SS_PHY_CTRL,
				  LANE0_PWR_PRESENT);
		dwc3_qcom_setbits(qcom->qscratch_base, QSCRATCH_HS_PHY_CTRL,
				  UTMI_OTG_VBUS_VALID | SW_SESSVLD_SEL);
	} else {
		dwc3_qcom_clrbits(qcom->qscratch_base, QSCRATCH_SS_PHY_CTRL,
				  LANE0_PWR_PRESENT);
		dwc3_qcom_clrbits(qcom->qscratch_base, QSCRATCH_HS_PHY_CTRL,
				  UTMI_OTG_VBUS_VALID | SW_SESSVLD_SEL);
	}
}

static int dwc3_qcom_interconnect_enable(struct dwc3_qcom *qcom)
{
	int ret;

	ret = icc_enable(qcom->icc_path_ddr);
	if (ret)
		return ret;

	ret = icc_enable(qcom->icc_path_apps);
	if (ret)
		icc_disable(qcom->icc_path_ddr);

	return ret;
}

static int dwc3_qcom_interconnect_disable(struct dwc3_qcom *qcom)
{
	int ret;

	ret = icc_disable(qcom->icc_path_ddr);
	if (ret)
		return ret;

	ret = icc_disable(qcom->icc_path_apps);
	if (ret)
		icc_enable(qcom->icc_path_ddr);

	return ret;
}

/**
 * dwc3_qcom_interconnect_init() - Get interconnect path handles
 * and set bandwidth.
 * @qcom:			Pointer to the concerned usb core.
 *
 */
static int dwc3_qcom_interconnect_init(struct dwc3_qcom *qcom)
{
	enum usb_device_speed max_speed;
	struct device *dev = qcom->dev;
	int ret;

	qcom->icc_path_ddr = of_icc_get(dev, "usb-ddr");
	if (IS_ERR(qcom->icc_path_ddr)) {
		return dev_err_probe(dev, PTR_ERR(qcom->icc_path_ddr),
				     "failed to get usb-ddr path\n");
	}

	qcom->icc_path_apps = of_icc_get(dev, "apps-usb");
	if (IS_ERR(qcom->icc_path_apps)) {
		ret = dev_err_probe(dev, PTR_ERR(qcom->icc_path_apps),
				    "failed to get apps-usb path\n");
		goto put_path_ddr;
	}

	max_speed = usb_get_maximum_speed(qcom->dwc.dev);
	if (max_speed >= USB_SPEED_SUPER || max_speed == USB_SPEED_UNKNOWN) {
		ret = icc_set_bw(qcom->icc_path_ddr,
				USB_MEMORY_AVG_SS_BW, USB_MEMORY_PEAK_SS_BW);
	} else {
		ret = icc_set_bw(qcom->icc_path_ddr,
				USB_MEMORY_AVG_HS_BW, USB_MEMORY_PEAK_HS_BW);
	}
	if (ret) {
		dev_err(dev, "failed to set bandwidth for usb-ddr path: %d\n", ret);
		goto put_path_apps;
	}

	ret = icc_set_bw(qcom->icc_path_apps, APPS_USB_AVG_BW, APPS_USB_PEAK_BW);
	if (ret) {
		dev_err(dev, "failed to set bandwidth for apps-usb path: %d\n", ret);
		goto put_path_apps;
	}

	return 0;

put_path_apps:
	icc_put(qcom->icc_path_apps);
put_path_ddr:
	icc_put(qcom->icc_path_ddr);
	return ret;
}

/**
 * dwc3_qcom_interconnect_exit() - Release interconnect path handles
 * @qcom:			Pointer to the concerned usb core.
 *
 * This function is used to release interconnect path handle.
 */
static void dwc3_qcom_interconnect_exit(struct dwc3_qcom *qcom)
{
	icc_put(qcom->icc_path_ddr);
	icc_put(qcom->icc_path_apps);
}

/* Only usable in contexts where the role can not change. */
static bool dwc3_qcom_is_host(struct dwc3_qcom *qcom)
{
	return qcom->dwc.xhci;
}

static enum usb_device_speed dwc3_qcom_read_usb2_speed(struct dwc3_qcom *qcom, int port_index)
{
	struct usb_device *udev;
	struct usb_hcd __maybe_unused *hcd;
	struct dwc3 *dwc = &qcom->dwc;

	/*
	 * FIXME: Fix this layering violation.
	 */
	hcd = platform_get_drvdata(dwc->xhci);

#ifdef CONFIG_USB
	udev = usb_hub_find_child(hcd->self.root_hub, port_index + 1);
#else
	udev = NULL;
#endif
	if (!udev)
		return USB_SPEED_UNKNOWN;

	return udev->speed;
}

static void dwc3_qcom_enable_wakeup_irq(int irq, unsigned int polarity)
{
	if (!irq)
		return;

	if (polarity)
		irq_set_irq_type(irq, polarity);

	enable_irq(irq);
	enable_irq_wake(irq);
}

static void dwc3_qcom_disable_wakeup_irq(int irq)
{
	if (!irq)
		return;

	disable_irq_wake(irq);
	disable_irq_nosync(irq);
}

static void dwc3_qcom_disable_port_interrupts(struct dwc3_qcom_port *port)
{
	dwc3_qcom_disable_wakeup_irq(port->qusb2_phy_irq);

	if (port->usb2_speed == USB_SPEED_LOW) {
		dwc3_qcom_disable_wakeup_irq(port->dm_hs_phy_irq);
	} else if ((port->usb2_speed == USB_SPEED_HIGH) ||
			(port->usb2_speed == USB_SPEED_FULL)) {
		dwc3_qcom_disable_wakeup_irq(port->dp_hs_phy_irq);
	} else {
		dwc3_qcom_disable_wakeup_irq(port->dp_hs_phy_irq);
		dwc3_qcom_disable_wakeup_irq(port->dm_hs_phy_irq);
	}

	dwc3_qcom_disable_wakeup_irq(port->ss_phy_irq);
}

static void dwc3_qcom_enable_port_interrupts(struct dwc3_qcom_port *port)
{
	dwc3_qcom_enable_wakeup_irq(port->qusb2_phy_irq, 0);

	/*
	 * Configure DP/DM line interrupts based on the USB2 device attached to
	 * the root hub port. When HS/FS device is connected, configure the DP line
	 * as falling edge to detect both disconnect and remote wakeup scenarios. When
	 * LS device is connected, configure DM line as falling edge to detect both
	 * disconnect and remote wakeup. When no device is connected, configure both
	 * DP and DM lines as rising edge to detect HS/HS/LS device connect scenario.
	 */

	if (port->usb2_speed == USB_SPEED_LOW) {
		dwc3_qcom_enable_wakeup_irq(port->dm_hs_phy_irq,
					    IRQ_TYPE_EDGE_FALLING);
	} else if ((port->usb2_speed == USB_SPEED_HIGH) ||
			(port->usb2_speed == USB_SPEED_FULL)) {
		dwc3_qcom_enable_wakeup_irq(port->dp_hs_phy_irq,
					    IRQ_TYPE_EDGE_FALLING);
	} else {
		dwc3_qcom_enable_wakeup_irq(port->dp_hs_phy_irq,
					    IRQ_TYPE_EDGE_RISING);
		dwc3_qcom_enable_wakeup_irq(port->dm_hs_phy_irq,
					    IRQ_TYPE_EDGE_RISING);
	}

	dwc3_qcom_enable_wakeup_irq(port->ss_phy_irq, 0);
}

static void dwc3_qcom_disable_interrupts(struct dwc3_qcom *qcom)
{
	int i;

	for (i = 0; i < qcom->num_ports; i++)
		dwc3_qcom_disable_port_interrupts(&qcom->ports[i]);
}

static void dwc3_qcom_enable_interrupts(struct dwc3_qcom *qcom)
{
	int i;

	for (i = 0; i < qcom->num_ports; i++)
		dwc3_qcom_enable_port_interrupts(&qcom->ports[i]);
}

static int dwc3_qcom_suspend(struct dwc3_qcom *qcom, bool wakeup)
{
	u32 val;
	int i, ret;

	if (qcom->is_suspended)
		return 0;

	for (i = 0; i < qcom->num_ports; i++) {
		val = readl(qcom->qscratch_base + pwr_evnt_irq_stat_reg[i]);
		if (!(val & PWR_EVNT_LPM_IN_L2_MASK))
			dev_err(qcom->dev, "port-%d HS-PHY not in L2\n", i + 1);
	}
	clk_bulk_disable_unprepare(qcom->num_clocks, qcom->clks);

	ret = dwc3_qcom_interconnect_disable(qcom);
	if (ret)
		dev_warn(qcom->dev, "failed to disable interconnect: %d\n", ret);

	/*
	 * The role is stable during suspend as role switching is done from a
	 * freezable workqueue.
	 */
	if (dwc3_qcom_is_host(qcom) && wakeup) {
		for (i = 0; i < qcom->num_ports; i++)
			qcom->ports[i].usb2_speed = dwc3_qcom_read_usb2_speed(qcom, i);
		dwc3_qcom_enable_interrupts(qcom);
	}

	qcom->is_suspended = true;

	return 0;
}

static int dwc3_qcom_resume(struct dwc3_qcom *qcom, bool wakeup)
{
	int ret;
	int i;

	if (!qcom->is_suspended)
		return 0;

	if (dwc3_qcom_is_host(qcom) && wakeup)
		dwc3_qcom_disable_interrupts(qcom);

	ret = clk_bulk_prepare_enable(qcom->num_clocks, qcom->clks);
	if (ret < 0)
		return ret;

	ret = dwc3_qcom_interconnect_enable(qcom);
	if (ret)
		dev_warn(qcom->dev, "failed to enable interconnect: %d\n", ret);

	/* Clear existing events from PHY related to L2 in/out */
	for (i = 0; i < qcom->num_ports; i++) {
		dwc3_qcom_setbits(qcom->qscratch_base,
				  pwr_evnt_irq_stat_reg[i],
				  PWR_EVNT_LPM_IN_L2_MASK | PWR_EVNT_LPM_OUT_L2_MASK);
	}

	qcom->is_suspended = false;

	return 0;
}

static irqreturn_t qcom_dwc3_resume_irq(int irq, void *data)
{
	struct dwc3_qcom *qcom = data;
	struct dwc3 *dwc = &qcom->dwc;

	/* If pm_suspended then let pm_resume take care of resuming h/w */
	if (qcom->pm_suspended)
		return IRQ_HANDLED;

	/*
	 * This is safe as role switching is done from a freezable workqueue
	 * and the wakeup interrupts are disabled as part of resume.
	 */
	if (dwc3_qcom_is_host(qcom))
		pm_runtime_resume(&dwc->xhci->dev);

	return IRQ_HANDLED;
}

static void dwc3_qcom_select_utmi_clk(struct dwc3_qcom *qcom, bool utmi)
{
	dwc3_qcom_setbits(qcom->qscratch_base, QSCRATCH_GENERAL_CFG,
			  PIPE_UTMI_CLK_DIS);

	usleep_range(100, 1000);

	if (utmi)
		dwc3_qcom_setbits(qcom->qscratch_base, QSCRATCH_GENERAL_CFG,
				  PIPE_UTMI_CLK_SEL | PIPE3_PHYSTATUS_SW);
	else
		dwc3_qcom_clrbits(qcom->qscratch_base, QSCRATCH_GENERAL_CFG,
				  PIPE_UTMI_CLK_SEL | PIPE3_PHYSTATUS_SW);

	usleep_range(100, 1000);

	dwc3_qcom_clrbits(qcom->qscratch_base, QSCRATCH_GENERAL_CFG,
			  PIPE_UTMI_CLK_DIS);

	qcom->utmi_pipe_clk = utmi;
}

static void dwc3_qcom_apply_usb_mode(struct dwc3_qcom *qcom)
{
	bool utmi = qcom->ignore_pipe_clk || qcom->dp_only;
	u32 reg;

	qcom->dwc.usb3_disabled = utmi;
	if (qcom->mode_registered) {
		/* Stop SuperSpeed training in host mode as well as gadget mode. */
		if (utmi)
			dwc3_qcom_setbits(qcom->qscratch_base, QSCRATCH_EXTRA_INP_REG,
					  EXTRA_INP_SS_DISABLE);
		else
			dwc3_qcom_clrbits(qcom->qscratch_base, QSCRATCH_EXTRA_INP_REG,
					  EXTRA_INP_SS_DISABLE);
	}

	/* QSCRATCH may lose its contents during suspend. */
	reg = readl(qcom->qscratch_base + QSCRATCH_GENERAL_CFG);
	if (!!(reg & PIPE_UTMI_CLK_SEL) != utmi)
		dwc3_qcom_select_utmi_clk(qcom, utmi);
}

static int dwc3_qcom_phy_mode_prepare(void *data)
{
	struct dwc3_qcom *qcom = data;
	struct dwc3 *dwc = &qcom->dwc;
	int ret;

	/* The PHY callback can arrive as soon as registration completes. */
	wait_for_completion(&qcom->mode_probe_done);
	if (!qcom->mode_ready)
		return -ENODEV;

	/* QMP holds only its Type-C lock, never its PHY operations lock. */
	mutex_lock(&dwc->mutex);
	qcom->mode_pm_ref = false;
	qcom->mode_quiesced = false;
	if (!qcom->pm_suspended) {
		ret = pm_runtime_resume_and_get(qcom->dev);
		if (ret < 0) {
			/* The failed resume left USB stopped. Resume it on UTMI. */
			scoped_guard(mutex, &qcom->pipe_clk_lock) {
				qcom->dp_only = true;
				dwc->usb3_disabled = true;
			}
			goto unlock_role;
		}
		qcom->mode_pm_ref = true;
	}

	mutex_lock(&qcom->pipe_clk_lock);
	if (qcom->mode_ready && !qcom->pm_suspended) {
		ret = dwc3_prepare_phy_change(dwc);
		if (ret)
			goto unlock_pipe;
		qcom->mode_quiesced = true;

		/* Keep register access clocked while QMP resets its common block. */
		dwc3_qcom_select_utmi_clk(qcom, true);
	}

	return 0;

unlock_pipe:
	mutex_unlock(&qcom->pipe_clk_lock);
	if (qcom->mode_pm_ref)
		pm_runtime_put_autosuspend(qcom->dev);
unlock_role:
	mutex_unlock(&dwc->mutex);
	return ret;
}

static void dwc3_qcom_phy_mode_complete(void *data, bool usb3_available)
{
	struct dwc3_qcom *qcom = data;
	struct dwc3 *dwc = &qcom->dwc;
	int ret;

	qcom->dp_only = !usb3_available;
	dwc->usb3_disabled = qcom->ignore_pipe_clk || qcom->dp_only;
	if (qcom->mode_quiesced) {
		dwc3_qcom_apply_usb_mode(qcom);
		ret = dwc3_complete_phy_change(dwc);
		if (ret)
			dev_err(qcom->dev, "failed to restart USB after PHY change: %d\n", ret);
	} else if (qcom->pm_suspended && dwc->current_dr_role == DWC3_GCTL_PRTCAP_HOST) {
		/* xHCI is already halted by system PM; re-enumerate on resume. */
		dwc->xhci_power_lost = 1;
	}

	mutex_unlock(&qcom->pipe_clk_lock);
	if (qcom->mode_pm_ref)
		pm_runtime_put_autosuspend(qcom->dev);
	mutex_unlock(&dwc->mutex);
}

static const struct qmp_combo_usb_mode_ops dwc3_qcom_usb_mode_ops = {
	.prepare = dwc3_qcom_phy_mode_prepare,
	.complete = dwc3_qcom_phy_mode_complete,
};

static int dwc3_qcom_register_phy_mode(struct dwc3 *dwc)
{
	struct dwc3_qcom *qcom = to_dwc3_qcom(dwc);
	bool usb3_available;
	int ret;

	if (dwc->num_usb3_ports != 1) {
		dwc3_qcom_apply_usb_mode(qcom);
		return 0;
	}

	ret = qmp_combo_set_usb_mode_ops(dwc->usb3_generic_phy[0],
					 &dwc3_qcom_usb_mode_ops, qcom, &usb3_available);
	if (ret == -EOPNOTSUPP) {
		dwc3_qcom_apply_usb_mode(qcom);
		return 0;
	}
	if (ret)
		return ret;

	qcom->mode_registered = true;
	qcom->dp_only = !usb3_available;
	dwc3_qcom_apply_usb_mode(qcom);
	return 0;
}

static void dwc3_qcom_unregister_phy_mode(struct dwc3_qcom *qcom)
{
	if (qcom->mode_registered) {
		qmp_combo_set_usb_mode_ops(qcom->dwc.usb3_generic_phy[0], NULL, NULL, NULL);
		qcom->mode_registered = false;
	}
}

static int dwc3_qcom_request_irq(struct dwc3_qcom *qcom, int irq,
				 const char *name)
{
	int ret;

	/* Keep wakeup interrupts disabled until suspend */
	ret = devm_request_threaded_irq(qcom->dev, irq, NULL,
					qcom_dwc3_resume_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					name, qcom);
	if (ret)
		dev_err(qcom->dev, "failed to request irq %s: %d\n", name, ret);

	return ret;
}

static int dwc3_qcom_setup_port_irq(struct dwc3_qcom *qcom,
				    struct platform_device *pdev,
				    int port_index, bool is_multiport)
{
	const char *irq_name;
	int irq;
	int ret;

	if (is_multiport)
		irq_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "dp_hs_phy_%d", port_index + 1);
	else
		irq_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "dp_hs_phy_irq");
	if (!irq_name)
		return -ENOMEM;

	irq = platform_get_irq_byname_optional(pdev, irq_name);
	if (irq > 0) {
		ret = dwc3_qcom_request_irq(qcom, irq, irq_name);
		if (ret)
			return ret;
		qcom->ports[port_index].dp_hs_phy_irq = irq;
	}

	if (is_multiport)
		irq_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "dm_hs_phy_%d", port_index + 1);
	else
		irq_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "dm_hs_phy_irq");
	if (!irq_name)
		return -ENOMEM;

	irq = platform_get_irq_byname_optional(pdev, irq_name);
	if (irq > 0) {
		ret = dwc3_qcom_request_irq(qcom, irq, irq_name);
		if (ret)
			return ret;
		qcom->ports[port_index].dm_hs_phy_irq = irq;
	}

	if (is_multiport)
		irq_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "ss_phy_%d", port_index + 1);
	else
		irq_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "ss_phy_irq");
	if (!irq_name)
		return -ENOMEM;

	irq = platform_get_irq_byname_optional(pdev, irq_name);
	if (irq > 0) {
		ret = dwc3_qcom_request_irq(qcom, irq, irq_name);
		if (ret)
			return ret;
		qcom->ports[port_index].ss_phy_irq = irq;
	}

	if (is_multiport)
		return 0;

	irq = platform_get_irq_byname_optional(pdev, "qusb2_phy");
	if (irq > 0) {
		ret = dwc3_qcom_request_irq(qcom, irq, "qusb2_phy");
		if (ret)
			return ret;
		qcom->ports[port_index].qusb2_phy_irq = irq;
	}

	return 0;
}

static int dwc3_qcom_find_num_ports(struct platform_device *pdev)
{
	char irq_name[14];
	int port_num;
	int irq;

	irq = platform_get_irq_byname_optional(pdev, "dp_hs_phy_1");
	if (irq < 0)
		return 1;

	for (port_num = 2; port_num <= DWC3_QCOM_MAX_PORTS; port_num++) {
		sprintf(irq_name, "dp_hs_phy_%d", port_num);

		irq = platform_get_irq_byname_optional(pdev, irq_name);
		if (irq < 0)
			return port_num - 1;
	}

	return DWC3_QCOM_MAX_PORTS;
}

static int dwc3_qcom_setup_irq(struct dwc3_qcom *qcom, struct platform_device *pdev)
{
	bool is_multiport;
	int ret;
	int i;

	qcom->num_ports = dwc3_qcom_find_num_ports(pdev);
	is_multiport = (qcom->num_ports > 1);

	for (i = 0; i < qcom->num_ports; i++) {
		ret = dwc3_qcom_setup_port_irq(qcom, pdev, i, is_multiport);
		if (ret)
			return ret;
	}

	return 0;
}

static void dwc3_qcom_set_role_notifier(struct dwc3 *dwc, enum usb_role next_role)
{
	struct dwc3_qcom *qcom = to_dwc3_qcom(dwc);

	if (qcom->current_role == next_role)
		return;

	if (pm_runtime_resume_and_get(qcom->dev)) {
		dev_dbg(qcom->dev, "Failed to resume device\n");
		return;
	}

	if (qcom->current_role == USB_ROLE_DEVICE)
		dwc3_qcom_vbus_override_enable(qcom, false);
	else if (qcom->current_role != USB_ROLE_DEVICE)
		dwc3_qcom_vbus_override_enable(qcom, true);

	pm_runtime_mark_last_busy(qcom->dev);
	pm_runtime_put_sync(qcom->dev);

	/* Role changes are serialized by dwc->mutex in drd_work */
	qcom->current_role = next_role;
}

static void dwc3_qcom_run_stop_notifier(struct dwc3 *dwc, bool is_on)
{
	struct dwc3_qcom *qcom = to_dwc3_qcom(dwc);

	/*
	 * When autosuspend is enabled and controller goes to suspend
	 * after removing UDC from userspace, the next UDC write needs
	 * setting of QSCRATCH VBUS_VALID to "1" to generate a connect
	 * done event.
	 */
	if (!is_on)
		return;

	dwc3_qcom_vbus_override_enable(qcom, true);
	pm_runtime_mark_last_busy(qcom->dev);
}

static struct dwc3_glue_ops dwc3_qcom_glue_ops = {
	.pre_set_role	= dwc3_qcom_set_role_notifier,
	.pre_run_stop	= dwc3_qcom_run_stop_notifier,
	.post_phy_registration = dwc3_qcom_register_phy_mode,
};

static int dwc3_qcom_probe(struct platform_device *pdev)
{
	struct dwc3_probe_data	probe_data = {};
	struct device		*dev = &pdev->dev;
	struct dwc3_qcom	*qcom;
	struct resource		res;
	struct resource		*r;
	int			ret;
	bool			wakeup_source;

	qcom = devm_kzalloc_obj(&pdev->dev, *qcom);
	if (!qcom)
		return -ENOMEM;

	qcom->dev = &pdev->dev;

	qcom->resets = devm_reset_control_array_get_optional_exclusive(dev);
	if (IS_ERR(qcom->resets)) {
		return dev_err_probe(&pdev->dev, PTR_ERR(qcom->resets),
				     "failed to get resets\n");
	}

	ret = devm_clk_bulk_get_all(&pdev->dev, &qcom->clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get clocks\n");
	qcom->num_clocks = ret;

	ret = reset_control_assert(qcom->resets);
	if (ret) {
		dev_err(&pdev->dev, "failed to assert resets, err=%d\n", ret);
		return ret;
	}

	usleep_range(10, 1000);

	ret = reset_control_deassert(qcom->resets);
	if (ret) {
		dev_err(&pdev->dev, "failed to deassert resets, err=%d\n", ret);
		return ret;
	}

	ret = clk_bulk_prepare_enable(qcom->num_clocks, qcom->clks);
	if (ret < 0)
		return ret;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		ret = -EINVAL;
		goto clk_disable;
	}
	res = *r;
	res.end = res.start + SDM845_QSCRATCH_BASE_OFFSET;

	qcom->qscratch_base = devm_ioremap(dev, res.end, SDM845_QSCRATCH_SIZE);
	if (!qcom->qscratch_base) {
		dev_err(dev, "failed to map qscratch region\n");
		ret = -ENOMEM;
		goto clk_disable;
	}

	ret = dwc3_qcom_setup_irq(qcom, pdev);
	if (ret) {
		dev_err(dev, "failed to setup IRQs, err=%d\n", ret);
		goto clk_disable;
	}

	/*
	 * Disable pipe_clk requirement if specified. Used when dwc3
	 * operates without SSPHY and only HS/FS/LS modes are supported.
	 */
	qcom->ignore_pipe_clk =
		device_property_read_bool(dev, "qcom,select-utmi-as-pipe-clk");
	/* The PHY may already be in DP-only mode before USB has probed. */
	dwc3_qcom_select_utmi_clk(qcom, true);

	qcom->mode = usb_get_dr_mode(dev);

	if (qcom->mode == USB_DR_MODE_HOST) {
		qcom->current_role = USB_ROLE_HOST;
	} else if (qcom->mode == USB_DR_MODE_PERIPHERAL) {
		qcom->current_role = USB_ROLE_DEVICE;
		dwc3_qcom_vbus_override_enable(qcom, true);
	} else {
		if ((device_property_read_bool(dev, "usb-role-switch")) &&
		    (usb_get_role_switch_default_mode(dev) == USB_DR_MODE_HOST))
			qcom->current_role = USB_ROLE_HOST;
		else
			qcom->current_role = USB_ROLE_DEVICE;
	}

	mutex_init(&qcom->pipe_clk_lock);
	init_completion(&qcom->mode_probe_done);
	qcom->dwc.glue_ops = &dwc3_qcom_glue_ops;

	qcom->dwc.dev = dev;
	probe_data.dwc = &qcom->dwc;
	probe_data.res = &res;
	probe_data.ignore_clocks_and_resets = true;
	probe_data.properties = DWC3_DEFAULT_PROPERTIES;
	ret = dwc3_core_probe(&probe_data);
	qcom->mode_ready = !ret;
	complete_all(&qcom->mode_probe_done);
	if (ret)  {
		ret = dev_err_probe(dev, ret, "failed to register DWC3 Core\n");
		goto clk_disable;
	}

	ret = dwc3_qcom_interconnect_init(qcom);
	if (ret)
		goto remove_core;

	wakeup_source = of_property_read_bool(dev->of_node, "wakeup-source");
	device_init_wakeup(&pdev->dev, wakeup_source);

	qcom->is_suspended = false;

	return 0;

remove_core:
	dwc3_qcom_unregister_phy_mode(qcom);
	dwc3_core_remove(&qcom->dwc);
clk_disable:
	dwc3_qcom_unregister_phy_mode(qcom);
	clk_bulk_disable_unprepare(qcom->num_clocks, qcom->clks);

	return ret;
}

static void dwc3_qcom_remove(struct platform_device *pdev)
{
	struct dwc3 *dwc = platform_get_drvdata(pdev);
	struct dwc3_qcom *qcom = to_dwc3_qcom(dwc);

	/* A failed resume must not leave a callback into the removed driver. */
	dwc3_qcom_unregister_phy_mode(qcom);
	if (pm_runtime_resume_and_get(qcom->dev) < 0)
		return;

	dwc3_core_remove(&qcom->dwc);
	clk_bulk_disable_unprepare(qcom->num_clocks, qcom->clks);
	dwc3_qcom_interconnect_exit(qcom);

	pm_runtime_put_noidle(qcom->dev);
}

static int dwc3_qcom_pm_suspend(struct device *dev)
{
	struct dwc3 *dwc = dev_get_drvdata(dev);
	struct dwc3_qcom *qcom = to_dwc3_qcom(dwc);
	bool wakeup = device_may_wakeup(dev);
	int ret;

	guard(mutex)(&dwc->mutex);
	guard(mutex)(&qcom->pipe_clk_lock);
	ret = dwc3_pm_suspend(dwc);
	if (ret)
		return ret;

	ret = dwc3_qcom_suspend(qcom, wakeup);
	if (ret) {
		dwc3_pm_resume(dwc);
		return ret;
	}
	qcom->pm_suspended = true;
	return 0;
}

static int dwc3_qcom_pm_resume(struct device *dev)
{
	struct dwc3 *dwc = dev_get_drvdata(dev);
	struct dwc3_qcom *qcom = to_dwc3_qcom(dwc);
	bool wakeup = device_may_wakeup(dev);
	int ret;

	guard(mutex)(&dwc->mutex);
	guard(mutex)(&qcom->pipe_clk_lock);
	ret = dwc3_qcom_resume(qcom, wakeup);
	if (ret)
		goto out;

	dwc3_qcom_apply_usb_mode(qcom);
	ret = dwc3_pm_resume(dwc);
out:
	qcom->pm_suspended = false;
	return ret;
}

static void dwc3_qcom_complete(struct device *dev)
{
	struct dwc3 *dwc = dev_get_drvdata(dev);

	dwc3_pm_complete(dwc);
}

static int dwc3_qcom_prepare(struct device *dev)
{
	struct dwc3 *dwc = dev_get_drvdata(dev);

	/* Keep the system PM handshake even if runtime-suspended. */
	if (to_dwc3_qcom(dwc)->mode_registered)
		return 0;

	return dwc3_pm_prepare(dwc);
}

static int dwc3_qcom_runtime_suspend(struct device *dev)
{
	struct dwc3 *dwc = dev_get_drvdata(dev);
	struct dwc3_qcom *qcom = to_dwc3_qcom(dwc);
	int ret;

	guard(mutex)(&qcom->pipe_clk_lock);
	ret = dwc3_runtime_suspend(&qcom->dwc);
	if (ret)
		return ret;

	return dwc3_qcom_suspend(qcom, true);
}

static int dwc3_qcom_runtime_resume(struct device *dev)
{
	struct dwc3 *dwc = dev_get_drvdata(dev);
	struct dwc3_qcom *qcom = to_dwc3_qcom(dwc);
	int ret;

	guard(mutex)(&qcom->pipe_clk_lock);
	ret = dwc3_qcom_resume(qcom, true);
	if (ret)
		return ret;

	dwc3_qcom_apply_usb_mode(qcom);
	return dwc3_runtime_resume(&qcom->dwc);
}

static int dwc3_qcom_runtime_idle(struct device *dev)
{
	return dwc3_runtime_idle(dev_get_drvdata(dev));
}

static const struct dev_pm_ops dwc3_qcom_dev_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(dwc3_qcom_pm_suspend, dwc3_qcom_pm_resume)
	RUNTIME_PM_OPS(dwc3_qcom_runtime_suspend, dwc3_qcom_runtime_resume,
			   dwc3_qcom_runtime_idle)
	.complete = pm_sleep_ptr(dwc3_qcom_complete),
	.prepare = pm_sleep_ptr(dwc3_qcom_prepare),
};

static const struct of_device_id dwc3_qcom_of_match[] = {
	{ .compatible = "qcom,snps-dwc3" },
	{ }
};
MODULE_DEVICE_TABLE(of, dwc3_qcom_of_match);

static struct platform_driver dwc3_qcom_driver = {
	.probe		= dwc3_qcom_probe,
	.remove		= dwc3_qcom_remove,
	.shutdown	= dwc3_qcom_remove,
	.driver		= {
		.name	= "dwc3-qcom",
		.pm	= pm_ptr(&dwc3_qcom_dev_pm_ops),
		.of_match_table	= dwc3_qcom_of_match,
	},
};

module_platform_driver(dwc3_qcom_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare DWC3 QCOM Glue Driver");
