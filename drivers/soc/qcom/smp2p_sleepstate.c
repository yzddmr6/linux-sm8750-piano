// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm SMP2P sleep-state notifier
 *
 * Copyright (c) 2014-2022,2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * The remote processors (ADSP/CDSP) need to know when the application
 * processor is about to enter system sleep. The vendor firmware published
 * a "sleepstate" SMP2P entry for this. Notification is opt-in until
 * its standby behavior has been established on the board.
 */
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/soc/qcom/smem_state.h>
#include <linux/suspend.h>

/* Changes take effect on the next suspend attempt, never halfway through it. */
static bool enabled;
module_param(enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Notify DSPs of AP sleep state (default off)");

#define PROC_AWAKE_ID	12
#define AWAKE_BIT	BIT(PROC_AWAKE_ID)

struct smp2p_sleepstate {
	struct qcom_smem_state *state;
	struct device *dev;
	int irq;
	bool active;
	struct wakeup_source *ws;
	struct notifier_block pm_nb;
};

static int smp2p_sleepstate_pm_notifier(struct notifier_block *nb,
					unsigned long event, void *unused)
{
	struct smp2p_sleepstate *ss = container_of(nb, struct smp2p_sleepstate, pm_nb);

	int ret;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		if (!READ_ONCE(enabled))
			break;
		ret = enable_irq_wake(ss->irq);
		if (ret)
			return notifier_from_errno(ret);
		WRITE_ONCE(ss->active, true);
		ret = qcom_smem_state_update_bits(ss->state, AWAKE_BIT, 0);
		if (ret) {
			WRITE_ONCE(ss->active, false);
			disable_irq_wake(ss->irq);
			return notifier_from_errno(ret);
		}
		break;
	case PM_POST_SUSPEND:
		/* Also called when a later notifier or device aborts suspend. */
		if (!ss->active)
			break;
		ret = qcom_smem_state_update_bits(ss->state, AWAKE_BIT, AWAKE_BIT);
		if (ret)
			dev_err(ss->dev, "failed to restore awake bit: %d\n", ret);
		WRITE_ONCE(ss->active, false);
		disable_irq_wake(ss->irq);
		break;
	}

	return NOTIFY_DONE;
}

static irqreturn_t smp2p_sleepstate_handler(int irq, void *data)
{
	struct smp2p_sleepstate *ss = data;

	if (READ_ONCE(ss->active))
		__pm_wakeup_event(ss->ws, 200);

	return IRQ_HANDLED;
}

static void smp2p_sleepstate_release_ws(void *data)
{
	struct smp2p_sleepstate *ss = data;

	wakeup_source_unregister(ss->ws);
}

static void smp2p_sleepstate_release_notifier(void *data)
{
	struct smp2p_sleepstate *ss = data;

	unregister_pm_notifier(&ss->pm_nb);
}

static int smp2p_sleepstate_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct smp2p_sleepstate *ss;
	unsigned int bit;
	int irq, ret;

	ss = devm_kzalloc(dev, sizeof(*ss), GFP_KERNEL);
	if (!ss)
		return -ENOMEM;

	ss->dev = dev;
	ss->state = devm_qcom_smem_state_get(dev, NULL, &bit);
	if (IS_ERR(ss->state))
		return dev_err_probe(dev, PTR_ERR(ss->state),
				     "failed to acquire sleepstate entry\n");

	/* A registered entry must not advertise an asleep AP before its first cycle. */
	ret = qcom_smem_state_update_bits(ss->state, AWAKE_BIT, AWAKE_BIT);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set the awake bit\n");

	irq = of_irq_get_byname(dev->of_node, "smp2p-sleepstate-in");
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to get the sleepstate IRQ\n");

	ss->irq = irq;

	/* The source must exist before the IRQ can invoke its handler. */
	ss->ws = wakeup_source_register(dev, "smp2p-sleepstate");
	if (!ss->ws)
		return -ENOMEM;

	ret = devm_add_action_or_reset(dev, smp2p_sleepstate_release_ws, ss);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, irq, NULL, smp2p_sleepstate_handler,
					IRQF_ONESHOT | IRQF_NO_SUSPEND |
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
					"smp2p_sleepstate", ss);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request the sleepstate IRQ\n");

	ss->pm_nb.notifier_call = smp2p_sleepstate_pm_notifier;
	ss->pm_nb.priority = INT_MAX;
	ret = register_pm_notifier(&ss->pm_nb);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register the PM notifier\n");

	/* Reverse devres order: notifier, synchronized IRQ, wakeup source. */
	return devm_add_action_or_reset(dev, smp2p_sleepstate_release_notifier, ss);
}

static const struct of_device_id smp2p_sleepstate_of_match[] = {
	{ .compatible = "qcom,smp2p-sleepstate" },
	{ }
};
MODULE_DEVICE_TABLE(of, smp2p_sleepstate_of_match);

static struct platform_driver smp2p_sleepstate_driver = {
	.probe = smp2p_sleepstate_probe,
	.driver = {
		.name = "smp2p_sleepstate",
		.of_match_table = smp2p_sleepstate_of_match,
	},
};
module_platform_driver(smp2p_sleepstate_driver);

MODULE_DESCRIPTION("Qualcomm SMP2P sleep-state notifier");
MODULE_LICENSE("GPL");
