// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, Linaro Ltd
 */
#include <linux/auxiliary_bus.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>

#define PMIC_GLINK_SEND_TIMEOUT (5 * HZ)
#define PMIC_GLINK_MAX_DEFERRED	64

enum {
	PMIC_GLINK_CLIENT_BATT = 0,
	PMIC_GLINK_CLIENT_ALTMODE,
	PMIC_GLINK_CLIENT_UCSI,
};

struct pmic_glink_data {
	unsigned long	client_mask;
	const char	*charger_pdr_service_name;
	const char	*charger_pdr_service_path;
};

struct pmic_glink {
	struct device *dev;
	struct pdr_handle *pdr;

	struct rpmsg_endpoint *ept;

	const struct pmic_glink_data *data;

	struct auxiliary_device altmode_aux;
	struct auxiliary_device ps_aux;
	struct auxiliary_device ucsi_aux;

	/* serializing client_state and pdr_state updates */
	struct mutex state_lock;
	unsigned int client_state;
	unsigned int pdr_state;
	bool pdr_available;

	/* serializing clients list updates */
	spinlock_t client_lock;
	struct list_head clients;

	/* notifications held back over system sleep */
	spinlock_t defer_lock;
	bool defer;
	struct list_head deferred;
	unsigned int num_deferred;
	unsigned int num_dropped;
	struct work_struct deliver_work;
};

static struct pmic_glink *__pmic_glink;
static DEFINE_MUTEX(__pmic_glink_lock);

struct pmic_glink_client {
	struct list_head node;

	struct pmic_glink *pg;
	struct device *dev;
	unsigned int id;

	void (*cb)(const void *data, size_t len, void *priv);
	void (*pdr_notify)(void *priv, int state);
	bool (*wakeup_filter)(const void *data, size_t len, void *priv);
	void *priv;

	bool stay_awake;
};

struct pmic_glink_deferred_msg {
	struct list_head node;
	size_t len;
	u8 data[] __counted_by(len);
};

static void pmic_glink_client_relax(struct pmic_glink_client *client)
{
	lockdep_assert_held(&client->pg->client_lock);

	if (client->stay_awake) {
		client->stay_awake = false;
		pm_relax(client->dev);
	}
}

static void _devm_pmic_glink_release_client(struct device *dev, void *res)
{
	struct pmic_glink_client *client = (struct pmic_glink_client *)res;
	struct pmic_glink *pg = client->pg;
	unsigned long flags;

	spin_lock_irqsave(&pg->client_lock, flags);
	list_del(&client->node);
	pmic_glink_client_relax(client);
	spin_unlock_irqrestore(&pg->client_lock, flags);
}

/**
 * devm_pmic_glink_client_alloc() - allocate a client of the PMIC GLINK service
 * @dev:	client device, a child of the pmic_glink device
 * @id:		owner of the messages for the client
 * @cb:		called, in atomic context, with each message for the client
 * @pdr:	called, in atomic context, with the state of the remote service
 * @priv:	passed to @cb, @pdr and the wakeup filter
 *
 * A notification for a client whose @dev may wake up the system, see
 * device_may_wakeup(), is a wakeup event, unless the wakeup filter of the
 * client rejects it. Notifications that arrive during system sleep reach @cb
 * once the system has resumed.
 *
 * Return: the client, to be registered with pmic_glink_client_register(), or
 * an ERR_PTR() on failure.
 */
struct pmic_glink_client *devm_pmic_glink_client_alloc(struct device *dev,
						       unsigned int id,
						       void (*cb)(const void *, size_t, void *),
						       void (*pdr)(void *, int),
						       void *priv)
{
	struct pmic_glink_client *client;
	struct pmic_glink *pg = dev_get_drvdata(dev->parent);

	client = devres_alloc(_devm_pmic_glink_release_client, sizeof(*client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	client->pg = pg;
	client->dev = dev;
	client->id = id;
	client->cb = cb;
	client->pdr_notify = pdr;
	client->priv = priv;
	INIT_LIST_HEAD(&client->node);

	devres_add(dev, client);

	return client;
}
EXPORT_SYMBOL_GPL(devm_pmic_glink_client_alloc);

/**
 * pmic_glink_client_set_wakeup_filter() - select the wakeup notifications
 * @client:	client from devm_pmic_glink_client_alloc(), not registered yet
 * @filter:	called, in atomic context, with each notification for the client
 *		while its device may wake up the system; returns true to wake
 *		the system up
 *
 * Without a filter, every notification for the client is a wakeup event.
 */
void pmic_glink_client_set_wakeup_filter(struct pmic_glink_client *client,
					 bool (*filter)(const void *, size_t, void *))
{
	client->wakeup_filter = filter;
}
EXPORT_SYMBOL_GPL(pmic_glink_client_set_wakeup_filter);

void pmic_glink_client_register(struct pmic_glink_client *client)
{
	struct pmic_glink *pg = client->pg;
	unsigned long flags;

	guard(mutex)(&pg->state_lock);
	spin_lock_irqsave(&pg->client_lock, flags);

	list_add(&client->node, &pg->clients);
	client->pdr_notify(client->priv, pg->client_state);

	spin_unlock_irqrestore(&pg->client_lock, flags);
}
EXPORT_SYMBOL_GPL(pmic_glink_client_register);

int pmic_glink_send(struct pmic_glink_client *client, void *data, size_t len)
{
	struct pmic_glink *pg = client->pg;
	bool timeout_reached = false;
	unsigned long start;
	int ret;

	guard(mutex)(&pg->state_lock);
	if (!pg->ept) {
		return -ECONNRESET;
	}

	start = jiffies;
	for (;;) {
		ret = rpmsg_send(pg->ept, data, len);
		if (ret != -EAGAIN)
			break;

		if (timeout_reached) {
			ret = -ETIMEDOUT;
			break;
		}

		usleep_range(1000, 5000);
		timeout_reached = time_after(jiffies, start + PMIC_GLINK_SEND_TIMEOUT);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(pmic_glink_send);

static bool pmic_glink_is_wakeup_event(struct pmic_glink_client *client,
				       const void *data, size_t len)
{
	if (!device_may_wakeup(client->dev))
		return false;

	return !client->wakeup_filter || client->wakeup_filter(data, len, client->priv);
}

static void pmic_glink_dispatch(struct pmic_glink *pg, const void *data, size_t len)
{
	const struct pmic_glink_hdr *hdr = data;
	struct pmic_glink_client *client;
	unsigned long flags;

	spin_lock_irqsave(&pg->client_lock, flags);
	list_for_each_entry(client, &pg->clients, node) {
		if (client->id != le32_to_cpu(hdr->owner))
			continue;

		/* From here on, the client keeps the system awake if it needs to */
		pmic_glink_client_relax(client);
		client->cb(data, len, client->priv);
	}
	spin_unlock_irqrestore(&pg->client_lock, flags);
}

static bool pmic_glink_defer(struct pmic_glink *pg, const void *data, size_t len)
{
	struct pmic_glink_deferred_msg *msg;

	guard(spinlock_irqsave)(&pg->defer_lock);

	if (!pg->defer)
		return false;

	/* An identical notification brings nothing new, keep the latest copy */
	list_for_each_entry(msg, &pg->deferred, node) {
		if (msg->len == len && !memcmp(msg->data, data, len)) {
			list_move_tail(&msg->node, &pg->deferred);
			return true;
		}
	}

	if (pg->num_deferred == PMIC_GLINK_MAX_DEFERRED) {
		msg = list_first_entry(&pg->deferred, struct pmic_glink_deferred_msg, node);
		list_del(&msg->node);
		kfree(msg);
		pg->num_deferred--;
		pg->num_dropped++;
	}

	msg = kmalloc_flex(*msg, data, len, GFP_ATOMIC);
	if (!msg) {
		pg->num_dropped++;
		return true;
	}

	msg->len = len;
	memcpy(msg->data, data, len);
	list_add_tail(&msg->node, &pg->deferred);
	pg->num_deferred++;

	return true;
}

static void pmic_glink_drop_deferred(struct pmic_glink *pg)
{
	struct pmic_glink_deferred_msg *msg, *tmp;

	guard(spinlock_irqsave)(&pg->defer_lock);

	list_for_each_entry_safe(msg, tmp, &pg->deferred, node) {
		list_del(&msg->node);
		kfree(msg);
	}
	pg->num_deferred = 0;
}

static int pmic_glink_rpmsg_callback(struct rpmsg_device *rpdev, void *data,
				     int len, void *priv, u32 addr)
{
	struct pmic_glink_client *client;
	struct pmic_glink_hdr *hdr;
	struct pmic_glink *pg = dev_get_drvdata(&rpdev->dev);
	bool notification, deferred;
	unsigned long flags;

	if (len < sizeof(*hdr)) {
		dev_warn(pg->dev, "ignoring truncated message\n");
		return 0;
	}

	hdr = data;

	/* The sender of a request is waiting for the response */
	notification = le32_to_cpu(hdr->type) != PMIC_GLINK_REQ_RESP;

	/* Defer under client_lock, to set stay_awake before it is delivered */
	spin_lock_irqsave(&pg->client_lock, flags);
	deferred = notification && pmic_glink_defer(pg, data, len);
	list_for_each_entry(client, &pg->clients, node) {
		if (client->id != le32_to_cpu(hdr->owner))
			continue;

		if (deferred) {
			if (pmic_glink_is_wakeup_event(client, data, len)) {
				pm_stay_awake(client->dev);
				pm_system_wakeup();
				client->stay_awake = true;
			}
			continue;
		}

		if (notification && pmic_glink_is_wakeup_event(client, data, len))
			pm_wakeup_hard_event(client->dev);

		client->cb(data, len, client->priv);
	}
	spin_unlock_irqrestore(&pg->client_lock, flags);

	return 0;
}

static void pmic_glink_aux_release(struct device *dev)
{
	of_node_put(dev->of_node);
}

static int pmic_glink_add_aux_device(struct pmic_glink *pg,
				     struct auxiliary_device *aux,
				     const char *name)
{
	struct device *parent = pg->dev;
	int ret;

	aux->name = name;
	aux->dev.parent = parent;
	aux->dev.release = pmic_glink_aux_release;
	device_set_of_node_from_dev(&aux->dev, parent);
	ret = auxiliary_device_init(aux);
	if (ret) {
		of_node_put(aux->dev.of_node);
		return ret;
	}

	ret = auxiliary_device_add(aux);
	if (ret)
		auxiliary_device_uninit(aux);

	return ret;
}

static void pmic_glink_del_aux_device(struct pmic_glink *pg,
				      struct auxiliary_device *aux)
{
	auxiliary_device_delete(aux);
	auxiliary_device_uninit(aux);
}

static bool pmic_glink_deferring(struct pmic_glink *pg)
{
	guard(spinlock_irqsave)(&pg->defer_lock);

	return pg->defer;
}

static void pmic_glink_state_notify_clients(struct pmic_glink *pg)
{
	struct pmic_glink_client *client;
	unsigned int new_state = pg->client_state;
	unsigned long flags;

	if (pg->client_state != SERVREG_SERVICE_STATE_UP) {
		/* On UP the clients use the remote, hold that back as well */
		if (pg->pdr_state == SERVREG_SERVICE_STATE_UP && pg->ept &&
		    !pmic_glink_deferring(pg))
			new_state = SERVREG_SERVICE_STATE_UP;
	} else {
		if (pg->pdr_state == SERVREG_SERVICE_STATE_DOWN || !pg->ept)
			new_state = SERVREG_SERVICE_STATE_DOWN;
	}

	if (pg->pdr_state == SERVREG_SERVICE_STATE_DOWN || !pg->ept)
		pmic_glink_drop_deferred(pg);

	if (new_state != pg->client_state) {
		spin_lock_irqsave(&pg->client_lock, flags);
		list_for_each_entry(client, &pg->clients, node)
			client->pdr_notify(client->priv, new_state);
		spin_unlock_irqrestore(&pg->client_lock, flags);
		pg->client_state = new_state;
	}
}

static void pmic_glink_deliver_work(struct work_struct *work)
{
	struct pmic_glink *pg = container_of(work, struct pmic_glink, deliver_work);
	struct pmic_glink_deferred_msg *msg;
	struct pmic_glink_client *client;
	unsigned int dropped;
	unsigned long flags;

	guard(mutex)(&pg->state_lock);

	for (;;) {
		spin_lock_irqsave(&pg->defer_lock, flags);
		msg = list_first_entry_or_null(&pg->deferred, struct pmic_glink_deferred_msg, node);
		if (!msg) {
			pg->defer = false;
			dropped = pg->num_dropped;
			pg->num_dropped = 0;
			spin_unlock_irqrestore(&pg->defer_lock, flags);
			break;
		}

		list_del(&msg->node);
		pg->num_deferred--;
		spin_unlock_irqrestore(&pg->defer_lock, flags);

		pmic_glink_dispatch(pg, msg->data, msg->len);
		kfree(msg);
	}

	/* Release the clients whose notifications were dropped */
	spin_lock_irqsave(&pg->client_lock, flags);
	list_for_each_entry(client, &pg->clients, node)
		pmic_glink_client_relax(client);
	spin_unlock_irqrestore(&pg->client_lock, flags);

	pmic_glink_state_notify_clients(pg);

	if (dropped)
		dev_warn(pg->dev, "%u notifications lost during system sleep\n", dropped);
}

static void pmic_glink_pdr_callback(int state, char *svc_path, void *priv)
{
	struct pmic_glink *pg = priv;

	guard(mutex)(&pg->state_lock);
	pg->pdr_state = state;

	pmic_glink_state_notify_clients(pg);
}

static int pmic_glink_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct pmic_glink *pg;

	guard(mutex)(&__pmic_glink_lock);
	pg = __pmic_glink;
	if (!pg)
		return dev_err_probe(&rpdev->dev, -EPROBE_DEFER,
				     "no pmic_glink device to attach to\n");

	dev_set_drvdata(&rpdev->dev, pg);
	pg->pdr_available = rpdev->id.driver_data;

	guard(mutex)(&pg->state_lock);
	pg->ept = rpdev->ept;
	if (!pg->pdr_available)
		pg->pdr_state = SERVREG_SERVICE_STATE_UP;
	pmic_glink_state_notify_clients(pg);

	return 0;
}

static void pmic_glink_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct pmic_glink *pg;

	guard(mutex)(&__pmic_glink_lock);
	pg = __pmic_glink;
	if (!pg)
		return;

	guard(mutex)(&pg->state_lock);
	pg->ept = NULL;
	if (!pg->pdr_available)
		pg->pdr_state = SERVREG_SERVICE_STATE_DOWN;
	pmic_glink_state_notify_clients(pg);
}

static const struct rpmsg_device_id pmic_glink_rpmsg_id_match[] = {
	{.name = "PMIC_RTR_ADSP_APPS", .driver_data = true },
	{.name = "PMIC_RTR_SOCCP_APPS", .driver_data = false },
	{}
};

static struct rpmsg_driver pmic_glink_rpmsg_driver = {
	.probe = pmic_glink_rpmsg_probe,
	.remove = pmic_glink_rpmsg_remove,
	.callback = pmic_glink_rpmsg_callback,
	.id_table = pmic_glink_rpmsg_id_match,
	.drv  = {
		.name  = "qcom_pmic_glink_rpmsg",
	},
};

static int pmic_glink_probe(struct platform_device *pdev)
{
	struct pdr_service *service;
	struct pmic_glink *pg;
	int ret;

	pg = devm_kzalloc_obj(&pdev->dev, *pg);
	if (!pg)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, pg);

	pg->dev = &pdev->dev;

	INIT_LIST_HEAD(&pg->clients);
	spin_lock_init(&pg->client_lock);
	mutex_init(&pg->state_lock);

	INIT_LIST_HEAD(&pg->deferred);
	spin_lock_init(&pg->defer_lock);
	INIT_WORK(&pg->deliver_work, pmic_glink_deliver_work);

	pg->data = of_device_get_match_data(&pdev->dev);
	if (!pg->data)
		return -EINVAL;

	pg->pdr = pdr_handle_alloc(pmic_glink_pdr_callback, pg);
	if (IS_ERR(pg->pdr)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(pg->pdr),
				    "failed to initialize pdr\n");
		return ret;
	}

	if (pg->data->client_mask & BIT(PMIC_GLINK_CLIENT_UCSI)) {
		ret = pmic_glink_add_aux_device(pg, &pg->ucsi_aux, "ucsi");
		if (ret)
			goto out_release_pdr_handle;
	}
	if (pg->data->client_mask & BIT(PMIC_GLINK_CLIENT_ALTMODE)) {
		ret = pmic_glink_add_aux_device(pg, &pg->altmode_aux, "altmode");
		if (ret)
			goto out_release_ucsi_aux;
	}
	if (pg->data->client_mask & BIT(PMIC_GLINK_CLIENT_BATT)) {
		ret = pmic_glink_add_aux_device(pg, &pg->ps_aux, "power-supply");
		if (ret)
			goto out_release_altmode_aux;
	}

	if (pg->data->charger_pdr_service_name && pg->data->charger_pdr_service_path) {
		service = pdr_add_lookup(pg->pdr, pg->data->charger_pdr_service_name,
					 pg->data->charger_pdr_service_path);
		if (IS_ERR(service)) {
			ret = dev_err_probe(&pdev->dev, PTR_ERR(service),
					    "failed adding pdr lookup for charger_pd\n");
			goto out_release_aux_devices;
		}
	}

	mutex_lock(&__pmic_glink_lock);
	__pmic_glink = pg;
	mutex_unlock(&__pmic_glink_lock);

	return 0;

out_release_aux_devices:
	if (pg->data->client_mask & BIT(PMIC_GLINK_CLIENT_BATT))
		pmic_glink_del_aux_device(pg, &pg->ps_aux);
out_release_altmode_aux:
	if (pg->data->client_mask & BIT(PMIC_GLINK_CLIENT_ALTMODE))
		pmic_glink_del_aux_device(pg, &pg->altmode_aux);
out_release_ucsi_aux:
	if (pg->data->client_mask & BIT(PMIC_GLINK_CLIENT_UCSI))
		pmic_glink_del_aux_device(pg, &pg->ucsi_aux);
out_release_pdr_handle:
	pdr_handle_release(pg->pdr);

	return ret;
}

static void pmic_glink_remove(struct platform_device *pdev)
{
	struct pmic_glink *pg = dev_get_drvdata(&pdev->dev);

	cancel_work_sync(&pg->deliver_work);

	pdr_handle_release(pg->pdr);

	if (pg->data->client_mask & BIT(PMIC_GLINK_CLIENT_BATT))
		pmic_glink_del_aux_device(pg, &pg->ps_aux);
	if (pg->data->client_mask & BIT(PMIC_GLINK_CLIENT_ALTMODE))
		pmic_glink_del_aux_device(pg, &pg->altmode_aux);
	if (pg->data->client_mask & BIT(PMIC_GLINK_CLIENT_UCSI))
		pmic_glink_del_aux_device(pg, &pg->ucsi_aux);

	pmic_glink_drop_deferred(pg);

	guard(mutex)(&__pmic_glink_lock);
	__pmic_glink = NULL;
}

static int pmic_glink_prepare(struct device *dev)
{
	struct pmic_glink *pg = dev_get_drvdata(dev);

	guard(spinlock_irqsave)(&pg->defer_lock);
	pg->defer = true;

	return 0;
}

static void pmic_glink_complete(struct device *dev)
{
	struct pmic_glink *pg = dev_get_drvdata(dev);

	/* Runs once the tasks are thawed, all devices have resumed by then */
	queue_work(system_freezable_wq, &pg->deliver_work);
}

static const struct dev_pm_ops pmic_glink_pm_ops = {
	.prepare = pmic_glink_prepare,
	.complete = pmic_glink_complete,
};

static const struct pmic_glink_data pmic_glink_adsp_data = {
	.client_mask = BIT(PMIC_GLINK_CLIENT_BATT) |
		       BIT(PMIC_GLINK_CLIENT_ALTMODE) |
		       BIT(PMIC_GLINK_CLIENT_UCSI),
	.charger_pdr_service_name = "tms/servreg",
	.charger_pdr_service_path = "msm/adsp/charger_pd",
};

static const struct pmic_glink_data pmic_glink_soccp_data = {
	.client_mask = BIT(PMIC_GLINK_CLIENT_BATT) |
		       BIT(PMIC_GLINK_CLIENT_ALTMODE) |
		       BIT(PMIC_GLINK_CLIENT_UCSI),
};

static const struct of_device_id pmic_glink_of_match[] = {
	{ .compatible = "qcom,glymur-pmic-glink", .data = &pmic_glink_soccp_data },
	{ .compatible = "qcom,kaanapali-pmic-glink", .data = &pmic_glink_soccp_data },
	{ .compatible = "qcom,pmic-glink", .data = &pmic_glink_adsp_data },
	{}
};
MODULE_DEVICE_TABLE(of, pmic_glink_of_match);

static struct platform_driver pmic_glink_driver = {
	.probe = pmic_glink_probe,
	.remove = pmic_glink_remove,
	.driver = {
		.name = "qcom_pmic_glink",
		.of_match_table = pmic_glink_of_match,
		.pm = pm_sleep_ptr(&pmic_glink_pm_ops),
	},
};

static int pmic_glink_init(void)
{
	int ret;

	ret = platform_driver_register(&pmic_glink_driver);
	if (ret < 0)
		return ret;

	ret = register_rpmsg_driver(&pmic_glink_rpmsg_driver);
	if (ret < 0) {
		platform_driver_unregister(&pmic_glink_driver);
		return ret;
	}

	return 0;
}
module_init(pmic_glink_init);

static void pmic_glink_exit(void)
{
	unregister_rpmsg_driver(&pmic_glink_rpmsg_driver);
	platform_driver_unregister(&pmic_glink_driver);
}
module_exit(pmic_glink_exit);

MODULE_DESCRIPTION("Qualcomm PMIC GLINK driver");
MODULE_LICENSE("GPL");
