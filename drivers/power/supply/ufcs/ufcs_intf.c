// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2023-2023 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[INTF]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/power_supply.h>

#include "ufcs_class.h"
#include "ufcs_intf.h"

int ufcs_msg_handler(struct ufcs_dev *ufcs)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}
	class = ufcs->class;

	kthread_queue_work(class->worker, &class->recv_work);

	return 0;
}
EXPORT_SYMBOL(ufcs_msg_handler);

/* All limits are input-side mA. Choose the best advertised mode at this
 * voltage and round current down on the charger's own step grid.
 */
static int ufcs_pdo_select(struct ufcs_class *class, int mv, int limit_ma,
			   bool exact, int *selected_ma)
{
	int best_ma = 0, index = -ENODATA;
	int i;

	if (class->test_mode)
		return -EBUSY;
	for (i = 0; i < class->pdo.num; i++) {
		u64 pdo = class->pdo.data[i];
		u32 change = class->pwr_change_info[i];
		int min_mv = UFCS_OUTPUT_MODE_VOL_MIN(pdo);
		int min_ma = UFCS_OUTPUT_MODE_CURR_MIN(pdo);
		int ma = min_t(int, limit_ma, UFCS_OUTPUT_MODE_CURR_MAX(pdo));

		if (mv < min_mv || mv > UFCS_OUTPUT_MODE_VOL_MAX(pdo) ||
		    (mv - min_mv) % UFCS_OUTPUT_MODE_VOL_STEP(pdo))
			continue;
		if (change)
			ma = min_t(int, ma, UFCS_POWER_CHANGE_CURR_MAX(change));
		if (ma < min_ma || !UFCS_OUTPUT_MODE_INDEX(pdo))
			continue;
		ma -= (ma - min_ma) % UFCS_OUTPUT_MODE_CURR_STEP(pdo);
		if (ma <= best_ma || (exact && ma != limit_ma))
			continue;
		best_ma = ma;
		index = UFCS_OUTPUT_MODE_INDEX(pdo);
	}
	*selected_ma = best_ma;
	return index;
}

static int ufcs_request(struct ufcs_class *class, int vol_mv, int limit_ma,
			bool exact, int *applied_ma)
{
	int curr_ma;
	int rc;
	struct ufcs_data_msg_request *request;
	struct ufcs_event *event;

	event = devm_kzalloc(&class->ufcs->dev,
		sizeof(struct ufcs_data_msg_request) + sizeof(struct ufcs_event),
		GFP_KERNEL);
	if (event == NULL) {
		ufcs_err("alloc event buf error\n");
		return -ENOMEM;
	}

	request = (struct ufcs_data_msg_request *)((u8 *)event + sizeof(struct ufcs_event));
	event->data = request;
	event->msg = NULL;
	event->type = UFCS_EVENT_SEND_REQUEST;
	INIT_LIST_HEAD(&event->list);

	mutex_lock(&class->ext_req_lock);
	mutex_lock(&class->pe_lock);
	reinit_completion(&class->request_ack);
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		rc = -EINVAL;
		goto err;
	}
	rc = ufcs_pdo_select(class, vol_mv, limit_ma, exact, &curr_ma);
	if (rc < 0)
		goto err;
	request->request = UFCS_REQUEST_DATA(rc, vol_mv, curr_ma);
	rc = ufcs_push_event(class, event);
	if (rc < 0) {
		ufcs_err("push pdo request event error, rc=%d\n", rc);
		goto err;
	}
	mutex_unlock(&class->pe_lock);
	wait_for_completion(&class->request_ack);
	rc = READ_ONCE(class->state.err);
	mutex_unlock(&class->ext_req_lock);

	if (rc < 0) {
		ufcs_err("set pdo error, rc=%d\n", rc);
		return rc;
	}

	if (applied_ma)
		*applied_ma = curr_ma;
	return 0;

err:
	mutex_unlock(&class->pe_lock);
	mutex_unlock(&class->ext_req_lock);
	devm_kfree(&class->ufcs->dev, event);
	return rc;
}

int ufcs_pdo_set(struct ufcs_class *class, int vol_mv, int curr_ma)
{
	return ufcs_request(class, vol_mv, curr_ma, true, NULL);
}

int ufcs_request_limited(struct ufcs_dev *ufcs, int mv, int cable_ma,
			 int pumps_ma, int envelope_ma, int *applied_ma)
{
	if (!ufcs || !applied_ma || mv <= 0 || cable_ma <= 0 ||
	    pumps_ma <= 0 || envelope_ma <= 0)
		return -EINVAL;
	return ufcs_request(ufcs->class, mv, min3(cable_ma, pumps_ma, envelope_ma),
			    false, applied_ma);
}
EXPORT_SYMBOL_GPL(ufcs_request_limited);

int ufcs_intf_pdo_set(struct ufcs_dev *ufcs, int vol_mv, int curr_ma)
{
	if (!ufcs)
		return -EINVAL;
	return ufcs_pdo_set(ufcs->class, vol_mv, curr_ma);
}
EXPORT_SYMBOL(ufcs_intf_pdo_set);

int ufcs_handshake(struct ufcs_dev *ufcs)
{
	int rc;
	struct ufcs_event *event;
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}
	class = ufcs->class;
	ufcs_info("start handshake\n");

	class->start_cable_detect = false;
	class->exit_ufcs_ack_received = false;
	event = devm_kzalloc(&class->ufcs->dev, sizeof(struct ufcs_event), GFP_KERNEL);
	if (event == NULL) {
		ufcs_err("alloc event buf error\n");
		return -ENOMEM;
	}

	event->data = NULL;
	event->msg = NULL;
	event->type = UFCS_EVENT_HANDSHAKE;
	INIT_LIST_HEAD(&event->list);

	mutex_lock(&class->handshake_lock);
	mutex_lock(&class->ext_req_lock);
	mutex_lock(&class->pe_lock);
	ufcs_event_reset(class);
	ufcs->handshake_state = UFCS_HS_IDLE;
	rc = ufcs->ops->enable(ufcs);
	if (rc) {
		ufcs->ops->disable(ufcs);
		mutex_unlock(&class->pe_lock);
		mutex_unlock(&class->handshake_lock);
		mutex_unlock(&class->ext_req_lock);
		devm_kfree(&ufcs->dev, event);
		return rc;
	}
	ufcs_clr_error_flag(ufcs);
	class->sm_task_wakeup = true;
	wake_up(&class->sm_wq);
	reinit_completion(&class->request_ack);
	rc = ufcs_push_event(class, event);
	if (rc < 0) {
		mutex_unlock(&class->pe_lock);
		mutex_unlock(&class->handshake_lock);
		mutex_unlock(&class->ext_req_lock);
		ufcs_err("push handshake event error, rc=%d\n", rc);
		devm_kfree(&class->ufcs->dev, event);
		return rc;
	}
	class->handshake_success = false;
	mutex_unlock(&class->pe_lock);
	wait_for_completion(&class->request_ack);
	if (READ_ONCE(class->handshake_success))
		rc = 0;
	else
		rc = READ_ONCE(ufcs->handshake_state) == UFCS_HS_TIMEOUT ? -ETIMEDOUT : -EIO;
	mutex_unlock(&class->ext_req_lock);
	mutex_unlock(&class->handshake_lock);
	return rc;
}
EXPORT_SYMBOL(ufcs_handshake);

int ufcs_source_hard_reset(struct ufcs_class *class)
{
	int rc;

	if (class == NULL) {
		ufcs_err("class is NULL\n");
		return -EINVAL;
	}

	ufcs_err("send source hard reset\n");

	class->ufcs->ops->source_hard_reset(class->ufcs);
	rc = class->ufcs->ops->disable(class->ufcs);
	ufcs_clean_process_info(class);
	ufcs_exit_sm_work(class);
	ufcs_send_state(UFCS_NOTIFY_SOURCE_HW_RESET, NULL);
	return rc;
}

int ufcs_intf_source_hard_reset(struct ufcs_dev *ufcs)
{
	int rc;

	if (!ufcs)
		return -EINVAL;
	mutex_lock(&ufcs->class->ext_req_lock);
	mutex_lock(&ufcs->class->pe_lock);
	rc = ufcs_source_hard_reset(ufcs->class);
	mutex_unlock(&ufcs->class->pe_lock);
	mutex_unlock(&ufcs->class->ext_req_lock);
	return rc;
}
EXPORT_SYMBOL(ufcs_intf_source_hard_reset);

int ufcs_cable_hard_reset(struct ufcs_class *class)
{
	int rc;

	if (class == NULL) {
		ufcs_err("class is NULL\n");
		return -EINVAL;
	}

	ufcs_err("send cable hard reset\n");
	rc = class->ufcs->ops->cable_hard_reset(class->ufcs);
	ufcs_send_state(UFCS_NOTIFY_CABLE_HW_RESET, NULL);

	return rc;
}

int ufcs_intf_cable_hard_reset(struct ufcs_dev *ufcs)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}
	class = ufcs->class;

	ufcs_err("send cable hard reset\n");
	return class->ufcs->ops->cable_hard_reset(class->ufcs);
}
EXPORT_SYMBOL(ufcs_intf_cable_hard_reset);

static int ufcs_send_exit_ufcs_mode(struct ufcs_class *class)
{
	int rc;
	struct ufcs_event *event;

	if (class->test_mode) {
		ufcs_err("test mode, unable to process external request\n");
		return -EBUSY;
	}

	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		return -EINVAL;
	}

	event = devm_kzalloc(&class->ufcs->dev, sizeof(struct ufcs_event), GFP_KERNEL);
	if (event == NULL) {
		ufcs_err("alloc event buf error\n");
		return -ENOMEM;
	}

	event->data = NULL;
	event->msg = NULL;
	event->type = UFCS_EVENT_SEND_EXIT_UFCS_MODE;
	INIT_LIST_HEAD(&event->list);

	mutex_lock(&class->ext_req_lock);
	mutex_lock(&class->pe_lock);
	reinit_completion(&class->request_ack);
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		rc = -EINVAL;
		goto err;
	}
	rc = ufcs_push_event(class, event);
	if (rc < 0) {
		ufcs_err("push exit ufcs mode event error, rc=%d\n", rc);
		goto err;
	}
	mutex_unlock(&class->pe_lock);
	wait_for_completion(&class->request_ack);
	class->handshake_success = false;
	rc = READ_ONCE(class->state.err);
	mutex_unlock(&class->ext_req_lock);

	if (rc < 0) {
		ufcs_err("exit ufcs mode error, rc=%d\n", rc);
		return rc;
	}

	return 0;

err:
	mutex_unlock(&class->pe_lock);
	mutex_unlock(&class->ext_req_lock);
	devm_kfree(&class->ufcs->dev, event);
	return rc;
}

int ufcs_force_exit(struct ufcs_dev *ufcs)
{
	struct ufcs_class *class;
	int rc = 0;

	if (!ufcs)
		return -EINVAL;
	class = ufcs->class;
	mutex_lock(&class->handshake_lock);
	if (READ_ONCE(class->handshake_success))
		rc = ufcs_send_exit_ufcs_mode(class);
	mutex_lock(&class->pe_lock);
	if (rc)
		rc = ufcs_source_hard_reset(class);
	else
		rc = ufcs->ops->disable(ufcs);
	ufcs_clean_process_info(class);
	mutex_unlock(&class->pe_lock);
	mutex_unlock(&class->handshake_lock);
	return rc;
}
EXPORT_SYMBOL(ufcs_force_exit);

int ufcs_config_watchdog(struct ufcs_class *class, u16 time_ms)
{
	int rc;
	struct ufcs_event *event;
	struct ufcs_data_msg_config_watchdog *config_wd;

	if (class == NULL) {
		ufcs_err("ufcs class is NULL\n");
		return -EINVAL;
	}
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		return -EINVAL;
	}

	event = devm_kzalloc(&class->ufcs->dev,
		sizeof(struct ufcs_event) + sizeof(struct ufcs_data_msg_config_watchdog),
		GFP_KERNEL);
	if (event == NULL) {
		ufcs_err("alloc event buf error\n");
		return -ENOMEM;
	}

	config_wd = (struct ufcs_data_msg_config_watchdog *)((u8 *)event + sizeof(struct ufcs_event));
	config_wd->over_time_ms = time_ms;
	event->data = config_wd;
	event->msg = NULL;
	event->type = UFCS_EVENT_SEND_CONFIG_WD;
	INIT_LIST_HEAD(&event->list);

	mutex_lock(&class->ext_req_lock);
	mutex_lock(&class->pe_lock);
	reinit_completion(&class->request_ack);
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		rc = -EINVAL;
		goto err;
	}
	rc = ufcs_push_event(class, event);
	if (rc < 0) {
		ufcs_err("push config watch dog event error, rc=%d\n", rc);
		goto err;
	}
	mutex_unlock(&class->pe_lock);
	wait_for_completion(&class->request_ack);
	rc = READ_ONCE(class->state.err);
	mutex_unlock(&class->ext_req_lock);

	if (rc < 0) {
		ufcs_err("config watch dog error, rc=%d\n", rc);
		return rc;
	}

	return 0;

err:
	mutex_unlock(&class->pe_lock);
	mutex_unlock(&class->ext_req_lock);
	devm_kfree(&class->ufcs->dev, event);
	return rc;
}

int ufcs_intf_config_watchdog(struct ufcs_dev *ufcs, u16 time_ms)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}
	class = ufcs->class;

	if (class->test_mode) {
		ufcs_err("test mode, unable to process external request\n");
		return -EBUSY;
	}

	return ufcs_config_watchdog(class, time_ms);
}
EXPORT_SYMBOL(ufcs_intf_config_watchdog);

void ufcs_clr_error_flag(struct ufcs_dev *ufcs)
{
	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return;
	}

	spin_lock(&ufcs->class->err_flag_lock);
	ufcs->err_flag_save = 0;
	kfifo_reset(&ufcs->err_flag_fifo);
	spin_unlock(&ufcs->class->err_flag_lock);
}
EXPORT_SYMBOL(ufcs_clr_error_flag);

int ufcs_set_error_flag(struct ufcs_dev *ufcs, unsigned int err_flag)
{
	struct ufcs_class *class;
	int rc;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}
	class = ufcs->class;

	spin_lock(&class->err_flag_lock);
	rc = kfifo_in(&ufcs->err_flag_fifo, &err_flag, sizeof(err_flag));
	spin_unlock(&class->err_flag_lock);
	if (rc != sizeof(err_flag)) {
		ufcs_err("write err_flag_fifo error, rc=%d\n", rc);
		schedule_work(&class->fifo_overflow_work);
		return -EFAULT;
	}

	return 0;
}
EXPORT_SYMBOL(ufcs_set_error_flag);

int ufcs_get_error_flag(struct ufcs_dev *ufcs, unsigned int *err_flag)
{
	int rc;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}

	spin_lock(&ufcs->class->err_flag_lock);
	rc = kfifo_out(&ufcs->err_flag_fifo, err_flag, sizeof(*err_flag));
	spin_unlock(&ufcs->class->err_flag_lock);
	if (rc != sizeof(*err_flag)) {
		ufcs_err("read err_flag_fifo error, rc=%d\n", rc);
		return -EFAULT;
	}

	return 0;
}
EXPORT_SYMBOL(ufcs_get_error_flag);

int ufcs_check_error_flag_all(struct ufcs_dev *ufcs, unsigned int *err_flag)
{
	unsigned int flag_buf[UFCS_ERR_FLAG_BUF_LEN];
	unsigned int size;
	int i;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}

	size = kfifo_out_peek(&ufcs->err_flag_fifo, &flag_buf, UFCS_ERR_FLAG_BUF_SIZE);
	*err_flag = 0;
	for (i = 0; i < size / sizeof(unsigned int); i++)
		*err_flag |= flag_buf[i];
	*err_flag |= ufcs->err_flag_save;

	return 0;
}
EXPORT_SYMBOL(ufcs_check_error_flag_all);

int ufcs_get_device_info(struct ufcs_class *class, u64 *dev_info)
{
	int rc;
	struct ufcs_event *event;

	if (class == NULL) {
		ufcs_err("ufcs class is NULL\n");
		return -EINVAL;
	}
	if (dev_info == NULL) {
		ufcs_err("dev_info buf is NULL\n");
		return -EINVAL;
	}
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		return -EINVAL;
	}

	event = devm_kzalloc(&class->ufcs->dev, sizeof(struct ufcs_event), GFP_KERNEL);
	if (event == NULL) {
		ufcs_err("alloc event buf error\n");
		return -ENOMEM;
	}

	event->data = NULL;
	event->msg = NULL;
	event->type = UFCS_EVENT_SEND_GET_DEVICE_INFO;
	INIT_LIST_HEAD(&event->list);

	mutex_lock(&class->ext_req_lock);
	mutex_lock(&class->pe_lock);
	reinit_completion(&class->request_ack);
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		rc = -EINVAL;
		goto err;
	}
	rc = ufcs_push_event(class, event);
	if (rc < 0) {
		ufcs_err("push get device info event error, rc=%d\n", rc);
		goto err;
	}
	mutex_unlock(&class->pe_lock);
	wait_for_completion(&class->request_ack);
	rc = READ_ONCE(class->state.err);
	mutex_unlock(&class->ext_req_lock);

	if (rc < 0) {
		ufcs_err("get device info error, rc=%d\n", rc);
		return rc;
	}

	*dev_info = class->dev_info;

	return 0;

err:
	mutex_unlock(&class->pe_lock);
	mutex_unlock(&class->ext_req_lock);
	devm_kfree(&class->ufcs->dev, event);
	return rc;
}

int ufcs_intf_get_device_info(struct ufcs_dev *ufcs, u64 *dev_info)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}
	class = ufcs->class;
	if (dev_info == NULL) {
		ufcs_err("dev_info buf is NULL\n");
		return -EINVAL;
	}

	if (class->test_mode) {
		ufcs_err("test mode, unable to process external request\n");
		return -EBUSY;
	}

	return ufcs_get_device_info(class, dev_info);
}
EXPORT_SYMBOL(ufcs_intf_get_device_info);

int ufcs_get_error_info(struct ufcs_class *class, u64 *err_info)
{
	int rc;
	struct ufcs_event *event;

	if (class == NULL) {
		ufcs_err("ufcs class is NULL\n");
		return -EINVAL;
	}
	if (err_info == NULL) {
		ufcs_err("err_info buf is NULL\n");
		return -EINVAL;
	}
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		return -EINVAL;
	}

	event = devm_kzalloc(&class->ufcs->dev, sizeof(struct ufcs_event), GFP_KERNEL);
	if (event == NULL) {
		ufcs_err("alloc event buf error\n");
		return -ENOMEM;
	}

	event->data = NULL;
	event->msg = NULL;
	event->type = UFCS_EVENT_SEND_GET_ERROR_INFO;
	INIT_LIST_HEAD(&event->list);

	mutex_lock(&class->ext_req_lock);
	mutex_lock(&class->pe_lock);
	reinit_completion(&class->request_ack);
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		rc = -EINVAL;
		goto err;
	}
	rc = ufcs_push_event(class, event);
	if (rc < 0) {
		ufcs_err("push get error info event error, rc=%d\n", rc);
		goto err;
	}
	mutex_unlock(&class->pe_lock);
	wait_for_completion(&class->request_ack);
	rc = READ_ONCE(class->state.err);
	mutex_unlock(&class->ext_req_lock);

	if (rc < 0) {
		ufcs_err("get error info error, rc=%d\n", rc);
		return rc;
	}

	*err_info = class->err_info;

	return 0;

err:
	mutex_unlock(&class->pe_lock);
	mutex_unlock(&class->ext_req_lock);
	devm_kfree(&class->ufcs->dev, event);
	return rc;
}

int ufcs_intf_get_error_info(struct ufcs_dev *ufcs, u64 *err_info)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}
	class = ufcs->class;
	if (err_info == NULL) {
		ufcs_err("err_info buf is NULL\n");
		return -EINVAL;
	}

	if (class->test_mode) {
		ufcs_err("test mode, unable to process external request\n");
		return -EBUSY;
	}

	return ufcs_get_error_info(class, err_info);
}
EXPORT_SYMBOL(ufcs_intf_get_error_info);

int ufcs_get_source_info(struct ufcs_class *class, u64 *src_info)
{
	int rc;
	struct ufcs_event *event;

	if (class == NULL) {
		ufcs_err("ufcs class is NULL\n");
		return -EINVAL;
	}
	if (src_info == NULL) {
		ufcs_err("src_info buf is NULL\n");
		return -EINVAL;
	}
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		return -EINVAL;
	}

	event = devm_kzalloc(&class->ufcs->dev, sizeof(struct ufcs_event), GFP_KERNEL);
	if (event == NULL) {
		ufcs_err("alloc event buf error\n");
		return -ENOMEM;
	}

	event->data = NULL;
	event->msg = NULL;
	event->type = UFCS_EVENT_SEND_GET_SOURCE_INFO;
	INIT_LIST_HEAD(&event->list);

	mutex_lock(&class->ext_req_lock);
	mutex_lock(&class->pe_lock);
	reinit_completion(&class->request_ack);
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		rc = -EINVAL;
		goto err;
	}
	rc = ufcs_push_event(class, event);
	if (rc < 0) {
		ufcs_err("push get source info event error, rc=%d\n", rc);
		goto err;
	}
	mutex_unlock(&class->pe_lock);
	wait_for_completion(&class->request_ack);
	rc = READ_ONCE(class->state.err);
	mutex_unlock(&class->ext_req_lock);

	if (rc < 0) {
		ufcs_err("get source info error, rc=%d\n", rc);
		return rc;
	}

	*src_info = class->src_info;

	return 0;

err:
	mutex_unlock(&class->pe_lock);
	mutex_unlock(&class->ext_req_lock);
	devm_kfree(&class->ufcs->dev, event);
	return rc;
}

int ufcs_intf_get_source_info(struct ufcs_dev *ufcs, u64 *src_info)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}
	class = ufcs->class;
	if (src_info == NULL) {
		ufcs_err("src_info buf is NULL\n");
		return -EINVAL;
	}

	if (class->test_mode) {
		ufcs_err("test mode, unable to process external request\n");
		return -EBUSY;
	}

	return ufcs_get_source_info(class, src_info);
}
EXPORT_SYMBOL(ufcs_intf_get_source_info);

int ufcs_get_cable_info(struct ufcs_class *class, u64 *cable_info)
{
	int rc;
	struct ufcs_event *event;

	if (class == NULL) {
		ufcs_err("ufcs class is NULL\n");
		return -EINVAL;
	}
	if (cable_info == NULL) {
		ufcs_err("cable_info buf is NULL\n");
		return -EINVAL;
	}
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		return -EINVAL;
	}

	event = devm_kzalloc(&class->ufcs->dev, sizeof(struct ufcs_event), GFP_KERNEL);
	if (event == NULL) {
		ufcs_err("alloc event buf error\n");
		return -ENOMEM;
	}

	event->data = NULL;
	event->msg = NULL;
	event->type = UFCS_EVENT_SEND_START_CABLE_DETECT;
	INIT_LIST_HEAD(&event->list);

	mutex_lock(&class->ext_req_lock);
	mutex_lock(&class->pe_lock);
	reinit_completion(&class->request_ack);
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		rc = -EINVAL;
		goto err;
	}
	rc = ufcs_push_event(class, event);
	if (rc < 0) {
		ufcs_err("push get cable info event error, rc=%d\n", rc);
		goto err;
	}
	mutex_unlock(&class->pe_lock);
	wait_for_completion(&class->request_ack);
	rc = READ_ONCE(class->state.err);
	mutex_unlock(&class->ext_req_lock);

	if (rc < 0) {
		ufcs_err("get cable info error, rc=%d\n", rc);
		return rc;
	}

	*cable_info = class->cable_info;

	return 0;

err:
	mutex_unlock(&class->pe_lock);
	mutex_unlock(&class->ext_req_lock);
	devm_kfree(&class->ufcs->dev, event);
	return rc;
}

int ufcs_intf_get_cable_info(struct ufcs_dev *ufcs, u64 *cable_info)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}
	class = ufcs->class;
	if (cable_info == NULL) {
		ufcs_err("cable_info buf is NULL\n");
		return -EINVAL;
	}

	if (class->test_mode) {
		ufcs_err("test mode, unable to process external request\n");
		return -EBUSY;
	}

	return ufcs_get_cable_info(class, cable_info);
}
EXPORT_SYMBOL(ufcs_intf_get_cable_info);

int ufcs_get_pdo_info(struct ufcs_class *class, u64 *pdo, int num)
{
	int rc;
	struct ufcs_event *event;
	int i;

	if (class == NULL) {
		ufcs_err("ufcs class is NULL\n");
		return -EINVAL;
	}
	if (pdo == NULL) {
		ufcs_err("pdo buf is NULL\n");
		return -EINVAL;
	}
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		return -EINVAL;
	}

	event = devm_kzalloc(&class->ufcs->dev, sizeof(struct ufcs_event), GFP_KERNEL);
	if (event == NULL) {
		ufcs_err("alloc event buf error\n");
		return -ENOMEM;
	}

	event->data = NULL;
	event->msg = NULL;
	event->type = UFCS_EVENT_SEND_GET_OUTPUT_CAP;
	INIT_LIST_HEAD(&event->list);

	mutex_lock(&class->ext_req_lock);
	mutex_lock(&class->pe_lock);
	reinit_completion(&class->request_ack);
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		rc = -EINVAL;
		goto err;
	}
	rc = ufcs_push_event(class, event);
	if (rc < 0) {
		ufcs_err("push get output cap event error, rc=%d\n", rc);
		goto err;
	}
	mutex_unlock(&class->pe_lock);
	wait_for_completion(&class->request_ack);
	rc = READ_ONCE(class->state.err);
	mutex_unlock(&class->ext_req_lock);

	if (rc < 0) {
		ufcs_err("get pdo info error, rc=%d\n", rc);
		return rc;
	}

	if (num < class->pdo.num)
		ufcs_err("pdo buf too short\n");
	else
		num = class->pdo.num;
	for (i = 0; i < num; i++)
		pdo[i] = class->pdo.data[i];

	return num;

err:
	mutex_unlock(&class->pe_lock);
	mutex_unlock(&class->ext_req_lock);
	devm_kfree(&class->ufcs->dev, event);
	return rc;
}

int ufcs_intf_get_pdo_info(struct ufcs_dev *ufcs, u64 *pdo, int num)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}
	class = ufcs->class;
	if (pdo == NULL) {
		ufcs_err("pdo buf is NULL\n");
		return -EINVAL;
	}

	if (class->test_mode) {
		ufcs_err("test mode, unable to process external request\n");
		return -EBUSY;
	}

	return ufcs_get_pdo_info(class, pdo, num);
}
EXPORT_SYMBOL(ufcs_intf_get_pdo_info);

int ufcs_verify_adapter(struct ufcs_class *class, u8 key_index, u8 *auth_data, u8 data_len)
{
	int rc;
	struct ufcs_event *event;
	struct ufcs_data_msg_verify_request *verify_request;

	if (class == NULL) {
		ufcs_err("ufcs class is NULL\n");
		return -EINVAL;
	}
	if (auth_data == NULL) {
		ufcs_err("auth_data is NULL\n");
		return -EINVAL;
	}
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		return -EINVAL;
	}
	if (data_len != UFCS_VERIFY_AUTH_DATA_SIZE) {
		ufcs_err("data_len is not equal to %d\n", UFCS_VERIFY_AUTH_DATA_SIZE);
		return -EINVAL;
	}

	event = devm_kzalloc(&class->ufcs->dev,
		sizeof(struct ufcs_data_msg_verify_request) + sizeof(struct ufcs_event),
		GFP_KERNEL);
	if (event == NULL) {
		ufcs_err("alloc event buf error\n");
		return -ENOMEM;
	}

	verify_request = (struct ufcs_data_msg_verify_request *)((u8 *)event + sizeof(struct ufcs_event));
	event->data = verify_request;
	event->msg = NULL;
	event->type = UFCS_EVENT_SEND_VERIFY_REQUEST;
	INIT_LIST_HEAD(&event->list);
	verify_request->index = key_index;
	get_random_bytes(verify_request->random_data, sizeof(verify_request->random_data));

	mutex_lock(&class->ext_req_lock);
	mutex_lock(&class->pe_lock);
	memcpy(class->verify_info.auth_data, auth_data, UFCS_VERIFY_AUTH_DATA_SIZE);
	class->verify_pass = false;
	reinit_completion(&class->request_ack);
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		rc = -EINVAL;
		goto err;
	}
	rc = ufcs_push_event(class, event);
	if (rc < 0) {
		ufcs_err("push verify request event error, rc=%d\n", rc);
		goto err;
	}
	mutex_unlock(&class->pe_lock);
	wait_for_completion(&class->request_ack);
	rc = READ_ONCE(class->state.err);
	if (!rc)
		rc = class->verify_pass;
	memzero_explicit(&class->verify_info, sizeof(class->verify_info));
	mutex_unlock(&class->ext_req_lock);

	if (rc < 0) {
		ufcs_err("adapter verify error, rc=%d\n", rc);
		return rc;
	}

	return rc;

err:
	mutex_unlock(&class->pe_lock);
	memzero_explicit(&class->verify_info, sizeof(class->verify_info));
	mutex_unlock(&class->ext_req_lock);
	devm_kfree(&class->ufcs->dev, event);
	return rc;
}

int ufcs_intf_verify_adapter(struct ufcs_dev *ufcs, u8 key_index, u8 *auth_data, u8 data_len)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}
	class = ufcs->class;
	if (auth_data == NULL) {
		ufcs_err("auth_data is NULL\n");
		return -EINVAL;
	}

	if (class->test_mode) {
		ufcs_err("test mode, unable to process external request\n");
		return -EBUSY;
	}

	return ufcs_verify_adapter(class, key_index, auth_data, data_len);
}
EXPORT_SYMBOL(ufcs_intf_verify_adapter);

int ufcs_get_power_change_info(struct ufcs_class *class, u32 *pwr_change_info, int num)
{
	int i;

	if (class == NULL) {
		ufcs_err("ufcs class is NULL\n");
		return -EINVAL;
	}
	if (pwr_change_info == NULL) {
		ufcs_err("pwr_change_info buf is NULL\n");
		return -EINVAL;
	}
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		return -EINVAL;
	}

	if (class->pdo.num == 0) {
		ufcs_err("need get pdo info first\n");
		return -EINVAL;
	}

	if (num < class->pdo.num)
		ufcs_err("pwr_change_info buf too short\n");
	else
		num = class->pdo.num;
	for (i = 0; i < num; i++)
		pwr_change_info[i] = class->pwr_change_info[i];

	return num;
}

int ufcs_intf_get_power_change_info(struct ufcs_dev *ufcs, u32 *pwr_change_info, int num)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}
	class = ufcs->class;
	if (pwr_change_info == NULL) {
		ufcs_err("pwr_change_info buf is NULL\n");
		return -EINVAL;
	}

	if (class->test_mode) {
		ufcs_err("test mode, unable to process external request\n");
		return -EBUSY;
	}

	return ufcs_get_power_change_info(class, pwr_change_info, num);
}
EXPORT_SYMBOL(ufcs_intf_get_power_change_info);

int ufcs_get_emark_info(struct ufcs_class *class, u64 *info)
{
	int rc;
	struct ufcs_event *event;

	if (class == NULL) {
		ufcs_err("ufcs class is NULL\n");
		return -EINVAL;
	}
	if (info == NULL) {
		ufcs_err("info buf is NULL\n");
		return -EINVAL;
	}
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		return -EINVAL;
	}

	event = devm_kzalloc(&class->ufcs->dev, sizeof(struct ufcs_event), GFP_KERNEL);
	if (event == NULL) {
		ufcs_err("alloc event buf error\n");
		return -ENOMEM;
	}

	event->data = NULL;
	event->msg = NULL;
	event->type = UFCS_EVENT_OPLUS_SEND_GET_EMARK_INFO_MSG;
	INIT_LIST_HEAD(&event->list);

	mutex_lock(&class->ext_req_lock);
	mutex_lock(&class->pe_lock);
	reinit_completion(&class->request_ack);
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		rc = -EINVAL;
		goto err;
	}
	rc = ufcs_push_event(class, event);
	if (rc < 0) {
		ufcs_err("push oplus get emark info event error, rc=%d\n", rc);
		goto err;
	}
	mutex_unlock(&class->pe_lock);
	wait_for_completion(&class->request_ack);
	rc = READ_ONCE(class->state.err);
	mutex_unlock(&class->ext_req_lock);

	if (rc < 0) {
		ufcs_err("get emark info error, rc=%d\n", rc);
		return rc;
	}
	*info = class->emark_info;

	return 0;

err:
	mutex_unlock(&class->pe_lock);
	mutex_unlock(&class->ext_req_lock);
	devm_kfree(&class->ufcs->dev, event);
	return rc;
}

int ufcs_intf_get_emark_info(struct ufcs_dev *ufcs, u64 *info)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}
	class = ufcs->class;
	if (info == NULL) {
		ufcs_err("info buf is NULL\n");
		return -EINVAL;
	}

	if (class->test_mode) {
		ufcs_err("test mode, unable to process external request\n");
		return -EBUSY;
	}

	return ufcs_get_emark_info(class, info);
}
EXPORT_SYMBOL(ufcs_intf_get_emark_info);

int ufcs_get_power_info_ext(struct ufcs_class *class, u64 *pie, int num)
{
	int rc;
	struct ufcs_event *event;
	int i;

	if (class == NULL) {
		ufcs_err("ufcs class is NULL\n");
		return -EINVAL;
	}
	if (pie == NULL) {
		ufcs_err("pie buf is NULL\n");
		return -EINVAL;
	}
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		return -EINVAL;
	}

	event = devm_kzalloc(&class->ufcs->dev, sizeof(struct ufcs_event), GFP_KERNEL);
	if (event == NULL) {
		ufcs_err("alloc event buf error\n");
		return -ENOMEM;
	}

	event->data = NULL;
	event->msg = NULL;
	event->type = UFCS_EVENT_OPLUS_SEND_GET_POWER_INFO_MSG;
	INIT_LIST_HEAD(&event->list);

	mutex_lock(&class->ext_req_lock);
	mutex_lock(&class->pe_lock);
	reinit_completion(&class->request_ack);
	if (!class->handshake_success) {
		ufcs_err("ufcs no handshake\n");
		rc = -EINVAL;
		goto err;
	}
	rc = ufcs_push_event(class, event);
	if (rc < 0) {
		ufcs_err("push oplus get power info event error, rc=%d\n", rc);
		goto err;
	}
	mutex_unlock(&class->pe_lock);
	wait_for_completion(&class->request_ack);
	rc = READ_ONCE(class->state.err);
	mutex_unlock(&class->ext_req_lock);

	if (rc < 0) {
		ufcs_err("get power info error, rc=%d\n", rc);
		return rc;
	}

	if (num < class->pie.num)
		ufcs_err("pie buf too short\n");
	else
		num = class->pie.num;
	for (i = 0; i < num; i++)
		pie[i] = class->pie.data[i];

	return num;

err:
	mutex_unlock(&class->pe_lock);
	mutex_unlock(&class->ext_req_lock);
	devm_kfree(&class->ufcs->dev, event);
	return rc;
}

int ufcs_intf_get_power_info_ext(struct ufcs_dev *ufcs, u64 *pie, int num)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return -EINVAL;
	}
	class = ufcs->class;
	if (pie == NULL) {
		ufcs_err("pie buf is NULL\n");
		return -EINVAL;
	}

	if (class->test_mode) {
		ufcs_err("test mode, unable to process external request\n");
		return -EBUSY;
	}

	return ufcs_get_power_info_ext(class, pie, num);
}
EXPORT_SYMBOL(ufcs_intf_get_power_info_ext);

bool ufcs_is_test_mode(struct ufcs_dev *ufcs)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return false;
	}
	class = ufcs->class;

	return class->test_mode;
}
EXPORT_SYMBOL(ufcs_is_test_mode);

bool ufcs_is_vol_acc_test_mode(struct ufcs_dev *ufcs)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return false;
	}
	class = ufcs->class;

	return class->vol_acc_test;
}
EXPORT_SYMBOL(ufcs_is_vol_acc_test_mode);

bool ufcs_handshake_success(struct ufcs_dev *ufcs)
{
	struct ufcs_class *class;

	if (ufcs == NULL) {
		ufcs_err("ufcs is NULL\n");
		return false;
	}
	class = ufcs->class;

	return class->handshake_success;
}
EXPORT_SYMBOL(ufcs_handshake_success);
