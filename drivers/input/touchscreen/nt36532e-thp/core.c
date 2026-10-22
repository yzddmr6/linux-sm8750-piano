// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2010 - 2022 Novatek, Inc.
 * SPI paging/host-download protocol from the piano vendor driver.
 * NTP1 framing follows ianchb's NT36532E THP stream. See README.rst.
 */
#include <drm/drm_panel.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/kref.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include "firmware.h"

#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36532E_THP_KUNIT_TEST)
#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/kthread.h>
#endif

#define NVT_WRITE_SIZE	(63 * 1024)
#define NVT_READ_SIZE	8192
#define NVT_FRAME_PREFIX	257
#define NVT_STREAM_MAGIC	0x3150544e
#define NVT_FRAME_VALID	BIT(0)
#define NVT_FRAME_EPOCH	BIT(1)
#define NVT_IRQ_ENABLED	0
/* Vendor nvt_check_fw_reset_state() budget for post-INIT states: 50 x 10 ms. */
#define NVT_FIRST_FRAME_MS	500
/* Consecutive frame reads allowed to fail with a recovered SPI channel. */
#define NVT_READ_RETRIES	1

struct nvt_stream_header {
	__le32 magic;
	__le16 header_length;
	__le16 frame_length;
	__le64 sequence;
	__le64 timestamp_ns;
	__le16 checksum;
	__le16 flags;
	__le32 firmware_length_words;
} __packed;

struct nvt_thp {
	struct spi_device *spi;
	struct regmap *map;
	struct miscdevice misc;
	struct drm_panel_follower follower;
	struct kref ref;
	/* API transitions serialize across cancellation; the worker never takes this. */
	struct mutex transition_lock;
	struct mutex lifecycle_lock; /* Serializes synchronous open and the worker. */
	struct work_struct start_work;
	struct work_struct charger_work;
	struct notifier_block charger_notifier;
	int charger_state, charger_sent; /* lifecycle_lock; -1 means unknown. */
	bool start_requested; /* transition_lock; one attempt until the next stop. */
#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36532E_THP_KUNIT_TEST)
	int (*test_start)(struct nvt_thp *nvt);
	void (*test_stop)(struct nvt_thp *nvt);
	struct completion *test_quiescing;
#endif
	struct mutex read_lock; /* Keeps the FIFO single-consumer across shared fds. */
	wait_queue_head_t wait;
	struct kfifo fifo;
	const char *firmware_name;
	int lcd_id; /* Panel module ID pin level; -1 without the pin. */
	u8 *tx, *rx, *record;
	size_t max_read;
	u16 frame_length;
	u8 reset_state; /* Last successful reset-state sample for this start. */
	u64 sequence;
	unsigned long flags;
	unsigned int read_errors;
	int error;
	bool dead, opened, running, epoch, suspended, panel_off;
};

static_assert(sizeof(struct nvt_stream_header) == 32);

static void nvt_destroy(struct kref *ref)
{
	struct nvt_thp *nvt = container_of(ref, struct nvt_thp, ref);

	kfifo_free(&nvt->fifo);
	kfree(nvt->record);
	kfree(nvt->rx);
	kfree(nvt->tx);
	kfree(nvt->misc.name);
	kfree(nvt);
}

static void nvt_put(void *data)
{
	struct nvt_thp *nvt = data;

	kref_put(&nvt->ref, nvt_destroy);
}

static int nvt_transfer(struct nvt_thp *nvt, size_t size, bool read)
{
	struct spi_transfer transfer = {
		.tx_buf = nvt->tx,
		.rx_buf = read ? nvt->rx : NULL,
		.len = size,
	};
	struct spi_message message;

	spi_message_init_with_transfers(&message, &transfer, 1);
	/*
	 * Only the spi_sync() status is meaningful: spi-geni-qcom's GPI DMA
	 * callback sets cur_msg->status before finalizing the transfer, so the
	 * SPI core never adds the transfer to actual_length on success.
	 */
	return spi_sync(nvt->spi, &message);
}

static int nvt_select_page(struct nvt_thp *nvt, u32 address)
{
	nvt->tx[0] = 0xff;
	nvt->tx[1] = address >> 15;
	nvt->tx[2] = address >> 7;
	return nvt_transfer(nvt, 3, false);
}

static int nvt_spi_read(void *context, const void *reg, size_t reg_size,
			void *value, size_t size)
{
	struct nvt_thp *nvt = context;
	u32 address;
	int ret;

	if (reg_size != sizeof(u32) || !size || size > nvt->max_read)
		return -EINVAL;
	address = get_unaligned((const u32 *)reg);
	ret = nvt_select_page(nvt, address);
	if (ret)
		return ret;
	memset(nvt->tx, 0, size + 2);
	nvt->tx[0] = address & 0x7f;
	ret = nvt_transfer(nvt, size + 2, true);
	if (!ret)
		memcpy(value, nvt->rx + 2, size);
	return ret;
}

static int nvt_spi_write(void *context, const void *data, size_t count)
{
	struct nvt_thp *nvt = context;
	u32 address;
	int ret;

	if (count <= sizeof(u32) || count - sizeof(u32) > NVT_WRITE_SIZE)
		return -EINVAL;
	address = get_unaligned((const u32 *)data);
	ret = nvt_select_page(nvt, address);
	if (ret)
		return ret;
	nvt->tx[0] = (address & 0x7f) | 0x80;
	memcpy(nvt->tx + 1, (const u8 *)data + sizeof(u32), count - sizeof(u32));
	return nvt_transfer(nvt, count - sizeof(u32) + 1, false);
}

static bool nvt_precious_register(struct device *dev, unsigned int reg)
{
	/* Do not turn a debugfs register dump into controller commands. */
	return true;
}

static bool nvt_valid_frame(const u8 *payload, size_t size)
{
	u32 words = get_unaligned_le32(payload + 8), sum;
	u16 crc = get_unaligned_le16(payload + 4);
	size_t offset;

	if (payload[0] != 0xa5 || words > (size - 20) / 4 ||
	    get_unaligned_le32(payload + 16) != ~words ||
	    get_unaligned_le16(payload + 12) != (u16)~crc)
		return false;
	sum = crc;
	for (offset = 20; offset < 20 + (size_t)words * 4; offset += 2)
		sum += get_unaligned_le16(payload + offset);
	return !(sum & 0xffff);
}

static irqreturn_t nvt_irq(int irq, void *data)
{
	struct nvt_thp *nvt = data;
	struct nvt_stream_header *header = (void *)nvt->record;
	u8 *frame = nvt->record + sizeof(*header);
	u8 *payload = frame + NVT_FRAME_PREFIX;
	size_t length = sizeof(*header) + nvt->frame_length;
	bool valid;
	int ret;

	if (!READ_ONCE(nvt->running))
		return IRQ_HANDLED;
	frame[0] = 0; /* Same command placeholder as the established NTP1 stream. */
	ret = regmap_bulk_read(nvt->map, 0x11c400, frame + 1, nvt->frame_length - 1);
	if (ret == -ETIMEDOUT || ret == -EIO) {
		/*
		 * A GSI transfer timeout is handled below the SPI core: the
		 * controller terminates, resets and restarts its DMA channels
		 * before the error reaches us, so the next frame can be read
		 * normally. Losing this frame is no worse than a missed IRQ.
		 * Only a second consecutive failure stops the stream.
		 */
		if (nvt->read_errors++ < NVT_READ_RETRIES) {
			dev_warn_ratelimited(&nvt->spi->dev,
					     "THP SPI read failed: %d; keeping the stream\n", ret);
			return IRQ_HANDLED;
		}
	}
	if (ret) {
		WRITE_ONCE(nvt->error, ret);
		WRITE_ONCE(nvt->running, false);
		if (test_and_clear_bit(NVT_IRQ_ENABLED, &nvt->flags))
			disable_irq_nosync(irq);
		dev_err_ratelimited(&nvt->spi->dev, "THP SPI read failed: %d\n", ret);
		wake_up_interruptible(&nvt->wait);
		return IRQ_HANDLED;
	}
	nvt->read_errors = 0;
	valid = nvt_valid_frame(payload, nvt->frame_length - NVT_FRAME_PREFIX);
	header->magic = cpu_to_le32(NVT_STREAM_MAGIC);
	header->header_length = cpu_to_le16(sizeof(*header));
	header->frame_length = cpu_to_le16(nvt->frame_length);
	header->sequence = cpu_to_le64(++nvt->sequence);
	header->timestamp_ns = cpu_to_le64(ktime_get_boottime_ns());
	header->checksum = cpu_to_le16(get_unaligned_le16(payload + 4));
	header->flags = cpu_to_le16((valid ? NVT_FRAME_VALID : 0) |
				   (valid && nvt->epoch ? NVT_FRAME_EPOCH : 0));
	header->firmware_length_words = cpu_to_le32(get_unaligned_le32(payload + 8));
	/* One producer, one serialized reader. Never publish half a record. */
	if (kfifo_avail(&nvt->fifo) >= length) {
		kfifo_in(&nvt->fifo, nvt->record, length);
		if (valid)
			WRITE_ONCE(nvt->epoch, false);
	}
	wake_up_interruptible(&nvt->wait);
	return IRQ_HANDLED;
}

static void nvt_stop_irq(struct nvt_thp *nvt)
{
	WRITE_ONCE(nvt->running, false);
	nvt->charger_sent = -1;
#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36532E_THP_KUNIT_TEST)
	if (nvt->test_stop) {
		nvt->test_stop(nvt);
		return;
	}
#endif
	if (test_and_clear_bit(NVT_IRQ_ENABLED, &nvt->flags))
		disable_irq(nvt->spi->irq);
	else
		synchronize_irq(nvt->spi->irq);
}

/* Only ONLINE from an external system supply counts, never USB enumeration. */
static int nvt_external_supply(struct power_supply *psy, void *data)
{
	union power_supply_propval value;
	int *online = data;

	if (psy->desc->type == POWER_SUPPLY_TYPE_BATTERY ||
	    psy->desc->type == POWER_SUPPLY_TYPE_UNKNOWN)
		return 0;
	if (!power_supply_get_property(psy, POWER_SUPPLY_PROP_SCOPE, &value) &&
	    value.intval == POWER_SUPPLY_SCOPE_DEVICE)
		return 0;
	if (power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &value))
		return 0;
	*online = !!value.intval;
	return *online;
}

static void nvt_charger_state(struct nvt_thp *nvt, int online)
{
	u8 command[] = { 0x51, 0x00 };
	int ret;

	lockdep_assert_held(&nvt->lifecycle_lock);
	if (online >= 0)
		nvt->charger_state = !!online;
	if (nvt->charger_state < 0)
		return; /* No readable supply and no cached state. */
	if (READ_ONCE(nvt->dead) || READ_ONCE(nvt->panel_off) ||
	    READ_ONCE(nvt->suspended) || !READ_ONCE(nvt->opened) ||
	    !READ_ONCE(nvt->running) || nvt->charger_sent == nvt->charger_state)
		return;
	command[0] = nvt->charger_state ? 0x53 : 0x51;
	/* regmap serializes page selection and the write against IRQ frame reads. */
	ret = regmap_bulk_write(nvt->map, 0x11c450, command, sizeof(command));
	if (ret) {
		dev_err(&nvt->spi->dev, "THP charger state %d failed: %d\n",
			nvt->charger_state, ret);
		WRITE_ONCE(nvt->error, ret);
		nvt_stop_irq(nvt);
		wake_up_interruptible(&nvt->wait);
		return; /* The existing reader/service error path owns recovery. */
	}
	nvt->charger_sent = nvt->charger_state;
	dev_info(&nvt->spi->dev, "THP charger state %d sent\n", nvt->charger_sent);
}

static void nvt_charger_work(struct work_struct *work)
{
	struct nvt_thp *nvt = container_of(work, struct nvt_thp, charger_work);
	int online = -ENODEV;

	mutex_lock(&nvt->lifecycle_lock);
	if (!READ_ONCE(nvt->dead) && !READ_ONCE(nvt->suspended)) {
		power_supply_for_each_psy(&online, nvt_external_supply);
		nvt_charger_state(nvt, online);
	}
	mutex_unlock(&nvt->lifecycle_lock);
}

static int nvt_charger_notify(struct notifier_block *nb, unsigned long event, void *data)
{
	struct nvt_thp *nvt = container_of(nb, struct nvt_thp, charger_notifier);

	if (event == PSY_EVENT_PROP_CHANGED)
		queue_work(system_freezable_wq, &nvt->charger_work);
	return NOTIFY_OK;
}

static void nvt_charger_unregister(void *data)
{
	struct nvt_thp *nvt = data;

	power_supply_unreg_notifier(&nvt->charger_notifier);
	cancel_work_sync(&nvt->charger_work);
}

/* Negative entries are read errors, never firmware values. */
static void nvt_read_modes(struct nvt_thp *nvt, const char *phase, int modes[3])
{
	static const u8 commands[] = { 0x01, 0x19, 0x04 };
	struct nvt_thp_command_status last;
	u16 value;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(commands); i++) {
		ret = nvt_thp_get_extend_command(nvt->map, commands[i], &value, &last);
		modes[i] = ret ? ret : value;
		if (ret)
			dev_info(&nvt->spi->dev,
				 "THP %s read BF %02x error %d, command byte %#04x\n",
				 phase, commands[i], ret, last.status);
	}
	dev_info(&nvt->spi->dev,
		 "THP %s readback: op mode=%d, idle baseline=%d, stylus=%d (negative=read error)\n",
		 phase, modes[0], modes[1], modes[2]);
}

/* Caller holds lifecycle_lock, with the IRQ stopped. */
static int nvt_start_stream(struct nvt_thp *nvt)
{
	const struct firmware *firmware;
	struct nvt_thp_command_status last;
	struct nvt_thp_firmware image;
	struct nvt_thp_boot_status boot = {};
	s64 first_frame_ms, reset_to_frame_ms, commands_ms, normal_run_ms;
	u16 payload_length;
	u8 first_frame_state = 0;
	ktime_t start;
	int ret, state_ret, modes[3];

	lockdep_assert_held(&nvt->lifecycle_lock);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36532E_THP_KUNIT_TEST)
	if (nvt->test_start)
		return nvt->test_start(nvt);
#endif
	WRITE_ONCE(nvt->error, 0);
	nvt->read_errors = 0;
	nvt->reset_state = 0;
	ret = request_firmware_direct(&firmware, nvt->firmware_name, &nvt->spi->dev);
	if (ret)
		return ret;
	ret = nvt_thp_parse_firmware(firmware->data, firmware->size, &image);
	if (!ret) {
		ret = nvt_thp_boot_firmware(nvt->map, firmware, &image, &payload_length, &boot);
		nvt_thp_free_firmware(&image);
	}
	release_firmware(firmware);
	nvt->reset_state = boot.reset_state;
	if (ret) {
		dev_err(&nvt->spi->dev, "THP firmware boot failed: %d, last reset state %#04x\n",
			ret, boot.reset_state);
		return ret;
	}
	/* IRQ packets must fit one transaction; do not split a changing frame. */
	if (payload_length < 64 || payload_length > nvt->max_read - (NVT_FRAME_PREFIX - 1)) {
		ret = -EMSGSIZE;
		return ret;
	}
	nvt->frame_length = NVT_FRAME_PREFIX + payload_length;
	WRITE_ONCE(nvt->epoch, true);
	WRITE_ONCE(nvt->running, true);
	set_bit(NVT_IRQ_ENABLED, &nvt->flags);
	enable_irq(nvt->spi->irq);
	/*
	 * REK is the firmware readiness contract. Retain the established
	 * first-frame admission as well; the frame stays queued for the reader.
	 */
	start = ktime_get();
	ret = wait_event_interruptible_timeout(nvt->wait,
					       !READ_ONCE(nvt->epoch) || READ_ONCE(nvt->error),
					       msecs_to_jiffies(NVT_FIRST_FRAME_MS));
	if (!ret)
		ret = -ETIMEDOUT;
	else if (ret > 0)
		ret = READ_ONCE(nvt->error);
	if (ret) {
		state_ret = nvt_thp_read_reset_state(nvt->map, &nvt->reset_state);
		dev_err(&nvt->spi->dev,
			"THP first valid frame not received: %d, last reset state %#04x (read %d)\n",
			ret, nvt->reset_state, state_ret);
		goto stop;
	}
	first_frame_ms = ktime_ms_delta(ktime_get(), start);
	reset_to_frame_ms = ktime_ms_delta(ktime_get(), boot.reset_time);
	/* Sample readiness after the first frame, before any host command. */
	start = ktime_get();
	ret = nvt_thp_read_reset_state(nvt->map, &first_frame_state);
	nvt->reset_state = first_frame_state;
	if (!ret)
		ret = nvt_thp_wait_normal_run(nvt->map, &nvt->reset_state);
	normal_run_ms = ktime_ms_delta(ktime_get(), start);
	if (ret) {
		dev_err(&nvt->spi->dev,
			"THP NORMAL_RUN wait failed: %d, first-frame reset state %#04x, last reset state %#04x, first-frame-to-NORMAL_RUN %lld ms\n",
			ret, first_frame_state, nvt->reset_state, normal_run_ms);
		goto stop;
	}
	start = ktime_get();
	ret = nvt_thp_apply_host_commands(nvt->map, &last);
	commands_ms = ktime_ms_delta(ktime_get(), start);
	/* Diagnostic read: preserve the last sample and command result on error. */
	state_ret = nvt_thp_read_reset_state(nvt->map, &nvt->reset_state);
	if (ret) {
		nvt_read_modes(nvt, "post-failure", modes);
		dev_err(&nvt->spi->dev,
			"THP host command BF %02x failed: %d, command byte %#04x, last reset state %#04x (read %d); post-failure op mode=%d, idle baseline=%d, stylus=%d; first-frame reset state %#04x, first-frame-to-NORMAL_RUN %lld ms\n",
			last.sub_command, ret, last.status, nvt->reset_state, state_ret,
			modes[0], modes[1], modes[2], first_frame_state, normal_run_ms);
		goto stop;
	}
	ret = READ_ONCE(nvt->error);
	if (ret)
		goto stop;
	dev_info(&nvt->spi->dev,
		 "THP first frame after %lld ms; stylus disabled, idle-baseline update and op mode 2 accepted\n",
		 first_frame_ms);
	dev_info(&nvt->spi->dev,
		 "THP stream ready, payload %u bytes; reset-to-REK %lld ms, reset-to-first-frame %lld ms, BF sequence %lld ms, last reset state %#04x (read %d); first-frame reset state %#04x, first-frame-to-NORMAL_RUN %lld ms\n",
		 payload_length, boot.rek_ms, reset_to_frame_ms, commands_ms,
		 nvt->reset_state, state_ret, first_frame_state, normal_run_ms);
	/* Firmware reload loses the setting, even if ONLINE has not changed. */
	nvt->charger_sent = -1;
	queue_work(system_freezable_wq, &nvt->charger_work);
	return 0;
stop:
	nvt_stop_irq(nvt);
	return ret;
}

static void nvt_start_work(struct work_struct *work)
{
	struct nvt_thp *nvt = container_of(work, struct nvt_thp, start_work);
	ktime_t start;
	int ret;

	mutex_lock(&nvt->lifecycle_lock);
	/* Stop transitions publish their gate before cancelling this work. */
	if (!READ_ONCE(nvt->opened) || READ_ONCE(nvt->panel_off) ||
	    READ_ONCE(nvt->suspended) || READ_ONCE(nvt->dead) ||
	    READ_ONCE(nvt->running))
		goto unlock;
	start = ktime_get();
	dev_dbg(&nvt->spi->dev, "THP worker start\n");
	ret = nvt_start_stream(nvt);
	if (ret) {
		WRITE_ONCE(nvt->error, ret);
		dev_err(&nvt->spi->dev, "THP async restart failed: %d, last reset state %#04x\n",
			ret, nvt->reset_state);
		wake_up_interruptible(&nvt->wait);
	}
	dev_dbg(&nvt->spi->dev, "THP stream %s: %d after %lld ms\n",
		ret ? "failed" : "ready", ret, ktime_ms_delta(ktime_get(), start));
unlock:
	mutex_unlock(&nvt->lifecycle_lock);
}

static void nvt_queue_start(struct nvt_thp *nvt)
{
	lockdep_assert_held(&nvt->transition_lock);
	if (!nvt->dead && nvt->opened && !nvt->panel_off && !nvt->suspended &&
	    !READ_ONCE(nvt->running) && !nvt->start_requested) {
		nvt->start_requested = true;
		/* Readers must not see the previous failure while start is queued. */
		WRITE_ONCE(nvt->error, 0);
		dev_dbg(&nvt->spi->dev, "THP follower/resume queued\n");
		schedule_work(&nvt->start_work);
	}
}

/*
 * Caller holds transition_lock and has closed a start gate. Holding it across
 * cancellation excludes enable/resume/open until IRQ shutdown is complete.
 * Never cancel under lifecycle_lock: the worker needs it to finish or bail.
 */
static void nvt_quiesce(struct nvt_thp *nvt)
{
	lockdep_assert_held(&nvt->transition_lock);
#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36532E_THP_KUNIT_TEST)
	if (nvt->test_quiescing)
		complete(nvt->test_quiescing);
#endif
	cancel_work_sync(&nvt->start_work);
	cancel_work_sync(&nvt->charger_work);
	nvt->start_requested = false;
	mutex_lock(&nvt->lifecycle_lock);
	nvt_stop_irq(nvt);
	mutex_unlock(&nvt->lifecycle_lock);
}

static int nvt_panel_disabling(struct drm_panel_follower *follower)
{
	struct nvt_thp *nvt = container_of(follower, struct nvt_thp, follower);

	mutex_lock(&nvt->transition_lock);
	if (!nvt->panel_off && !nvt->dead) {
		WRITE_ONCE(nvt->panel_off, true);
		nvt_quiesce(nvt);
	}
	mutex_unlock(&nvt->transition_lock);
	return 0;
}

static int nvt_panel_enabled(struct drm_panel_follower *follower)
{
	struct nvt_thp *nvt = container_of(follower, struct nvt_thp, follower);

	mutex_lock(&nvt->transition_lock);
	if (!nvt->dead) {
		if (nvt->panel_off) {
			WRITE_ONCE(nvt->error, 0);
			WRITE_ONCE(nvt->panel_off, false);
		}
		/* Preserve queued bytes, including a partially read NTP1 record. */
		wake_up_interruptible(&nvt->wait);
		nvt_queue_start(nvt);
	}
	mutex_unlock(&nvt->transition_lock);
	return 0;
}

static const struct drm_panel_follower_funcs nvt_panel_follower_funcs = {
	/* TDDI scanning needs the display timing, not just panel power. */
	.panel_enabled = nvt_panel_enabled,
	.panel_disabling = nvt_panel_disabling,
	.panel_unpreparing = nvt_panel_disabling,
};

static int nvt_open(struct inode *inode, struct file *file)
{
	struct nvt_thp *nvt = container_of(file->private_data, struct nvt_thp, misc);
	int ret;

	if (file->f_mode & FMODE_WRITE)
		return -EPERM;
	ret = mutex_lock_interruptible(&nvt->transition_lock);
	if (ret)
		return ret;
	mutex_lock(&nvt->lifecycle_lock);
	if (nvt->dead) {
		ret = -ENODEV;
		goto unlock;
	}
	if (nvt->opened || nvt->suspended) {
		ret = -EBUSY;
		goto unlock;
	}
	kfifo_reset(&nvt->fifo);
	WRITE_ONCE(nvt->error, 0);
	/* udev may open the stream before the panel has been enabled. */
	if (!nvt->panel_off) {
		ret = nvt_start_stream(nvt);
		if (ret) {
			dev_err(&nvt->spi->dev, "THP stream start failed: %d, last reset state %#04x\n",
				ret, nvt->reset_state);
			goto unlock;
		}
	}
	WRITE_ONCE(nvt->opened, true);
	kref_get(&nvt->ref);
	file->private_data = nvt;
	stream_open(inode, file);
unlock:
	mutex_unlock(&nvt->lifecycle_lock);
	mutex_unlock(&nvt->transition_lock);
	return ret;
}

static int nvt_release(struct inode *inode, struct file *file)
{
	struct nvt_thp *nvt = file->private_data;

	mutex_lock(&nvt->transition_lock);
	WRITE_ONCE(nvt->opened, false);
	if (!nvt->dead)
		nvt_quiesce(nvt);
	mutex_unlock(&nvt->transition_lock);
	nvt_put(nvt);
	return 0;
}

static bool nvt_read_ready(struct nvt_thp *nvt)
{
	return READ_ONCE(nvt->dead) ||
	       (!READ_ONCE(nvt->panel_off) && !READ_ONCE(nvt->suspended) &&
		(READ_ONCE(nvt->error) || !kfifo_is_empty(&nvt->fifo)));
}

static ssize_t nvt_read(struct file *file, char __user *buffer, size_t count, loff_t *position)
{
	struct nvt_thp *nvt = file->private_data;
	unsigned int copied;
	int ret;

	if (!count)
		return 0;
	ret = mutex_lock_interruptible(&nvt->read_lock);
	if (ret)
		return ret;
	if (!nvt_read_ready(nvt)) {
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto unlock;
		}
		ret = wait_event_interruptible(nvt->wait, nvt_read_ready(nvt));
		if (ret)
			goto unlock;
	}
	if (READ_ONCE(nvt->dead)) {
		ret = -ENODEV;
		goto unlock;
	}
	ret = READ_ONCE(nvt->error);
	if (!ret) {
		ret = kfifo_to_user(&nvt->fifo, buffer, min_t(size_t, count, UINT_MAX), &copied);
		if (!ret)
			ret = copied;
	}
unlock:
	mutex_unlock(&nvt->read_lock);
	return ret;
}

static __poll_t nvt_poll(struct file *file, poll_table *wait)
{
	struct nvt_thp *nvt = file->private_data;

	poll_wait(file, &nvt->wait, wait);
	if (READ_ONCE(nvt->dead))
		return EPOLLERR | EPOLLHUP;
	if (READ_ONCE(nvt->panel_off) || READ_ONCE(nvt->suspended))
		return 0;
	if (READ_ONCE(nvt->error))
		return EPOLLERR | EPOLLHUP;
	return kfifo_is_empty(&nvt->fifo) ? 0 : EPOLLIN | EPOLLRDNORM;
}

static const struct file_operations nvt_fops = {
	.owner = THIS_MODULE,
	.open = nvt_open,
	.release = nvt_release,
	.read = nvt_read,
	.poll = nvt_poll,
};

/* Stock p81 lcd_id levels: 0 is the CSOT panel module, 1 the BOE module. */
static const char *const nvt_panel_vendors[] = { "csot", "boe" };

/*
 * firmware-name holds one image per panel module, indexed by the lcd-id
 * level. Without the ID pin it must name exactly one image.
 */
static int nvt_select_firmware(struct device *dev, int lcd_id, const char **name)
{
	const char *names[ARRAY_SIZE(nvt_panel_vendors)];
	int count, index = 0;

	count = device_property_string_array_count(dev, "firmware-name");
	if (count <= 0)
		return dev_err_probe(dev, count ?: -ENODATA, "firmware-name is required\n");
	if (lcd_id >= 0)
		index = lcd_id;
	else if (count > 1)
		return dev_err_probe(dev, -EINVAL, "several firmware names but no lcd-id GPIO\n");
	if (index >= min_t(int, count, ARRAY_SIZE(names)))
		return dev_err_probe(dev, -EINVAL, "no firmware-name for lcd-id %d\n", lcd_id);
	count = device_property_read_string_array(dev, "firmware-name", names, ARRAY_SIZE(names));
	if (count < 0)
		return count;
	*name = names[index];
	return 0;
}

static ssize_t panel_vendor_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	struct nvt_thp *nvt = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", nvt_panel_vendors[nvt->lcd_id]);
}
static DEVICE_ATTR_RO(panel_vendor);

static struct attribute *nvt_panel_attrs[] = {
	&dev_attr_panel_vendor.attr,
	NULL
};

static const struct attribute_group nvt_panel_group = {
	.attrs = nvt_panel_attrs,
};

/* Read the panel module ID pin and select the firmware for that module. */
static int nvt_probe_panel_module(struct nvt_thp *nvt)
{
	struct device *dev = &nvt->spi->dev;
	struct gpio_desc *lcd_id;
	int ret;

	nvt->lcd_id = -1;
	lcd_id = devm_gpiod_get_optional(dev, "novatek,lcd-id", GPIOD_IN);
	if (IS_ERR(lcd_id))
		return dev_err_probe(dev, PTR_ERR(lcd_id), "Failed to get lcd-id GPIO\n");
	if (lcd_id) {
		ret = gpiod_get_value_cansleep(lcd_id);
		if (ret < 0)
			return dev_err_probe(dev, ret, "Failed to read lcd-id GPIO\n");
		nvt->lcd_id = ret;
	}
	ret = nvt_select_firmware(dev, nvt->lcd_id, &nvt->firmware_name);
	if (ret)
		return ret;
	if (lcd_id)
		dev_info(dev, "lcd-id %d (%s), firmware %s\n", nvt->lcd_id,
			 nvt_panel_vendors[nvt->lcd_id], nvt->firmware_name);
	return 0;
}

static int nvt_probe(struct spi_device *spi)
{
	struct regmap_config config = {
		/* Config callbacks use native-endian register formatting. */
		.reg_bits = 32,
		.val_bits = 8,
		.read = nvt_spi_read,
		.write = nvt_spi_write,
		.max_register = 0x7fffff,
		.cache_type = REGCACHE_NONE,
		.precious_reg = nvt_precious_register,
	};
	struct nvt_thp *nvt;
	size_t transfer_max;
	int ret;

	if (spi->irq <= 0 || (spi->mode & (SPI_CPOL | SPI_CPHA)) ||
	    (spi->controller->flags & SPI_CONTROLLER_HALF_DUPLEX))
		return -EINVAL;
	transfer_max = min(spi_max_transfer_size(spi), spi_max_message_size(spi));
	if (transfer_max < NVT_FRAME_PREFIX + 64 + 1)
		return -EMSGSIZE;
	nvt = kzalloc_obj(*nvt);
	if (!nvt)
		return -ENOMEM;
	kref_init(&nvt->ref);
	ret = devm_add_action_or_reset(&spi->dev, nvt_put, nvt);
	if (ret)
		return ret;
	nvt->spi = spi;
	nvt->max_read = min_t(size_t, NVT_READ_SIZE - 2, transfer_max - 2);
	mutex_init(&nvt->transition_lock);
	mutex_init(&nvt->lifecycle_lock);
	INIT_WORK(&nvt->start_work, nvt_start_work);
	INIT_WORK(&nvt->charger_work, nvt_charger_work);
	nvt->charger_state = nvt->charger_sent = -1;
	mutex_init(&nvt->read_lock);
	init_waitqueue_head(&nvt->wait);
	ret = nvt_probe_panel_module(nvt);
	if (ret)
		return ret;
	nvt->tx = kmalloc(NVT_WRITE_SIZE + 1, GFP_KERNEL);
	nvt->rx = kmalloc(NVT_READ_SIZE, GFP_KERNEL);
	nvt->record = kmalloc(sizeof(struct nvt_stream_header) + NVT_READ_SIZE, GFP_KERNEL);
	nvt->misc.name = kasprintf(GFP_KERNEL, "nvt-thp-%s", dev_name(&spi->dev));
	if (!nvt->tx || !nvt->rx || !nvt->record || !nvt->misc.name)
		return -ENOMEM;
	ret = kfifo_alloc(&nvt->fifo, SZ_256K, GFP_KERNEL);
	if (ret)
		return ret;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;
	config.max_raw_read = nvt->max_read;
	config.max_raw_write = min_t(size_t, NVT_WRITE_SIZE, transfer_max - 1);
	nvt->map = devm_regmap_init(&spi->dev, NULL, nvt, &config);
	if (IS_ERR(nvt->map))
		return PTR_ERR(nvt->map);
	ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL, nvt_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN, dev_name(&spi->dev), nvt);
	if (ret)
		return ret;
	spi_set_drvdata(spi, nvt);
	if (drm_is_panel_follower(&spi->dev)) {
		nvt->panel_off = true;
		nvt->follower.funcs = &nvt_panel_follower_funcs;
		ret = devm_drm_panel_add_follower(&spi->dev, &nvt->follower);
		if (ret)
			return dev_err_probe(&spi->dev, ret, "Failed to follow panel\n");
	}
	nvt->charger_notifier.notifier_call = nvt_charger_notify;
	ret = power_supply_reg_notifier(&nvt->charger_notifier);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&spi->dev, nvt_charger_unregister, nvt);
	if (ret)
		return ret;
	/* Cache the initial state without starting the transport or issuing I/O. */
	queue_work(system_freezable_wq, &nvt->charger_work);
	nvt->misc.minor = MISC_DYNAMIC_MINOR;
	nvt->misc.fops = &nvt_fops;
	nvt->misc.parent = &spi->dev;
	nvt->misc.mode = 0600;
	/* The panel module is readable before the misc device announces the stream. */
	if (nvt->lcd_id >= 0) {
		ret = devm_device_add_group(&spi->dev, &nvt_panel_group);
		if (ret)
			return ret;
	}
	return misc_register(&nvt->misc);
}

static void nvt_shutdown(struct spi_device *spi)
{
	struct nvt_thp *nvt = spi_get_drvdata(spi);

	mutex_lock(&nvt->transition_lock);
	WRITE_ONCE(nvt->dead, true);
	nvt_quiesce(nvt);
	wake_up_interruptible(&nvt->wait);
	mutex_unlock(&nvt->transition_lock);
}

static void nvt_remove(struct spi_device *spi)
{
	struct nvt_thp *nvt = spi_get_drvdata(spi);

	/* Do not hold transition_lock while misc_open may be waiting for it. */
	misc_deregister(&nvt->misc);
	nvt_shutdown(spi);
}

static const struct of_device_id nvt_of_match[] = {
	{ .compatible = "novatek,nt36532e" },
	{ }
};
MODULE_DEVICE_TABLE(of, nvt_of_match);

static const struct spi_device_id nvt_ids[] = {
	{ "nt36532e" },
	{ }
};
MODULE_DEVICE_TABLE(spi, nvt_ids);

static int nvt_suspend(struct device *dev)
{
	struct nvt_thp *nvt = dev_get_drvdata(dev);
	bool panel_off;

	mutex_lock(&nvt->transition_lock);
	panel_off = nvt->panel_off;
	WRITE_ONCE(nvt->suspended, true);
	/* A disabled panel has already cancelled work and stopped its IRQ. */
	if (!nvt->dead && !panel_off)
		nvt_quiesce(nvt);
	else
		/* ONLINE callbacks may query other devices: drain even with panel off. */
		cancel_work_sync(&nvt->charger_work);
	mutex_unlock(&nvt->transition_lock);
	return 0;
}

static int nvt_resume(struct device *dev)
{
	struct nvt_thp *nvt = dev_get_drvdata(dev);

	mutex_lock(&nvt->transition_lock);
	WRITE_ONCE(nvt->suspended, false);
	/* The later of panel enable and SPI resume queues the single attempt. */
	nvt_queue_start(nvt);
	wake_up_interruptible(&nvt->wait);
	mutex_unlock(&nvt->transition_lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(nvt_pm, nvt_suspend, nvt_resume);

static struct spi_driver nvt_driver = {
	.driver = {
		.name = "nt36532e-thp",
		.of_match_table = nvt_of_match,
		.pm = pm_sleep_ptr(&nvt_pm),
	},
	.probe = nvt_probe,
	.remove = nvt_remove,
	.shutdown = nvt_shutdown,
	.id_table = nvt_ids,
};
module_spi_driver(nvt_driver);

#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36532E_THP_KUNIT_TEST)
static void nvt_test_free_fifo(void *data)
{
	struct nvt_thp *nvt = data;

	kfifo_free(&nvt->fifo);
}

static void nvt_test_stop_noop(struct nvt_thp *nvt)
{
}

static void nvt_test_open_disabled_panel(struct kunit *test)
{
	struct nvt_thp *nvt = kunit_kzalloc(test, sizeof(*nvt), GFP_KERNEL);
	struct file file = { .f_mode = FMODE_READ };
	struct file second = { .f_mode = FMODE_READ };
	struct inode inode = {};

	KUNIT_ASSERT_NOT_NULL(test, nvt);
	mutex_init(&nvt->transition_lock);
	mutex_init(&nvt->lifecycle_lock);
	INIT_WORK(&nvt->start_work, nvt_start_work);
	INIT_WORK(&nvt->charger_work, nvt_charger_work);
	nvt->charger_state = nvt->charger_sent = -1;
	nvt->test_stop = nvt_test_stop_noop;
	init_waitqueue_head(&nvt->wait);
	kref_init(&nvt->ref);
	KUNIT_ASSERT_EQ(test, kfifo_alloc(&nvt->fifo, 64, GFP_KERNEL), 0);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, nvt_test_free_fifo, nvt), 0);
	nvt->panel_off = true;
	nvt->sequence = 41;
	nvt->error = -EIO;
	kfifo_in(&nvt->fifo, "old", 3);
	file.private_data = &nvt->misc;
	second.private_data = &nvt->misc;

	/* No SPI device: a disabled-panel open must not try firmware I/O. */
	KUNIT_ASSERT_EQ(test, nvt_open(&inode, &file), 0);
	KUNIT_EXPECT_TRUE(test, nvt->opened);
	KUNIT_EXPECT_FALSE(test, nvt->running);
	KUNIT_EXPECT_EQ(test, nvt->sequence, 41ULL);
	KUNIT_EXPECT_TRUE(test, kfifo_is_empty(&nvt->fifo));
	KUNIT_EXPECT_EQ(test, nvt_poll(&file, NULL), (__poll_t)0);
	KUNIT_EXPECT_FALSE(test, nvt_read_ready(nvt));
	KUNIT_EXPECT_EQ(test, nvt_open(&inode, &second), -EBUSY);
	KUNIT_EXPECT_EQ(test, nvt_panel_disabling(&nvt->follower), 0);
	KUNIT_EXPECT_EQ(test, nvt_panel_disabling(&nvt->follower), 0);

	/* Removal still wakes a waiting reader, and close drops its reference. */
	nvt->dead = true;
	KUNIT_EXPECT_EQ(test, nvt_poll(&file, NULL), (__poll_t)(EPOLLERR | EPOLLHUP));
	KUNIT_EXPECT_EQ(test, nvt_release(&inode, &file), 0);
	KUNIT_EXPECT_FALSE(test, nvt->opened);
	KUNIT_EXPECT_EQ(test, kref_read(&nvt->ref), 1U);
}

static void nvt_test_suspend_open_stream(struct kunit *test)
{
	struct nvt_thp *nvt = kunit_kzalloc(test, sizeof(*nvt), GFP_KERNEL);
	struct device dev = {};
	struct file file = {};

	KUNIT_ASSERT_NOT_NULL(test, nvt);
	mutex_init(&nvt->transition_lock);
	mutex_init(&nvt->lifecycle_lock);
	INIT_WORK(&nvt->start_work, nvt_start_work);
	INIT_WORK(&nvt->charger_work, nvt_charger_work);
	nvt->charger_state = nvt->charger_sent = -1;
	nvt->test_stop = nvt_test_stop_noop;
	init_waitqueue_head(&nvt->wait);
	dev_set_drvdata(&dev, nvt);
	file.private_data = nvt;
	nvt->opened = true;
	nvt->panel_off = true;
	KUNIT_ASSERT_EQ(test, kfifo_alloc(&nvt->fifo, 64, GFP_KERNEL), 0);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, nvt_test_free_fifo, nvt), 0);
	kfifo_in(&nvt->fifo, "partial", 7);

	/* No SPI device: screen-off sleep must keep the open stream and bytes. */
	KUNIT_ASSERT_EQ(test, nvt_suspend(&dev), 0);
	KUNIT_EXPECT_TRUE(test, nvt->suspended);
	KUNIT_EXPECT_TRUE(test, nvt->opened);
	KUNIT_EXPECT_EQ(test, kfifo_len(&nvt->fifo), 7U);
	KUNIT_ASSERT_EQ(test, nvt_resume(&dev), 0);
	KUNIT_EXPECT_FALSE(test, nvt->suspended);
	KUNIT_EXPECT_FALSE(test, nvt_read_ready(nvt));
	KUNIT_EXPECT_EQ(test, kfifo_len(&nvt->fifo), 7U);

	/* DRM may enable before the SPI resume. It must not perform SPI I/O. */
	KUNIT_ASSERT_EQ(test, nvt_suspend(&dev), 0);
	KUNIT_ASSERT_EQ(test, nvt_panel_enabled(&nvt->follower), 0);
	KUNIT_EXPECT_FALSE(test, nvt->panel_off);
	KUNIT_EXPECT_FALSE(test, nvt->running);
	KUNIT_EXPECT_EQ(test, nvt_poll(&file, NULL), (__poll_t)0);
	KUNIT_EXPECT_FALSE(test, nvt_read_ready(nvt));
	/* Closed stream resumes without restarting even with the panel enabled. */
	nvt->opened = false;
	KUNIT_ASSERT_EQ(test, nvt_resume(&dev), 0);
	KUNIT_EXPECT_FALSE(test, nvt->suspended);
}

struct nvt_async_test {
	struct nvt_thp nvt;
	struct spi_device spi;
	struct completion started, finish_start, quiescing, stopped;
	struct task_struct *stop_task;
	unsigned int starts, stops;
	int start_error;
	bool remove, misc_registered;
};

static int nvt_test_start_blocked(struct nvt_thp *nvt)
{
	struct nvt_async_test *ctx = container_of(nvt, struct nvt_async_test, nvt);

	ctx->starts++;
	complete(&ctx->started);
	wait_for_completion(&ctx->finish_start);
	WRITE_ONCE(nvt->running, !ctx->start_error);
	if (!ctx->start_error)
		set_bit(NVT_IRQ_ENABLED, &nvt->flags);
	return ctx->start_error;
}

static void nvt_test_stop_count(struct nvt_thp *nvt)
{
	struct nvt_async_test *ctx = container_of(nvt, struct nvt_async_test, nvt);

	ctx->stops++;
	clear_bit(NVT_IRQ_ENABLED, &nvt->flags);
}

static void nvt_test_async_cleanup(void *data)
{
	struct nvt_async_test *ctx = data;

	complete_all(&ctx->finish_start);
	if (ctx->stop_task)
		kthread_stop_put(ctx->stop_task);
	if (ctx->misc_registered)
		misc_deregister(&ctx->nvt.misc);
	cancel_work_sync(&ctx->nvt.start_work);
	cancel_work_sync(&ctx->nvt.charger_work);
}

static struct nvt_async_test *nvt_test_async_init(struct kunit *test)
{
	struct nvt_async_test *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	struct nvt_thp *nvt;

	if (!ctx)
		return NULL;
	nvt = &ctx->nvt;
	mutex_init(&nvt->transition_lock);
	mutex_init(&nvt->lifecycle_lock);
	INIT_WORK(&nvt->start_work, nvt_start_work);
	INIT_WORK(&nvt->charger_work, nvt_charger_work);
	nvt->charger_state = nvt->charger_sent = -1;
	init_waitqueue_head(&nvt->wait);
	init_completion(&ctx->started);
	init_completion(&ctx->finish_start);
	init_completion(&ctx->quiescing);
	init_completion(&ctx->stopped);
	nvt->spi = &ctx->spi;
	ctx->spi.dev.init_name = "nvt-async-test";
	spi_set_drvdata(&ctx->spi, nvt);
	nvt->test_start = nvt_test_start_blocked;
	nvt->test_stop = nvt_test_stop_count;
	nvt->test_quiescing = &ctx->quiescing;
	nvt->opened = true;
	nvt->panel_off = true;
	if (kunit_add_action_or_reset(test, nvt_test_async_cleanup, ctx))
		return NULL;
	return ctx;
}

static int nvt_test_disable_thread(void *data)
{
	struct nvt_async_test *ctx = data;

	if (ctx->remove) {
		nvt_remove(&ctx->spi);
		ctx->misc_registered = false;
	} else
		nvt_panel_disabling(&ctx->nvt.follower);
	complete(&ctx->stopped);
	return 0;
}

static void nvt_test_enable_async_coalesces(struct kunit *test)
{
	struct nvt_async_test *ctx = nvt_test_async_init(test);
	struct nvt_thp *nvt;

	KUNIT_ASSERT_NOT_NULL(test, ctx);
	nvt = &ctx->nvt;
	/* Worker cannot enter start until both callbacks have returned. */
	mutex_lock(&nvt->lifecycle_lock);
	KUNIT_EXPECT_EQ(test, nvt_panel_enabled(&nvt->follower), 0);
	KUNIT_EXPECT_EQ(test, nvt_panel_enabled(&nvt->follower), 0);
	KUNIT_EXPECT_FALSE(test, completion_done(&ctx->started));
	mutex_unlock(&nvt->lifecycle_lock);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&ctx->started, HZ), 0UL);
	/* A running attempt is coalesced too, without waiting for its start. */
	KUNIT_EXPECT_EQ(test, nvt_panel_enabled(&nvt->follower), 0);
	KUNIT_EXPECT_FALSE(test, completion_done(&ctx->finish_start));
	complete(&ctx->finish_start);
	flush_work(&nvt->start_work);
	KUNIT_EXPECT_EQ(test, ctx->starts, 1U);
	KUNIT_EXPECT_TRUE(test, nvt->running);
	nvt_panel_disabling(&nvt->follower);
	KUNIT_EXPECT_FALSE(test, nvt->running);
	KUNIT_EXPECT_FALSE(test, test_bit(NVT_IRQ_ENABLED, &nvt->flags));
}

static void nvt_test_stop_async(struct kunit *test, bool running, bool remove)
{
	struct nvt_async_test *ctx = nvt_test_async_init(test);
	struct nvt_thp *nvt;
	unsigned long reached;

	KUNIT_ASSERT_NOT_NULL(test, ctx);
	nvt = &ctx->nvt;
	ctx->remove = remove;
	if (remove) {
		nvt->misc.minor = MISC_DYNAMIC_MINOR;
		nvt->misc.name = "nvt-kunit-remove";
		nvt->misc.fops = &nvt_fops;
		KUNIT_ASSERT_EQ(test, misc_register(&nvt->misc), 0);
		ctx->misc_registered = true;
	}
	if (!running)
		mutex_lock(&nvt->lifecycle_lock);
	nvt_panel_enabled(&nvt->follower);
	nvt_charger_notify(&nvt->charger_notifier, PSY_EVENT_PROP_CHANGED, NULL);
	if (running)
		KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&ctx->started, HZ), 0UL);
	ctx->stop_task = kthread_create(nvt_test_disable_thread, ctx, "nvt-kunit-stop");
	if (IS_ERR(ctx->stop_task)) {
		ctx->stop_task = NULL;
		if (!running)
			mutex_unlock(&nvt->lifecycle_lock);
		KUNIT_FAIL(test, "could not start stop thread");
		return;
	}
	get_task_struct(ctx->stop_task);
	wake_up_process(ctx->stop_task);
	reached = wait_for_completion_timeout(&ctx->quiescing, HZ);
	KUNIT_EXPECT_NE(test, reached, 0UL);
	KUNIT_EXPECT_FALSE(test, completion_done(&ctx->stopped));
	if (!running)
		mutex_unlock(&nvt->lifecycle_lock);
	complete(&ctx->finish_start);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&ctx->stopped, HZ), 0UL);
	KUNIT_EXPECT_EQ(test, ctx->starts, running ? 1U : 0U);
	KUNIT_EXPECT_EQ(test, ctx->stops, 1U);
	KUNIT_EXPECT_FALSE(test, nvt->running);
	KUNIT_EXPECT_FALSE(test, test_bit(NVT_IRQ_ENABLED, &nvt->flags));
	KUNIT_EXPECT_FALSE(test, work_busy(&nvt->start_work));
	KUNIT_EXPECT_FALSE(test, work_busy(&nvt->charger_work));
	if (remove) {
		KUNIT_EXPECT_TRUE(test, nvt->dead);
		nvt_panel_enabled(&nvt->follower);
		KUNIT_EXPECT_FALSE(test, work_busy(&nvt->start_work));
	}
}

static void nvt_test_disable_pending(struct kunit *test)
{
	nvt_test_stop_async(test, false, false);
}

static void nvt_test_disable_running(struct kunit *test)
{
	nvt_test_stop_async(test, true, false);
}

static void nvt_test_remove_pending(struct kunit *test)
{
	nvt_test_stop_async(test, false, true);
}

static void nvt_test_async_error(struct kunit *test)
{
	struct nvt_async_test *ctx = nvt_test_async_init(test);
	struct file file;

	KUNIT_ASSERT_NOT_NULL(test, ctx);
	ctx->start_error = -EIO;
	file.private_data = &ctx->nvt;
	complete_all(&ctx->finish_start);
	nvt_panel_enabled(&ctx->nvt.follower);
	flush_work(&ctx->nvt.start_work);
	KUNIT_EXPECT_EQ(test, ctx->nvt.error, -EIO);
	KUNIT_EXPECT_TRUE(test, nvt_read_ready(&ctx->nvt));
	KUNIT_EXPECT_EQ(test, nvt_poll(&file, NULL), (__poll_t)(EPOLLERR | EPOLLHUP));
	/* Duplicate enable must neither hide the error nor retry the download. */
	nvt_panel_enabled(&ctx->nvt.follower);
	flush_work(&ctx->nvt.start_work);
	KUNIT_EXPECT_EQ(test, ctx->starts, 1U);
	KUNIT_EXPECT_EQ(test, ctx->nvt.error, -EIO);

	/* Resume must not expose the previous error before the worker runs. */
	KUNIT_ASSERT_EQ(test, nvt_suspend(&ctx->spi.dev), 0);
	mutex_lock(&ctx->nvt.lifecycle_lock);
	KUNIT_EXPECT_EQ(test, nvt_resume(&ctx->spi.dev), 0);
	KUNIT_EXPECT_EQ(test, ctx->nvt.error, 0);
	mutex_unlock(&ctx->nvt.lifecycle_lock);
	flush_work(&ctx->nvt.start_work);
	KUNIT_EXPECT_EQ(test, ctx->starts, 2U);
	KUNIT_EXPECT_EQ(test, ctx->nvt.error, -EIO);
}

struct nvt_charger_test {
	struct nvt_thp nvt;
	struct spi_device spi;
	unsigned int writes;
	u8 command;
	bool fail;
};

static int nvt_test_charger_write(void *context, const void *data, size_t count)
{
	struct nvt_charger_test *ctx = context;
	const u8 *bytes = data;

	if (count != sizeof(u32) + 2 || get_unaligned((const u32 *)data) != 0x11c450 ||
	    (bytes[4] != 0x51 && bytes[4] != 0x53) || bytes[5])
		return -EINVAL;
	ctx->writes++;
	ctx->command = bytes[4];
	return ctx->fail ? -EREMOTEIO : 0;
}

static int nvt_test_charger_read(void *context, const void *reg, size_t reg_size,
				 void *value, size_t size)
{
	return -EOPNOTSUPP;
}

static void nvt_test_charger_cleanup(void *data)
{
	struct nvt_charger_test *ctx = data;

	cancel_work_sync(&ctx->nvt.charger_work);
	regmap_exit(ctx->nvt.map);
}

static struct nvt_charger_test *nvt_test_charger_init(struct kunit *test)
{
	struct nvt_charger_test *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	const struct regmap_config config = {
		.reg_bits = 32,
		.val_bits = 8,
		.write = nvt_test_charger_write,
		.read = nvt_test_charger_read,
		.max_register = 0x7fffff,
		.max_raw_write = 2,
		.cache_type = REGCACHE_NONE,
	};

	KUNIT_ASSERT_NOT_NULL(test, ctx);
	ctx->nvt.spi = &ctx->spi;
	ctx->spi.dev.init_name = "nvt-charger-test";
	mutex_init(&ctx->nvt.lifecycle_lock);
	init_waitqueue_head(&ctx->nvt.wait);
	INIT_WORK(&ctx->nvt.charger_work, nvt_charger_work);
	ctx->nvt.test_stop = nvt_test_stop_noop;
	ctx->nvt.opened = ctx->nvt.running = true;
	ctx->nvt.charger_state = ctx->nvt.charger_sent = -1;
	ctx->nvt.map = regmap_init(NULL, NULL, ctx, &config);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->nvt.map);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, nvt_test_charger_cleanup, ctx), 0);
	return ctx;
}

static void nvt_test_charger_changes(struct kunit *test)
{
	struct nvt_charger_test *ctx = nvt_test_charger_init(test);
	struct nvt_thp *nvt = &ctx->nvt;

	mutex_lock(&nvt->lifecycle_lock);
	nvt_charger_state(nvt, 0);
	KUNIT_EXPECT_EQ(test, ctx->command, (u8)0x51);
	nvt_charger_state(nvt, 0);
	KUNIT_EXPECT_EQ(test, ctx->writes, 1U);
	nvt_charger_state(nvt, 1);
	KUNIT_EXPECT_EQ(test, ctx->command, (u8)0x53);
	nvt_charger_state(nvt, 1);
	nvt_charger_state(nvt, -ENODEV);
	KUNIT_EXPECT_EQ(test, ctx->writes, 2U);
	nvt_charger_state(nvt, 0);
	KUNIT_EXPECT_EQ(test, ctx->command, (u8)0x51);
	KUNIT_EXPECT_EQ(test, ctx->writes, 3U);
	mutex_unlock(&nvt->lifecycle_lock);
}

static void nvt_test_charger_lifecycle_gates(struct kunit *test)
{
	struct nvt_charger_test *ctx = nvt_test_charger_init(test);
	struct nvt_thp *nvt = &ctx->nvt;

	mutex_lock(&nvt->lifecycle_lock);
	nvt_charger_state(nvt, -ENODEV);
	nvt->opened = false;
	nvt_charger_state(nvt, 1);
	nvt->opened = true;
	nvt->running = false;
	nvt_charger_state(nvt, 0);
	nvt->running = true;
	nvt->panel_off = true;
	nvt_charger_state(nvt, 1);
	nvt->panel_off = false;
	nvt->suspended = true;
	nvt_charger_state(nvt, 0);
	nvt->suspended = false;
	nvt->dead = true;
	nvt_charger_state(nvt, 1);
	KUNIT_EXPECT_EQ(test, ctx->writes, 0U);
	KUNIT_EXPECT_EQ(test, nvt->charger_sent, -1);
	KUNIT_EXPECT_EQ(test, nvt->charger_state, 1);
	nvt->dead = false;
	nvt_charger_state(nvt, 0);
	KUNIT_EXPECT_EQ(test, ctx->writes, 1U);
	KUNIT_EXPECT_EQ(test, ctx->command, (u8)0x51);
	mutex_unlock(&nvt->lifecycle_lock);
}

static void nvt_test_charger_restore_and_error(struct kunit *test)
{
	struct nvt_charger_test *ctx = nvt_test_charger_init(test);
	struct nvt_thp *nvt = &ctx->nvt;

	mutex_lock(&nvt->lifecycle_lock);
	nvt_charger_state(nvt, 1);
	nvt_stop_irq(nvt);
	nvt_charger_state(nvt, 1);
	KUNIT_EXPECT_EQ(test, ctx->writes, 1U);
	/* A recovered firmware epoch resends the cached ONLINE state once. */
	nvt->running = true;
	nvt_charger_state(nvt, -ENODEV);
	nvt_charger_state(nvt, 1);
	KUNIT_EXPECT_EQ(test, ctx->writes, 2U);
	ctx->fail = true;
	nvt_charger_state(nvt, 0);
	KUNIT_EXPECT_EQ(test, nvt->error, -EREMOTEIO);
	KUNIT_EXPECT_FALSE(test, nvt->running);
	KUNIT_EXPECT_EQ(test, nvt->charger_sent, -1);
	nvt_charger_state(nvt, 0);
	KUNIT_EXPECT_EQ(test, ctx->writes, 3U); /* no blind command retry */
	ctx->fail = false;
	nvt->running = true;
	nvt->error = 0;
	nvt_charger_state(nvt, 0);
	KUNIT_EXPECT_EQ(test, ctx->writes, 4U);
	KUNIT_EXPECT_EQ(test, nvt->charger_sent, 0);
	mutex_unlock(&nvt->lifecycle_lock);
}

struct nvt_supply_test {
	struct power_supply psy;
	struct power_supply_desc desc;
	int scope, online, error;
};

static int nvt_test_supply_property(struct power_supply *psy,
				    enum power_supply_property prop,
				    union power_supply_propval *value)
{
	struct nvt_supply_test *ctx = container_of(psy, struct nvt_supply_test, psy);

	if (prop == POWER_SUPPLY_PROP_SCOPE) {
		value->intval = ctx->scope;
		return 0;
	}
	value->intval = ctx->online;
	return ctx->error;
}

static void nvt_test_charger_external_online(struct kunit *test)
{
	static const enum power_supply_property properties[] = {
		POWER_SUPPLY_PROP_SCOPE, POWER_SUPPLY_PROP_ONLINE,
	};
	struct nvt_supply_test *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	int online = -ENODEV;

	KUNIT_ASSERT_NOT_NULL(test, ctx);
	ctx->desc.type = POWER_SUPPLY_TYPE_USB;
	ctx->desc.properties = properties;
	ctx->desc.num_properties = ARRAY_SIZE(properties);
	ctx->desc.get_property = nvt_test_supply_property;
	ctx->psy.desc = &ctx->desc;
	atomic_set(&ctx->psy.use_cnt, 1);
	init_rwsem(&ctx->psy.extensions_sem);
	INIT_LIST_HEAD(&ctx->psy.extensions);
	ctx->scope = POWER_SUPPLY_SCOPE_SYSTEM;
	ctx->online = 1;
	KUNIT_EXPECT_EQ(test, nvt_external_supply(&ctx->psy, &online), 1);
	KUNIT_EXPECT_EQ(test, online, 1);
	ctx->online = 0;
	KUNIT_EXPECT_EQ(test, nvt_external_supply(&ctx->psy, &online), 0);
	KUNIT_EXPECT_EQ(test, online, 0);
	ctx->online = 1;
	ctx->desc.type = POWER_SUPPLY_TYPE_BATTERY;
	KUNIT_EXPECT_EQ(test, nvt_external_supply(&ctx->psy, &online), 0);
	KUNIT_EXPECT_EQ(test, online, 0);
	ctx->desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	KUNIT_EXPECT_EQ(test, nvt_external_supply(&ctx->psy, &online), 0);
	KUNIT_EXPECT_EQ(test, online, 0);
	ctx->desc.type = POWER_SUPPLY_TYPE_USB;
	ctx->scope = POWER_SUPPLY_SCOPE_DEVICE;
	KUNIT_EXPECT_EQ(test, nvt_external_supply(&ctx->psy, &online), 0);
	KUNIT_EXPECT_EQ(test, online, 0);
	ctx->scope = POWER_SUPPLY_SCOPE_SYSTEM;
	ctx->error = -EIO;
	online = -ENODEV;
	KUNIT_EXPECT_EQ(test, nvt_external_supply(&ctx->psy, &online), 0);
	KUNIT_EXPECT_EQ(test, online, -ENODEV);
}

static const char *const nvt_test_firmware[] = {
	"novatek/piano/novatek_nt36532_piano_fw_csot.bin",
	"novatek/piano/novatek_nt36532_piano_fw_boe.bin",
};

/* A device whose firmware-name lists the first @count test names. */
static struct device *nvt_test_names_device(struct kunit *test, const char *name,
					    unsigned int count)
{
	const struct property_entry props[] = {
		PROPERTY_ENTRY_STRING_ARRAY_LEN("firmware-name", nvt_test_firmware, count),
		{ }
	};
	struct device *dev = kunit_device_register(test, name);

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	KUNIT_ASSERT_EQ(test, device_create_managed_software_node(dev, count ? props : &props[1],
								  NULL), 0);
	return dev;
}

static void nvt_test_select_by_lcd_id(struct kunit *test)
{
	struct device *dev = nvt_test_names_device(test, "nvt-select-both", 2);
	const char *name = NULL;

	KUNIT_EXPECT_EQ(test, nvt_select_firmware(dev, 0, &name), 0);
	KUNIT_EXPECT_STREQ(test, name, nvt_test_firmware[0]);
	KUNIT_EXPECT_EQ(test, nvt_select_firmware(dev, 1, &name), 0);
	KUNIT_EXPECT_STREQ(test, name, nvt_test_firmware[1]);
}

static void nvt_test_select_without_lcd_id(struct kunit *test)
{
	struct device *single = nvt_test_names_device(test, "nvt-select-single", 1);
	struct device *both = nvt_test_names_device(test, "nvt-select-ambiguous", 2);
	const char *name = NULL;

	/* The previous single-name binding keeps working unchanged. */
	KUNIT_EXPECT_EQ(test, nvt_select_firmware(single, -1, &name), 0);
	KUNIT_EXPECT_STREQ(test, name, nvt_test_firmware[0]);
	name = NULL;
	KUNIT_EXPECT_EQ(test, nvt_select_firmware(both, -1, &name), -EINVAL);
	KUNIT_EXPECT_NULL(test, name);
}

static void nvt_test_select_errors(struct kunit *test)
{
	struct device *single = nvt_test_names_device(test, "nvt-select-short", 1);
	struct device *none = nvt_test_names_device(test, "nvt-select-none", 0);
	const char *name = NULL;

	/* A BOE module must not fall back to the CSOT image. */
	KUNIT_EXPECT_EQ(test, nvt_select_firmware(single, 1, &name), -EINVAL);
	KUNIT_EXPECT_NULL(test, name);
	KUNIT_EXPECT_EQ(test, nvt_select_firmware(none, -1, &name), -EINVAL);
	KUNIT_EXPECT_EQ(test, nvt_select_firmware(none, 0, &name), -EINVAL);
	KUNIT_EXPECT_NULL(test, name);
}

static void nvt_test_panel_vendor(struct kunit *test)
{
	struct device *dev = kunit_device_register(test, "nvt-panel-vendor");
	struct nvt_thp *nvt = kunit_kzalloc(test, sizeof(*nvt), GFP_KERNEL);
	char *buf = kunit_kzalloc(test, PAGE_SIZE, GFP_KERNEL);

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	KUNIT_ASSERT_NOT_NULL(test, nvt);
	KUNIT_ASSERT_NOT_NULL(test, buf);
	dev_set_drvdata(dev, nvt);
	nvt->lcd_id = 0;
	KUNIT_EXPECT_EQ(test, panel_vendor_show(dev, &dev_attr_panel_vendor, buf), 5);
	KUNIT_EXPECT_STREQ(test, buf, "csot\n");
	nvt->lcd_id = 1;
	KUNIT_EXPECT_EQ(test, panel_vendor_show(dev, &dev_attr_panel_vendor, buf), 4);
	KUNIT_EXPECT_STREQ(test, buf, "boe\n");
}

static struct kunit_case nvt_select_cases[] = {
	KUNIT_CASE(nvt_test_select_by_lcd_id),
	KUNIT_CASE(nvt_test_select_without_lcd_id),
	KUNIT_CASE(nvt_test_select_errors),
	KUNIT_CASE(nvt_test_panel_vendor),
	{}
};

static struct kunit_suite nvt_select_suite = {
	.name = "nt36532e-select",
	.test_cases = nvt_select_cases,
};

static struct kunit_case nvt_panel_cases[] = {
	KUNIT_CASE(nvt_test_charger_changes),
	KUNIT_CASE(nvt_test_charger_lifecycle_gates),
	KUNIT_CASE(nvt_test_charger_restore_and_error),
	KUNIT_CASE(nvt_test_charger_external_online),
	KUNIT_CASE(nvt_test_open_disabled_panel),
	KUNIT_CASE(nvt_test_suspend_open_stream),
	KUNIT_CASE(nvt_test_enable_async_coalesces),
	KUNIT_CASE(nvt_test_disable_pending),
	KUNIT_CASE(nvt_test_disable_running),
	KUNIT_CASE(nvt_test_remove_pending),
	KUNIT_CASE(nvt_test_async_error),
	{}
};

static struct kunit_suite nvt_panel_suite = {
	.name = "nt36532e-panel",
	.test_cases = nvt_panel_cases,
};

kunit_test_suites(&nvt_panel_suite, &nvt_select_suite);
#endif

MODULE_DESCRIPTION("NT36532E cascade SPI host-touch transport");
MODULE_LICENSE("GPL");
