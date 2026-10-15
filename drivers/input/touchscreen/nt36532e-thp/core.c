// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2010 - 2022 Novatek, Inc.
 * SPI paging/host-download protocol from the piano vendor driver.
 * NTP1 framing follows ianchb's NT36532E THP stream. See README.rst.
 */
#include <drm/drm_panel.h>
#include <linux/firmware.h>
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
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/uaccess.h>
#include "firmware.h"

#if IS_ENABLED(CONFIG_TOUCHSCREEN_NT36532E_THP_KUNIT_TEST)
#include <kunit/test.h>
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
	struct mutex lifecycle_lock; /* Serializes open, close, removal and PM. */
	struct mutex read_lock; /* Keeps the FIFO single-consumer across shared fds. */
	wait_queue_head_t wait;
	struct kfifo fifo;
	const char *firmware_name;
	u8 *tx, *rx, *record;
	size_t max_read;
	u16 frame_length;
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
	if (test_and_clear_bit(NVT_IRQ_ENABLED, &nvt->flags))
		disable_irq(nvt->spi->irq);
	else
		synchronize_irq(nvt->spi->irq);
}

/* Caller holds lifecycle_lock, with the IRQ stopped. */
static int nvt_start_stream(struct nvt_thp *nvt)
{
	const struct firmware *firmware;
	struct nvt_thp_command_status last;
	struct nvt_thp_firmware image;
	s64 first_frame_ms;
	u16 payload_length;
	ktime_t start;
	int ret;

	lockdep_assert_held(&nvt->lifecycle_lock);
	WRITE_ONCE(nvt->error, 0);
	nvt->read_errors = 0;
	ret = request_firmware_direct(&firmware, nvt->firmware_name, &nvt->spi->dev);
	if (ret)
		return ret;
	ret = nvt_thp_parse_firmware(firmware->data, firmware->size, &image);
	if (!ret) {
		ret = nvt_thp_boot_firmware(nvt->map, firmware, &image, &payload_length);
		nvt_thp_free_firmware(&image);
	}
	release_firmware(firmware);
	if (ret)
		return ret;
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
	 * Sent right after the boot reset state, the idle-baseline command did
	 * not clear within its budget on piano. The stock stack sends it only
	 * while frames flow, so wait for the first valid frame, which stays
	 * queued for the reader.
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
		dev_err(&nvt->spi->dev, "THP first valid frame not received: %d\n", ret);
		goto stop;
	}
	first_frame_ms = ktime_ms_delta(ktime_get(), start);
	ret = nvt_thp_apply_host_commands(nvt->map, &last);
	if (ret) {
		dev_err(&nvt->spi->dev,
			"THP host command BF %02x failed: %d, command byte %#04x\n",
			last.sub_command, ret, last.status);
		goto stop;
	}
	ret = READ_ONCE(nvt->error);
	if (ret)
		goto stop;
	dev_info(&nvt->spi->dev,
		 "THP first frame after %lld ms; stylus disabled, idle-baseline update and op mode 2 accepted\n",
		 first_frame_ms);
	dev_info(&nvt->spi->dev, "THP stream ready, payload %u bytes\n", payload_length);
	return 0;
stop:
	nvt_stop_irq(nvt);
	return ret;
}

static int nvt_panel_disabling(struct drm_panel_follower *follower)
{
	struct nvt_thp *nvt = container_of(follower, struct nvt_thp, follower);

	mutex_lock(&nvt->lifecycle_lock);
	if (!nvt->panel_off) {
		nvt_stop_irq(nvt);
		WRITE_ONCE(nvt->panel_off, true);
	}
	mutex_unlock(&nvt->lifecycle_lock);
	return 0;
}

static int nvt_panel_enabled(struct drm_panel_follower *follower)
{
	struct nvt_thp *nvt = container_of(follower, struct nvt_thp, follower);
	int ret = 0;

	mutex_lock(&nvt->lifecycle_lock);
	if (nvt->dead)
		goto unlock;
	WRITE_ONCE(nvt->error, 0);
	WRITE_ONCE(nvt->panel_off, false);
	/* Preserve queued bytes, including any partially read NTP1 record. */
	wake_up_interruptible(&nvt->wait);
	/* Panel and SPI resume callbacks may run in either order. */
	if (nvt->opened && !nvt->suspended && !nvt->running) {
		ret = nvt_start_stream(nvt);
		if (ret) {
			WRITE_ONCE(nvt->error, ret);
			dev_err(&nvt->spi->dev, "THP panel restart failed: %d\n", ret);
			wake_up_interruptible(&nvt->wait);
		}
	}
unlock:
	mutex_unlock(&nvt->lifecycle_lock);
	return ret;
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
	ret = mutex_lock_interruptible(&nvt->lifecycle_lock);
	if (ret)
		return ret;
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
			dev_err(&nvt->spi->dev, "THP stream start failed: %d\n", ret);
			goto unlock;
		}
	}
	nvt->opened = true;
	kref_get(&nvt->ref);
	file->private_data = nvt;
	stream_open(inode, file);
unlock:
	mutex_unlock(&nvt->lifecycle_lock);
	return ret;
}

static int nvt_release(struct inode *inode, struct file *file)
{
	struct nvt_thp *nvt = file->private_data;

	mutex_lock(&nvt->lifecycle_lock);
	if (!nvt->dead)
		nvt_stop_irq(nvt);
	nvt->opened = false;
	mutex_unlock(&nvt->lifecycle_lock);
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
	mutex_init(&nvt->lifecycle_lock);
	mutex_init(&nvt->read_lock);
	init_waitqueue_head(&nvt->wait);
	ret = device_property_read_string(&spi->dev, "firmware-name", &nvt->firmware_name);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "firmware-name is required\n");
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
	nvt->misc.minor = MISC_DYNAMIC_MINOR;
	nvt->misc.fops = &nvt_fops;
	nvt->misc.parent = &spi->dev;
	nvt->misc.mode = 0600;
	return misc_register(&nvt->misc);
}

static void nvt_remove(struct spi_device *spi)
{
	struct nvt_thp *nvt = spi_get_drvdata(spi);

	/* Do not hold lifecycle_lock while misc_open may be waiting for it. */
	misc_deregister(&nvt->misc);
	mutex_lock(&nvt->lifecycle_lock);
	WRITE_ONCE(nvt->dead, true);
	nvt_stop_irq(nvt);
	wake_up_interruptible(&nvt->wait);
	mutex_unlock(&nvt->lifecycle_lock);
}

static void nvt_shutdown(struct spi_device *spi)
{
	struct nvt_thp *nvt = spi_get_drvdata(spi);

	mutex_lock(&nvt->lifecycle_lock);
	WRITE_ONCE(nvt->dead, true);
	nvt_stop_irq(nvt);
	wake_up_interruptible(&nvt->wait);
	mutex_unlock(&nvt->lifecycle_lock);
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

	mutex_lock(&nvt->lifecycle_lock);
	/* An open userspace stream must survive system sleep.  Stop SPI before
	 * its controller suspends; the panel callback owns the shared reset.
	 * An already disabled panel has synchronously stopped the IRQ.
	 */
	if (!nvt->panel_off)
		nvt_stop_irq(nvt);
	WRITE_ONCE(nvt->suspended, true);
	mutex_unlock(&nvt->lifecycle_lock);
	return 0;
}

static int nvt_resume(struct device *dev)
{
	struct nvt_thp *nvt = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&nvt->lifecycle_lock);
	WRITE_ONCE(nvt->suspended, false);
	/* If DRM resumes later, panel_enabled will restart instead.  Keep the
	 * FIFO and the open fd, including any partially consumed NTP1 record.
	 */
	if (!nvt->dead && nvt->opened && !nvt->panel_off && !nvt->running) {
		ret = nvt_start_stream(nvt);
		if (ret) {
			WRITE_ONCE(nvt->error, ret);
			dev_err(dev, "THP resume restart failed: %d\n", ret);
		}
	}
	wake_up_interruptible(&nvt->wait);
	mutex_unlock(&nvt->lifecycle_lock);
	return ret;
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

static void nvt_test_open_disabled_panel(struct kunit *test)
{
	struct nvt_thp *nvt = kunit_kzalloc(test, sizeof(*nvt), GFP_KERNEL);
	struct file file = { .f_mode = FMODE_READ };
	struct file second = { .f_mode = FMODE_READ };
	struct inode inode = {};

	KUNIT_ASSERT_NOT_NULL(test, nvt);
	mutex_init(&nvt->lifecycle_lock);
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
	mutex_init(&nvt->lifecycle_lock);
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

static struct kunit_case nvt_panel_cases[] = {
	KUNIT_CASE(nvt_test_open_disabled_panel),
	KUNIT_CASE(nvt_test_suspend_open_stream),
	{}
};

static struct kunit_suite nvt_panel_suite = {
	.name = "nt36532e-panel",
	.test_cases = nvt_panel_cases,
};

kunit_test_suite(nvt_panel_suite);
#endif

MODULE_DESCRIPTION("NT36532E cascade SPI host-touch transport");
MODULE_LICENSE("GPL");
