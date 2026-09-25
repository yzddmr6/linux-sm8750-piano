// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2010 - 2022 Novatek, Inc.
 * SPI paging/host-download protocol from the piano vendor driver.
 * NTP1 framing follows ianchb's NT36532E THP stream. See README.rst.
 */
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

#define NVT_WRITE_SIZE	(63 * 1024)
#define NVT_READ_SIZE	8192
#define NVT_FRAME_PREFIX	257
#define NVT_STREAM_MAGIC	0x3150544e
#define NVT_FRAME_VALID	BIT(0)
#define NVT_FRAME_EPOCH	BIT(1)
#define NVT_IRQ_ENABLED	0

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
	int error;
	bool dead, opened, running, epoch, suspended;
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
	if (ret) {
		WRITE_ONCE(nvt->error, ret);
		WRITE_ONCE(nvt->running, false);
		if (test_and_clear_bit(NVT_IRQ_ENABLED, &nvt->flags))
			disable_irq_nosync(irq);
		dev_err_ratelimited(&nvt->spi->dev, "THP SPI read failed: %d\n", ret);
		wake_up_interruptible(&nvt->wait);
		return IRQ_HANDLED;
	}
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
			nvt->epoch = false;
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

static int nvt_open(struct inode *inode, struct file *file)
{
	struct nvt_thp *nvt = container_of(file->private_data, struct nvt_thp, misc);
	const struct firmware *firmware;
	struct nvt_thp_firmware image;
	u16 payload_length;
	int ret;

	if (file->f_mode & FMODE_WRITE)
		return -EPERM;
	ret = mutex_lock_interruptible(&nvt->lifecycle_lock);
	if (ret)
		return ret;
	if (nvt->dead || nvt->opened || nvt->suspended) {
		ret = nvt->dead ? -ENODEV : -EBUSY;
		goto unlock;
	}
	ret = request_firmware_direct(&firmware, nvt->firmware_name, &nvt->spi->dev);
	if (ret)
		goto unlock;
	ret = nvt_thp_parse_firmware(firmware->data, firmware->size, &image);
	if (!ret) {
		ret = nvt_thp_boot_firmware(nvt->map, firmware, &image, &payload_length);
		nvt_thp_free_firmware(&image);
	}
	release_firmware(firmware);
	if (ret)
		goto unlock;
	/* IRQ packets must fit one transaction; do not split a changing frame. */
	if (payload_length < 64 || payload_length > nvt->max_read - (NVT_FRAME_PREFIX - 1)) {
		ret = -EMSGSIZE;
		goto unlock;
	}
	ret = nvt_thp_apply_host_commands(nvt->map);
	if (ret) {
		dev_err(&nvt->spi->dev,
			"THP idle-baseline update/op mode 2 command failed: %d\n", ret);
		goto unlock;
	}
	dev_info(&nvt->spi->dev, "THP idle-baseline update and op mode 2 commands completed\n");
	nvt->frame_length = NVT_FRAME_PREFIX + payload_length;
	kfifo_reset(&nvt->fifo);
	nvt->sequence = 0;
	nvt->epoch = true;
	nvt->opened = true;
	WRITE_ONCE(nvt->error, 0);
	WRITE_ONCE(nvt->running, true);
	kref_get(&nvt->ref);
	file->private_data = nvt;
	stream_open(inode, file);
	set_bit(NVT_IRQ_ENABLED, &nvt->flags);
	enable_irq(nvt->spi->irq);
	dev_info(&nvt->spi->dev, "THP stream ready, payload %u bytes\n", payload_length);
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
	return READ_ONCE(nvt->dead) || READ_ONCE(nvt->error) || !kfifo_is_empty(&nvt->fifo);
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
	if (READ_ONCE(nvt->dead) || READ_ONCE(nvt->error))
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
	int ret;

	mutex_lock(&nvt->lifecycle_lock);
	ret = nvt->opened ? -EBUSY : 0;
	if (!ret)
		nvt->suspended = true;
	mutex_unlock(&nvt->lifecycle_lock);
	return ret;
}

static int nvt_resume(struct device *dev)
{
	struct nvt_thp *nvt = dev_get_drvdata(dev);

	mutex_lock(&nvt->lifecycle_lock);
	nvt->suspended = false;
	mutex_unlock(&nvt->lifecycle_lock);
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

MODULE_DESCRIPTION("NT36532E cascade SPI host-touch transport");
MODULE_LICENSE("GPL");
