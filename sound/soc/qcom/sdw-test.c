// SPDX-License-Identifier: GPL-2.0-only
/* Real helper/allocator regression with synthetic DAIs; no audio hardware. */
#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <trace/events/kmem.h>
#include <dt-bindings/sound/qcom,q6afe.h>

#include "sdw.h"

struct sdw_test_ctx {
	struct snd_pcm_substream substream;
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai_link link;
	struct snd_soc_dai cpu, macro, transport;
	struct snd_soc_dai_driver macro_driver, transport_driver;
	struct snd_soc_dai *dais[3];
	struct sdw_stream_runtime *stream;
	struct task_struct *owner;
	const void *allocation;
	bool tracking;
	unsigned int allocations, releases, set_calls;
	int set_error, map_error;
};

static void sdw_test_alloc(void *data, unsigned long caller, const void *ptr,
			   size_t requested, size_t allocated, gfp_t flags, int node)
{
	struct sdw_test_ctx *ctx = data;

	if (ctx->tracking && current == ctx->owner && ptr &&
	    requested == sizeof(struct sdw_stream_runtime)) {
		ctx->allocation = ptr;
		ctx->allocations++;
	}
}

static void sdw_test_free(void *data, unsigned long caller, const void *ptr)
{
	struct sdw_test_ctx *ctx = data;

	if (ctx->tracking && current == ctx->owner && ptr && ptr == ctx->allocation) {
		ctx->allocation = NULL;
		ctx->releases++;
	}
}

static int sdw_test_set_stream(struct snd_soc_dai *dai, void *stream, int direction)
{
	struct sdw_test_ctx *ctx = dev_get_drvdata(dai->dev);

	ctx->set_calls++;
	if (ctx->set_error)
		return ctx->set_error;
	ctx->stream = stream;
	return 0;
}

static void *sdw_test_get_stream(struct snd_soc_dai *dai, int direction)
{
	struct sdw_test_ctx *ctx = dev_get_drvdata(dai->dev);

	return ctx->stream;
}

static int sdw_test_get_map(const struct snd_soc_dai *dai, unsigned int *tx_num,
			    unsigned int *tx_slot, unsigned int *rx_num,
			    unsigned int *rx_slot)
{
	struct sdw_test_ctx *ctx = dev_get_drvdata(dai->dev);

	*tx_num = 0;
	*rx_num = 0;
	return ctx->map_error;
}

static const struct snd_soc_dai_ops sdw_test_macro_ops;
static const struct snd_soc_dai_ops sdw_test_transport_ops = {
	.set_stream = sdw_test_set_stream,
	.get_stream = sdw_test_get_stream,
	.get_channel_map = sdw_test_get_map,
};

static int sdw_test_init(struct kunit *test)
{
	struct sdw_test_ctx *ctx;
	struct device *dev;
	int ret;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	test->priv = ctx;
	ctx->rtd = kunit_kzalloc(test, sizeof(*ctx->rtd), GFP_KERNEL);
	if (!ctx->rtd)
		return -ENOMEM;
	dev = kunit_device_register(test, "qcom-sdw-test");
	if (IS_ERR(dev))
		return PTR_ERR(dev);
	dev_set_drvdata(dev, ctx);
	ctx->macro_driver.ops = &sdw_test_macro_ops;
	ctx->transport_driver.ops = &sdw_test_transport_ops;
	ctx->cpu = (struct snd_soc_dai) { .id = TX_CODEC_DMA_TX_3, .name = "TX3" };
	ctx->macro = (struct snd_soc_dai) {
		.dev = dev, .name = "tx-macro", .driver = &ctx->macro_driver,
	};
	ctx->transport = (struct snd_soc_dai) {
		.dev = dev, .name = "synthetic-sdw", .driver = &ctx->transport_driver,
	};
	ctx->dais[0] = &ctx->cpu;
	ctx->dais[1] = &ctx->macro;
	ctx->dais[2] = &ctx->transport;
	ctx->link.num_cpus = 1;
	ctx->link.num_codecs = 1;
	ctx->rtd->dev = dev;
	ctx->rtd->dai_link = &ctx->link;
	ctx->rtd->dais = ctx->dais;
	ctx->substream.private_data = ctx->rtd;
	ctx->substream.stream = SNDRV_PCM_STREAM_CAPTURE;
	ctx->owner = current;
	ret = register_trace_kmalloc(sdw_test_alloc, ctx);
	if (ret)
		return ret;
	ret = register_trace_kfree(sdw_test_free, ctx);
	if (ret) {
		unregister_trace_kmalloc(sdw_test_alloc, ctx);
		tracepoint_synchronize_unregister();
	}
	return ret;
}

static void sdw_test_exit(struct kunit *test)
{
	struct sdw_test_ctx *ctx = test->priv;

	ctx->tracking = false;
	unregister_trace_kfree(sdw_test_free, ctx);
	unregister_trace_kmalloc(sdw_test_alloc, ctx);
	tracepoint_synchronize_unregister();
	/* Also clean up a leaked allocation when running against the unfixed code. */
	kfree(ctx->allocation);
}

static void sdw_test_direct_dmic(struct kunit *test)
{
	struct sdw_test_ctx *ctx = test->priv;
	bool prepared = false;
	int ret, prepare_ret, free_ret;

	ctx->tracking = true;
	ret = qcom_snd_sdw_startup(&ctx->substream);
	prepare_ret = qcom_snd_sdw_prepare(&ctx->substream, &prepared);
	free_ret = qcom_snd_sdw_hw_free(&ctx->substream, &prepared);
	qcom_snd_sdw_shutdown(&ctx->substream);
	ctx->tracking = false;
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, prepare_ret, 0);
	KUNIT_EXPECT_EQ(test, free_ret, 0);
	KUNIT_EXPECT_FALSE(test, prepared);
	KUNIT_EXPECT_EQ(test, ctx->allocations, 0);
	KUNIT_EXPECT_EQ(test, ctx->releases, 0);
}

static void sdw_test_mixed_link(struct kunit *test)
{
	struct sdw_test_ctx *ctx = test->priv;
	int ret;

	ctx->link.num_codecs = 2;
	ctx->tracking = true;
	ret = qcom_snd_sdw_startup(&ctx->substream);
	ctx->tracking = false;
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, ctx->set_calls, 1);
	KUNIT_ASSERT_NOT_NULL(test, ctx->stream);
	KUNIT_EXPECT_PTR_EQ(test, qcom_snd_sdw_get_stream(&ctx->substream), ctx->stream);
	KUNIT_EXPECT_EQ(test, ctx->stream->state, SDW_STREAM_ALLOCATED);
	ctx->tracking = true;
	qcom_snd_sdw_shutdown(&ctx->substream);
	ctx->tracking = false;
	KUNIT_EXPECT_EQ(test, ctx->allocations, 1);
	KUNIT_EXPECT_EQ(test, ctx->releases, 1);
}

static void sdw_test_rejected_stream(struct kunit *test)
{
	struct sdw_test_ctx *ctx = test->priv;
	int ret;

	ctx->link.num_codecs = 2;
	ctx->set_error = -ENOTSUPP;
	ctx->tracking = true;
	ret = qcom_snd_sdw_startup(&ctx->substream);
	qcom_snd_sdw_shutdown(&ctx->substream);
	ctx->tracking = false;
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, ctx->allocations, 1);
	KUNIT_EXPECT_EQ(test, ctx->releases, 1);
}

static void sdw_test_map_error(struct kunit *test)
{
	struct sdw_test_ctx *ctx = test->priv;
	int ret;

	ctx->link.num_codecs = 2;
	ctx->map_error = -EIO;
	ctx->tracking = true;
	ret = qcom_snd_sdw_startup(&ctx->substream);
	ctx->tracking = false;
	KUNIT_EXPECT_EQ(test, ret, -EIO);
	KUNIT_EXPECT_EQ(test, ctx->allocations, 1);
	KUNIT_EXPECT_EQ(test, ctx->releases, 1);
}

static void sdw_test_non_sdw_port(struct kunit *test)
{
	struct sdw_test_ctx *ctx = test->priv;
	int ret;

	ctx->cpu.id = VA_CODEC_DMA_TX_0;
	ctx->link.num_codecs = 2;
	ctx->tracking = true;
	ret = qcom_snd_sdw_startup(&ctx->substream);
	qcom_snd_sdw_shutdown(&ctx->substream);
	ctx->tracking = false;
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, ctx->set_calls, 0);
	KUNIT_EXPECT_EQ(test, ctx->allocations, 0);
}

static struct kunit_case sdw_test_cases[] = {
	KUNIT_CASE(sdw_test_direct_dmic),
	KUNIT_CASE(sdw_test_mixed_link),
	KUNIT_CASE(sdw_test_rejected_stream),
	KUNIT_CASE(sdw_test_map_error),
	KUNIT_CASE(sdw_test_non_sdw_port),
	{}
};

static struct kunit_suite sdw_test_suite = {
	.name = "qcom-sdw-lifecycle",
	.init = sdw_test_init,
	.exit = sdw_test_exit,
	.test_cases = sdw_test_cases,
};
kunit_test_suite(sdw_test_suite);

MODULE_LICENSE("GPL");
