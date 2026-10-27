// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the Xiaomi Pad 8 Pro panel modules: the profile selected
 * by the panel ID pin and the DSI commands each profile sends. Included by
 * panel-novatek-nt36532.c to reach its static profiles.
 */

#include <kunit/test.h>
#include <linux/hex.h>

#define NT36532_TEST_MSGS	128

struct nt36532_test_msg {
	u8 type;
	u16 flags;
	size_t len;
	u8 data[sizeof(struct drm_dsc_picture_parameter_set)];
};

/* A DSI host recording every message the panel sends. */
struct nt36532_test {
	struct mipi_dsi_host host;
	struct mipi_dsi_device dsi[2];
	struct nt36532 ctx;
	struct nt36532_test_msg msg[NT36532_TEST_MSGS];
	unsigned int count;
};

static ssize_t nt36532_test_transfer(struct mipi_dsi_host *host,
				     const struct mipi_dsi_msg *msg)
{
	struct nt36532_test *t = container_of(host, struct nt36532_test, host);
	struct nt36532_test_msg *m;

	if (t->count == ARRAY_SIZE(t->msg) || msg->tx_len > sizeof(m->data))
		return -ENOSPC;

	m = &t->msg[t->count++];
	m->type = msg->type;
	m->flags = msg->flags;
	m->len = msg->tx_len;
	memcpy(m->data, msg->tx_buf, msg->tx_len);

	return msg->tx_len;
}

static const struct mipi_dsi_host_ops nt36532_test_host_ops = {
	.transfer = nt36532_test_transfer,
};

/* Run the init sequence of @info for @mode with DSC set up like the MSM host. */
static struct nt36532_test *nt36532_test_init(struct kunit *test,
					      const struct panel_info *info,
					      const struct drm_display_mode *mode)
{
	struct nt36532_test *t = kunit_kzalloc(test, sizeof(*t), GFP_KERNEL);
	struct drm_dsc_config *dsc;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, t);
	t->host.ops = &nt36532_test_host_ops;
	for (i = 0; i < ARRAY_SIZE(t->dsi); i++) {
		t->dsi[i].host = &t->host;
		t->dsi[i].mode_flags = info->mode_flags;
		t->ctx.dsi[i] = &t->dsi[i];
	}
	t->ctx.panel_info = info;

	dsc = &t->ctx.dsc;
	*dsc = *info->dsc_cfg;
	dsc->pic_width = mode->hdisplay;
	dsc->pic_height = mode->vdisplay;
	dsc->convert_rgb = 1;
	drm_dsc_set_const_params(dsc);
	drm_dsc_set_rc_buf_thresh(dsc);
	KUNIT_ASSERT_EQ(test, drm_dsc_setup_rc_params(dsc, DRM_DSC_1_1_PRE_SCR), 0);
	dsc->initial_scale_value = drm_dsc_initial_scale_value(dsc);
	dsc->line_buf_depth = dsc->bits_per_component + 1;
	KUNIT_ASSERT_EQ(test, drm_dsc_compute_rc_parameters(dsc), 0);

	KUNIT_ASSERT_EQ(test, info->init_sequence(&t->ctx, mode), 0);

	return t;
}

static const struct {
	const char *name;
	const struct panel_info *info;
} nt36532_test_modules[] = {
	{ "42-02-0a", &piano_csot_panel_info },
	{ "35-02-0b", &piano_boe_panel_info },
};

/*
 * One line per DSI message, for comparing the streams with the stock
 * qcom,mdss-dsi-on-command of each timing outside the kernel.
 */
static void nt36532_test_log_sequences(struct kunit *test)
{
	char hex[2 * sizeof_field(struct nt36532_test_msg, data) + 1];
	unsigned int i, mode, n;

	for (i = 0; i < ARRAY_SIZE(nt36532_test_modules); i++) {
		const struct panel_info *info = nt36532_test_modules[i].info;

		for (mode = 0; mode < info->num_modes; mode++) {
			const struct drm_display_mode *m = &info->modes[mode];
			struct nt36532_test *t = nt36532_test_init(test, info, m);

			for (n = 0; n < t->count; n++) {
				*bin2hex(hex, t->msg[n].data, t->msg[n].len) = '\0';
				kunit_info(test, "nt36532-seq %s %d %u %02x %04x %s\n",
					   nt36532_test_modules[i].name,
					   drm_mode_vrefresh(m), n, t->msg[n].type,
					   t->msg[n].flags, hex);
			}
		}
	}
}

static void nt36532_test_select(struct kunit *test)
{
	const struct of_device_id *id;

	/* No ID GPIO: the first profile, as for single-profile compatibles. */
	KUNIT_EXPECT_PTR_EQ(test, nt36532_select_variant(&csot_variants, -1),
			    &csot_panel_info);
	KUNIT_EXPECT_NULL(test, nt36532_select_variant(&csot_variants, 1));

	KUNIT_EXPECT_PTR_EQ(test, nt36532_select_variant(&piano_variants, -1),
			    &piano_csot_panel_info);
	KUNIT_EXPECT_PTR_EQ(test, nt36532_select_variant(&piano_variants, 0),
			    &piano_csot_panel_info);
	KUNIT_EXPECT_PTR_EQ(test, nt36532_select_variant(&piano_variants, 1),
			    &piano_boe_panel_info);
	KUNIT_EXPECT_NULL(test, nt36532_select_variant(&piano_variants, 2));

	for (id = nt36532_of_match; id->compatible[0]; id++)
		if (!strcmp(id->compatible, "xiaomi,piano-nt36532"))
			break;
	KUNIT_EXPECT_PTR_EQ(test, id->data, (const void *)&piano_variants);
}

/* Both stock modules use qcom,mdss-dsi-reset-sequence <0 10 1 3 0 3 1 15>. */
static void nt36532_test_piano_reset(struct kunit *test)
{
	KUNIT_EXPECT_PTR_EQ(test, piano_csot_panel_info.reset, &piano_reset);
	KUNIT_EXPECT_PTR_EQ(test, piano_boe_panel_info.reset, &piano_reset);
}

/*
 * The Piano table starts with the 144 Hz mode, whose horizontal timing
 * differs from the 120 Hz one, and the 120 Hz mode is the one whose
 * display path has been validated. Every mode stays advertised.
 */
static void nt36532_test_piano_preferred(struct kunit *test)
{
	unsigned int i;

	/* The table carries the rate in the pixel clock, not in .vrefresh. */
	KUNIT_EXPECT_EQ(test, drm_mode_vrefresh(&piano_display_modes[0]), 144);
	KUNIT_EXPECT_EQ(test, drm_mode_vrefresh(&piano_display_modes[1]), 120);

	KUNIT_EXPECT_EQ(test, nt36532_preferred_mode(&piano_csot_panel_info, true),
			1);
	KUNIT_EXPECT_EQ(test, nt36532_preferred_mode(&piano_boe_panel_info, true),
			1);

	/* Other boards keep the first mode of their table. */
	KUNIT_EXPECT_EQ(test, nt36532_preferred_mode(&piano_csot_panel_info, false),
			0);
	KUNIT_EXPECT_EQ(test, nt36532_preferred_mode(&csot_panel_info, false), 0);

	/* No mode is dropped by the choice. */
	KUNIT_EXPECT_EQ(test, piano_csot_panel_info.num_modes,
			ARRAY_SIZE(piano_display_modes));
	for (i = 0; i < piano_csot_panel_info.num_modes; i++)
		KUNIT_EXPECT_PTR_EQ(test, &piano_csot_panel_info.modes[i],
				    &piano_display_modes[i]);
}

static bool nt36532_test_msg_is(const struct nt36532_test_msg *m,
				const u8 *data, size_t len)
{
	return m->len == len && !memcmp(m->data, data, len);
}

static void nt36532_test_expect_same(struct kunit *test,
				     const struct nt36532_test_msg *a,
				     const struct nt36532_test_msg *b)
{
	KUNIT_EXPECT_EQ(test, a->type, b->type);
	KUNIT_EXPECT_EQ(test, a->flags, b->flags);
	KUNIT_EXPECT_EQ(test, a->len, b->len);
	KUNIT_EXPECT_MEMEQ(test, a->data, b->data, min(a->len, b->len));
}

static unsigned int nt36532_test_find(struct kunit *test, struct nt36532_test *t,
				      unsigned int from, const u8 *data, size_t len)
{
	unsigned int n;

	for (n = from; n < t->count; n++)
		if (nt36532_test_msg_is(&t->msg[n], data, len))
			return n;
	KUNIT_FAIL(test, "no %*ph message from index %u", (int)len, data, from);
	return t->count;
}

/*
 * The stock 35-02-0b (BOE) on-command is the 42-02-0a (CSOT) one with ESD
 * D1 0x00 instead of 0x20, BC 77 07 in place of an eight-command vendor
 * block, and B2 sent before B3 instead of after it.
 */
static void nt36532_test_piano_modules(struct kunit *test)
{
	static const u8 csot_vendor[][2] = {
		{ 0xc4, 0x82 }, { 0xff, 0x26 }, { 0xfb, 0x01 }, { 0x3b, 0x06 },
		{ 0x4b, 0x06 }, { 0xff, 0x22 }, { 0xfb, 0x01 }, { 0xc4, 0x06 },
	};
	static const u8 boe_vendor[] = { 0xbc, 0x77, 0x07 };
	static const u8 csot_d1[] = { 0xd1, 0x20 }, boe_d1[] = { 0xd1, 0x00 };
	const struct drm_display_mode *modes = piano_display_modes;
	unsigned int mode, n, d1, vendor, b2, b3;
	u8 rate[2];

	for (mode = 0; mode < ARRAY_SIZE(piano_display_modes); mode++) {
		struct nt36532_test *csot, *boe;
		int hz = drm_mode_vrefresh(&modes[mode]);

		kunit_info(test, "%d Hz\n", hz);
		csot = nt36532_test_init(test, &piano_csot_panel_info, &modes[mode]);
		boe = nt36532_test_init(test, &piano_boe_panel_info, &modes[mode]);
		KUNIT_ASSERT_EQ(test, csot->count,
				boe->count + ARRAY_SIZE(csot_vendor) - 1);

		d1 = nt36532_test_find(test, csot, 0, csot_d1, sizeof(csot_d1));
		KUNIT_ASSERT_LT(test, d1, csot->count);
		KUNIT_EXPECT_TRUE(test, nt36532_test_msg_is(&boe->msg[d1], boe_d1,
							    sizeof(boe_d1)));

		vendor = nt36532_test_find(test, csot, d1, csot_vendor[0], 2);
		KUNIT_ASSERT_LT(test, vendor + ARRAY_SIZE(csot_vendor), csot->count);
		for (n = 0; n < ARRAY_SIZE(csot_vendor); n++)
			KUNIT_EXPECT_TRUE(test, nt36532_test_msg_is(&csot->msg[vendor + n],
								    csot_vendor[n], 2));
		KUNIT_EXPECT_TRUE(test, nt36532_test_msg_is(&boe->msg[vendor], boe_vendor,
							    sizeof(boe_vendor)));

		switch (hz) {
		case 144:
			rate[0] = 0x00;
			rate[1] = 0x00;
			break;
		case 90:
			rate[0] = 0x00;
			rate[1] = 0x80;
			break;
		default:
			rate[0] = 0x91;
			rate[1] = 0x40;
			break;
		}
		b3 = nt36532_test_find(test, csot, vendor,
				       (const u8[]){ 0xb3, rate[1] }, 2);
		b2 = nt36532_test_find(test, csot, vendor,
				       (const u8[]){ 0xb2, rate[0] }, 2);
		KUNIT_ASSERT_EQ(test, b2, b3 + 1);

		/* Everything else is byte-identical, including the PPS. */
		for (n = 0; n < boe->count; n++) {
			unsigned int c = n < vendor ? n : n + ARRAY_SIZE(csot_vendor) - 1;

			if (n == d1 || n == vendor)
				continue;
			if (c == b3 || c == b2)
				c = c == b3 ? b2 : b3;
			nt36532_test_expect_same(test, &csot->msg[c], &boe->msg[n]);
		}
	}
}

static struct kunit_case nt36532_test_cases[] = {
	KUNIT_CASE(nt36532_test_select),
	KUNIT_CASE(nt36532_test_piano_reset),
	KUNIT_CASE(nt36532_test_piano_preferred),
	KUNIT_CASE(nt36532_test_piano_modules),
	KUNIT_CASE(nt36532_test_log_sequences),
	{}
};

static struct kunit_suite nt36532_test_suite = {
	.name = "nt36532-panel",
	.test_cases = nt36532_test_cases,
};

kunit_test_suite(nt36532_test_suite);
