// SPDX-License-Identifier: GPL-2.0-only
/* Exercise connector rechecks through a PPM transport with no hardware. */
#include <kunit/test.h>

struct ucsi_connection_test {
	struct ucsi ucsi;
	struct ucsi_connector con;
	u16 change;
	u16 observed;
	int error;
	unsigned int callbacks;
};

static int ucsi_test_sync(struct ucsi *ucsi, u64 command, u32 *cci,
			  void *data, size_t size, void *out, size_t out_size)
{
	struct ucsi_connection_test *ctx = ucsi_get_drvdata(ucsi);

	if (UCSI_COMMAND(command) == UCSI_ACK_CC_CI)
		return 0;
	if (ctx->error)
		return ctx->error;

	memset(data, 0, size);
	bitmap_write(data, ctx->change, 0, 16);
	bitmap_set(data, 19, 1);
	*cci = UCSI_CCI_COMMAND_COMPLETE | (size << 8);
	return 0;
}

static void ucsi_test_status(struct ucsi_connector *con)
{
	struct ucsi_connection_test *ctx = ucsi_get_drvdata(con->ucsi);

	ctx->callbacks++;
	ctx->observed = UCSI_CONSTAT(con, CHANGE);
}

static int ucsi_connection_test_init(struct kunit *test)
{
	static const struct ucsi_operations ops = {
		.sync_control = ucsi_test_sync,
		.connector_status = ucsi_test_status,
	};
	struct ucsi_connection_test *ctx;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	test->priv = ctx;
	ctx->ucsi.ops = &ops;
	ctx->ucsi.version = UCSI_VERSION_1_2;
	mutex_init(&ctx->ucsi.ppm_lock);
	ucsi_set_drvdata(&ctx->ucsi, ctx);
	ctx->con.ucsi = &ctx->ucsi;
	ctx->con.num = 1;
	bitmap_set(ctx->con.status, 19, 1);
	return 0;
}

static void ucsi_test_recheck_connected(struct kunit *test)
{
	struct ucsi_connection_test *ctx = test->priv;

	/* Same connected bit, but a new physical connection during sleep. */
	ctx->change = UCSI_CONSTAT_CONNECT_CHANGE;
	KUNIT_ASSERT_EQ(test, ucsi_check_connection(&ctx->con), 0);
	KUNIT_EXPECT_EQ(test, ctx->callbacks, 1U);
	KUNIT_EXPECT_EQ(test, ctx->observed, UCSI_CONSTAT_CONNECT_CHANGE);

	/* The delayed one-second check also reports the fresh status. */
	ctx->change = 0;
	KUNIT_ASSERT_EQ(test, ucsi_check_connection(&ctx->con), 0);
	KUNIT_EXPECT_EQ(test, ctx->callbacks, 2U);
	KUNIT_EXPECT_EQ(test, ctx->observed, 0);
}

static void ucsi_test_read_failure(struct kunit *test)
{
	struct ucsi_connection_test *ctx = test->priv;

	ctx->error = -EIO;
	KUNIT_EXPECT_EQ(test, ucsi_get_connector_status(&ctx->con, false), -EIO);
	KUNIT_EXPECT_EQ(test, ctx->callbacks, 0U);
}

static struct kunit_case ucsi_connection_cases[] = {
	KUNIT_CASE(ucsi_test_recheck_connected),
	KUNIT_CASE(ucsi_test_read_failure),
	{}
};

static struct kunit_suite ucsi_connection_suite = {
	.name = "ucsi-connection",
	.init = ucsi_connection_test_init,
	.test_cases = ucsi_connection_cases,
};

kunit_test_suite(ucsi_connection_suite);
