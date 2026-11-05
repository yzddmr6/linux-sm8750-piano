// SPDX-License-Identifier: GPL-2.0-only
/* Exercise the actual registration worker and UCSI synchronous error paths. */
#include <kunit/device.h>
#include <kunit/test.h>

struct glink_ucsi_test {
	struct pmic_glink_ucsi glink;
	int version_error;
	u16 version;
	unsigned int unregister_commands;
};

static int glink_test_version(struct ucsi *ucsi, u16 *version)
{
	struct glink_ucsi_test *f = ucsi_get_drvdata(ucsi);

	*version = f->version;
	return f->version_error;
}

static int glink_test_cci(struct ucsi *ucsi, u32 *cci)
{
	return -EIO;
}

static int glink_test_message(struct ucsi *ucsi, void *data, size_t size)
{
	return -EIO;
}

static int glink_test_sync(struct ucsi *ucsi, u64 command, u32 *cci,
			   void *data, size_t size, void *out, size_t out_size)
{
	return -EIO;
}

static int glink_test_async(struct ucsi *ucsi, u64 command)
{
	struct glink_ucsi_test *f = ucsi_get_drvdata(ucsi);

	if (command == UCSI_SET_NOTIFICATION_ENABLE)
		f->unregister_commands++;
	return -EIO;
}

static void glink_test_cleanup(void *data)
{
	struct glink_ucsi_test *f = data;

	cancel_delayed_work_sync(&f->glink.ucsi->work);
	if (f->glink.ucsi_registered)
		ucsi_unregister(f->glink.ucsi);
	ucsi_destroy(f->glink.ucsi);
}

static void glink_test_registration(struct kunit *test)
{
	static const struct ucsi_operations ops = {
		.read_version = glink_test_version,
		.read_cci = glink_test_cci,
		.poll_cci = glink_test_cci,
		.read_message_in = glink_test_message,
		.sync_control = glink_test_sync,
		.async_control = glink_test_async,
	};
	struct glink_ucsi_test *f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);
	struct auxiliary_device *adev = kunit_kzalloc(test, sizeof(*adev), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, adev);
	f->glink.dev = kunit_device_register(test, "ucsi-glink-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, f->glink.dev);
	spin_lock_init(&f->glink.state_lock);
	INIT_WORK(&f->glink.register_work, pmic_glink_ucsi_register);
	INIT_WORK(&f->glink.notify_work, pmic_glink_ucsi_notify);
	f->glink.ucsi = ucsi_create(f->glink.dev, &ops);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, f->glink.ucsi);
	ucsi_set_drvdata(f->glink.ucsi, f);
	kunit_add_action_or_reset(test, glink_test_cleanup, f);

	f->glink.pd_running = true;
	f->version_error = -ETIMEDOUT;
	pmic_glink_ucsi_register(&f->glink.register_work);
	KUNIT_EXPECT_FALSE(test, f->glink.ucsi_registered);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&f->glink.ucsi->work));
	f->glink.pd_running = false;
	pmic_glink_ucsi_register(&f->glink.register_work);
	KUNIT_EXPECT_EQ(test, f->unregister_commands, 0);

	f->glink.pd_running = true;
	f->version_error = 0;
	f->version = 0;
	pmic_glink_ucsi_register(&f->glink.register_work);
	KUNIT_EXPECT_FALSE(test, f->glink.ucsi_registered);
	dev_set_drvdata(&adev->dev, &f->glink);
	pmic_glink_ucsi_remove(adev);
	KUNIT_EXPECT_EQ(test, f->unregister_commands, 0);

	/* The success branch still owns exactly one unregister on service down. */
	f->version = 0x100;
	pmic_glink_ucsi_register(&f->glink.register_work);
	KUNIT_EXPECT_TRUE(test, f->glink.ucsi_registered);
	f->glink.pd_running = false;
	pmic_glink_ucsi_register(&f->glink.register_work);
	KUNIT_EXPECT_FALSE(test, f->glink.ucsi_registered);
	KUNIT_EXPECT_EQ(test, f->unregister_commands, 1);
	pmic_glink_ucsi_register(&f->glink.register_work);
	KUNIT_EXPECT_EQ(test, f->unregister_commands, 1);
}

static struct kunit_case glink_ucsi_cases[] = {
	KUNIT_CASE(glink_test_registration),
	{}
};

static struct kunit_suite glink_ucsi_suite = {
	.name = "ucsi-pmic-glink",
	.test_cases = glink_ucsi_cases,
};
kunit_test_suite(glink_ucsi_suite);
