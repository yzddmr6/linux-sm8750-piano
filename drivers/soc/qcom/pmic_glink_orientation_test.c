// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>
#include <linux/soc/qcom/pmic_glink.h>

static void pmic_glink_orientation_priority_test(struct kunit *test)
{
	struct pmic_glink_orientation state = {};
	enum typec_orientation direction;

	direction = pmic_glink_orientation_update(&state,
						  TYPEC_ORIENTATION_NORMAL, false);
	KUNIT_EXPECT_EQ(test, direction, TYPEC_ORIENTATION_NORMAL);
	direction = pmic_glink_orientation_update(&state,
						  TYPEC_ORIENTATION_REVERSE, true);
	KUNIT_EXPECT_EQ(test, direction, TYPEC_ORIENTATION_REVERSE);
	/* Even a late attach GPIO must not flip the already programmed AUX. */
	direction = pmic_glink_orientation_update(&state,
						  TYPEC_ORIENTATION_NORMAL, false);
	KUNIT_EXPECT_EQ(test, direction, TYPEC_ORIENTATION_REVERSE);
}

static void pmic_glink_orientation_disconnect_test(struct kunit *test)
{
	struct pmic_glink_orientation state = {};
	enum typec_orientation direction;

	pmic_glink_orientation_update(&state, TYPEC_ORIENTATION_REVERSE, true);
	state.warned = true;
	direction = pmic_glink_orientation_update(&state,
						  TYPEC_ORIENTATION_NONE, false);
	KUNIT_EXPECT_EQ(test, direction, TYPEC_ORIENTATION_NONE);
	KUNIT_EXPECT_FALSE(test, state.warned);
	direction = pmic_glink_orientation_update(&state,
						  TYPEC_ORIENTATION_NORMAL, false);
	KUNIT_EXPECT_EQ(test, direction, TYPEC_ORIENTATION_NORMAL);
}

static void pmic_glink_orientation_pdr_test(struct kunit *test)
{
	struct pmic_glink_orientation state = {};
	enum typec_orientation direction;

	pmic_glink_orientation_update(&state, TYPEC_ORIENTATION_REVERSE, true);
	atomic_inc(&state.epoch);
	/* A queued notification from the dead service is no longer valid. */
	KUNIT_EXPECT_FALSE(test, pmic_glink_orientation_sync(&state, 0));
	KUNIT_ASSERT_TRUE(test, pmic_glink_orientation_sync(&state, 1));
	KUNIT_EXPECT_EQ(test, state.altmode, TYPEC_ORIENTATION_NONE);
	direction = pmic_glink_orientation_update(&state,
						  TYPEC_ORIENTATION_NORMAL, false);
	KUNIT_EXPECT_EQ(test, direction, TYPEC_ORIENTATION_NORMAL);
}

static struct kunit_case pmic_glink_orientation_cases[] = {
	KUNIT_CASE(pmic_glink_orientation_priority_test),
	KUNIT_CASE(pmic_glink_orientation_disconnect_test),
	KUNIT_CASE(pmic_glink_orientation_pdr_test),
	{}
};

static struct kunit_suite pmic_glink_orientation_suite = {
	.name = "pmic-glink-orientation",
	.test_cases = pmic_glink_orientation_cases,
};

kunit_test_suite(pmic_glink_orientation_suite);
MODULE_LICENSE("GPL");
