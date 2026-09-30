// SPDX-License-Identifier: GPL-2.0-only
/* Check the device model's glue-directory ownership across successful moves. */
#include <kunit/test.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/module.h>

static void move_test_unregister(void *data)
{
	device_unregister(data);
}

static void move_test_class_destroy(void *data)
{
	class_destroy(data);
}

static void move_test_root_unregister(void *data)
{
	root_device_unregister(data);
}

enum move_target {
	MOVE_TO_VIRTUAL,
	MOVE_TO_OTHER_PARENT,
	MOVE_TO_SAME_PARENT,
};

static void test_glue_reference(struct kunit *test, enum move_target target)
{
	struct device *parent, *other = NULL, *child;
	struct class *class;
	struct kobject *old_glue;
	unsigned int before, after, expected;
	int ret;

	class = class_create("kunit-move-class");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, class);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, move_test_class_destroy, class), 0);
	parent = root_device_register("kunit-move-parent");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, parent);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, move_test_root_unregister, parent), 0);
	if (target == MOVE_TO_OTHER_PARENT) {
		other = root_device_register("kunit-move-other");
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, other);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, move_test_root_unregister, other), 0);
	}
	child = device_create(class, parent, 0, NULL, "kunit-move-child");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, child);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, move_test_unregister, child), 0);

	/* Keep the old glue directory alive for observation even after detaching. */
	old_glue = kobject_get(child->kobj.parent);
	KUNIT_ASSERT_NOT_NULL(test, old_glue);
	before = kref_read(&old_glue->kref);
	expected = target == MOVE_TO_SAME_PARENT ? before : 1;
	ret = device_move(child, target == MOVE_TO_SAME_PARENT ? parent : other,
			  DPM_ORDER_NONE);
	after = kref_read(&old_glue->kref);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, after, expected);

	/* Clean up the single extra leaked reference when testing the old code.
	 * The expectation above still fails; this only keeps the test isolated.
	 */
	if (!ret && after == expected + 1)
		kobject_put(old_glue);
	kobject_put(old_glue);
}

static void device_move_to_virtual_test(struct kunit *test)
{
	test_glue_reference(test, MOVE_TO_VIRTUAL);
}

static void device_move_between_parents_test(struct kunit *test)
{
	test_glue_reference(test, MOVE_TO_OTHER_PARENT);
}

static void device_move_same_parent_test(struct kunit *test)
{
	test_glue_reference(test, MOVE_TO_SAME_PARENT);
}

static struct kunit_case device_move_cases[] = {
	KUNIT_CASE(device_move_to_virtual_test),
	KUNIT_CASE(device_move_between_parents_test),
	KUNIT_CASE(device_move_same_parent_test),
	{}
};

static struct kunit_suite device_move_suite = {
	.name = "device-move-glue",
	.test_cases = device_move_cases,
};
kunit_test_suite(device_move_suite);

MODULE_LICENSE("GPL");
