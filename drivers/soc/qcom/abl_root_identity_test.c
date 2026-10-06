// SPDX-License-Identifier: GPL-2.0-only
/*
 * Included by abl_root_identity.c. Uses unattached synthetic nodes; the
 * of_update_property() step on the live root is left to boot testing.
 */
#include <kunit/test.h>

static const char abl_test_stock[] = "qcom,sun-mtp\0qcom,sun\0qcom,mtp";
static const char abl_test_mainline[] = "xiaomi,piano\0qcom,sm8750";

static void abl_test_add(struct kunit *test, struct device_node *np,
			 const char *name, const void *value, int length)
{
	struct property *prop = kunit_kzalloc(test, sizeof(*prop), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, prop);
	prop->name = (char *)name;
	prop->value = (void *)value;
	prop->length = length;
	prop->next = np->properties;
	np->properties = prop;
}

static struct abl_root_identity abl_test_compatible(void)
{
	return (struct abl_root_identity) {
		.saved = "xiaomi,mainline-compatible",
		.prop.name = "compatible",
	};
}

static void abl_test_restore_replaced(struct kunit *test)
{
	struct abl_root_identity id = abl_test_compatible();
	struct device_node np = {};

	abl_test_add(test, &np, "compatible", abl_test_stock, sizeof(abl_test_stock));
	abl_test_add(test, &np, id.saved, abl_test_mainline, sizeof(abl_test_mainline));

	KUNIT_EXPECT_EQ(test, abl_root_identity_prepare(&np, &id), 1);
	KUNIT_EXPECT_PTR_EQ(test, id.prop.value, (void *)abl_test_mainline);
	KUNIT_EXPECT_EQ(test, id.prop.length, (int)sizeof(abl_test_mainline));
	KUNIT_EXPECT_STREQ(test, id.prop.name, "compatible");
}

static void abl_test_restore_missing_live(struct kunit *test)
{
	struct abl_root_identity id = {
		.saved = "xiaomi,mainline-model",
		.prop.name = "model",
	};
	static const char model[] = "Xiaomi Pad 8 Pro";
	struct device_node np = {};

	abl_test_add(test, &np, id.saved, model, sizeof(model));

	KUNIT_EXPECT_EQ(test, abl_root_identity_prepare(&np, &id), 1);
	KUNIT_EXPECT_PTR_EQ(test, id.prop.value, (void *)model);
}

static void abl_test_keep_matching(struct kunit *test)
{
	struct abl_root_identity id = abl_test_compatible();
	static const char copy[] = "xiaomi,piano\0qcom,sm8750";
	struct device_node np = {};

	abl_test_add(test, &np, "compatible", copy, sizeof(copy));
	abl_test_add(test, &np, id.saved, abl_test_mainline, sizeof(abl_test_mainline));

	KUNIT_EXPECT_EQ(test, abl_root_identity_prepare(&np, &id), 0);
	KUNIT_EXPECT_NULL(test, id.prop.value);
}

static void abl_test_no_saved_copy(struct kunit *test)
{
	struct abl_root_identity id = abl_test_compatible();
	struct device_node np = {};

	abl_test_add(test, &np, "compatible", abl_test_stock, sizeof(abl_test_stock));

	KUNIT_EXPECT_EQ(test, abl_root_identity_prepare(&np, &id), -ENOENT);
	KUNIT_EXPECT_NULL(test, id.prop.value);
}

static void abl_test_reject_invalid(struct kunit *test)
{
	struct abl_root_identity id = abl_test_compatible();
	static const char unterminated[] = { 'x', 'i', 'a', 'o', 'm', 'i' };
	struct device_node np = {};

	abl_test_add(test, &np, "compatible", abl_test_stock, sizeof(abl_test_stock));
	abl_test_add(test, &np, id.saved, unterminated, sizeof(unterminated));
	KUNIT_EXPECT_EQ(test, abl_root_identity_prepare(&np, &id), -EILSEQ);
	KUNIT_EXPECT_NULL(test, id.prop.value);

	np.properties = NULL;
	abl_test_add(test, &np, "compatible", abl_test_stock, sizeof(abl_test_stock));
	abl_test_add(test, &np, id.saved, abl_test_mainline, 0);
	KUNIT_EXPECT_EQ(test, abl_root_identity_prepare(&np, &id), -ENODATA);
	KUNIT_EXPECT_NULL(test, id.prop.value);
}

static struct kunit_case abl_root_identity_cases[] = {
	KUNIT_CASE(abl_test_restore_replaced),
	KUNIT_CASE(abl_test_restore_missing_live),
	KUNIT_CASE(abl_test_keep_matching),
	KUNIT_CASE(abl_test_no_saved_copy),
	KUNIT_CASE(abl_test_reject_invalid),
	{}
};

static struct kunit_suite abl_root_identity_suite = {
	.name = "qcom-abl-root-identity",
	.test_cases = abl_root_identity_cases,
};
kunit_test_suite(abl_root_identity_suite);
