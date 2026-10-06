// SPDX-License-Identifier: GPL-2.0-only
/*
 * Restore the device tree root identity replaced by Qualcomm ABL.
 *
 * Some ABL builds copy the root "model" and "compatible" of the vendor DTBO
 * they apply over those of the booted DTB. Root match tables (PD mapper, UBWC
 * configuration, ...) then no longer recognise the SoC. A DTB that keeps its
 * own identity in root properties the DTBO does not carry gets it back here,
 * before the first core initcall. /sys/firmware/fdt keeps the ABL values.
 */

#define pr_fmt(fmt) "abl-root-identity: " fmt

#include <linux/array_size.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/string.h>

struct abl_root_identity {
	const char *saved;
	struct property prop;
};

/* Linked into the live root node, so not __initdata. */
static struct abl_root_identity abl_root_identity[] = {
	{ .saved = "xiaomi,mainline-model", .prop.name = "model" },
	{ .saved = "xiaomi,mainline-compatible", .prop.name = "compatible" },
};

/*
 * Point @id->prop at the saved copy in @np. Returns 1 if it differs from the
 * live property, 0 if they are equal, -ENOENT without a saved copy and another
 * negative errno if the saved copy is not a non-empty string list.
 */
static int abl_root_identity_prepare(const struct device_node *np,
				     struct abl_root_identity *id)
{
	const struct property *saved, *live;
	int ret;

	saved = of_find_property(np, id->saved, NULL);
	if (!saved)
		return -ENOENT;

	ret = of_property_count_strings(np, id->saved);
	if (ret < 0)
		return ret;

	live = of_find_property(np, id->prop.name, NULL);
	if (live && live->length == saved->length &&
	    !memcmp(live->value, saved->value, saved->length))
		return 0;

	id->prop.value = saved->value;
	id->prop.length = saved->length;
	return 1;
}

static int __init abl_root_identity_init(void)
{
	struct device_node *root __free(device_node) = of_find_node_by_path("/");
	const char *model = "", *compatible = "";
	unsigned int saved = 0, restored = 0;
	int i, ret;

	if (!root)
		return 0;

	/* Replaced properties stay on the node's deadprops list. */
	of_property_read_string(root, "model", &model);
	of_property_read_string(root, "compatible", &compatible);

	for (i = 0; i < ARRAY_SIZE(abl_root_identity); i++) {
		struct abl_root_identity *id = &abl_root_identity[i];

		ret = abl_root_identity_prepare(root, id);
		if (ret == -ENOENT)
			continue;
		if (ret < 0) {
			pr_err("ignoring invalid %s: %d\n", id->saved, ret);
			continue;
		}
		saved++;
		if (ret && !of_update_property(root, &id->prop))
			restored++;
	}

	if (restored) {
		const char *new_model = "", *new_compatible = "";

		of_property_read_string(root, "model", &new_model);
		of_property_read_string(root, "compatible", &new_compatible);
		pr_info("restored model \"%s\" -> \"%s\", compatible \"%s\" -> \"%s\"\n",
			model, new_model, compatible, new_compatible);
	} else if (saved) {
		pr_info("root identity already matches the saved copy\n");
	}

	return 0;
}
early_initcall(abl_root_identity_init);

#if IS_ENABLED(CONFIG_QCOM_ABL_ROOT_IDENTITY_KUNIT_TEST)
#include "abl_root_identity_test.c"
#endif
