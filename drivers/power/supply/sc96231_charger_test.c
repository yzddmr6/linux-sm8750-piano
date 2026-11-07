// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>
#include <kunit/device.h>

static void sc96231_test_hall(struct kunit *test)
{
	bool hall[6] = {}, keyboard[2] = {};
	int i;

	KUNIT_EXPECT_FALSE(test, sc96231_hall_docked(hall, keyboard));
	/* Folding the keyboard, including its Hall4-S magnet, is not a pen. */
	hall[3] = true;
	for (i = 0; i < 4; i++) {
		keyboard[0] = i & 1;
		keyboard[1] = i & 2;
		KUNIT_EXPECT_FALSE(test, sc96231_hall_docked(hall, keyboard));
	}
	memset(hall, 0, sizeof(hall));
	memset(keyboard, 0, sizeof(keyboard));
	hall[0] = true;
	KUNIT_EXPECT_TRUE(test, sc96231_hall_docked(hall, keyboard));
	hall[0] = false;
	hall[1] = true;
	KUNIT_EXPECT_TRUE(test, sc96231_hall_docked(hall, keyboard));
	for (i = 2; i < 6; i++) {
		hall[i] = true;
		KUNIT_EXPECT_FALSE(test, sc96231_hall_docked(hall, keyboard));
		hall[i] = false;
	}
	/* Stock ignores Hall4-S only when a keyboard position is asserted. */
	hall[3] = keyboard[0] = true;
	KUNIT_EXPECT_TRUE(test, sc96231_hall_docked(hall, keyboard));
}

static void sc96231_test_lifecycle(struct kunit *test)
{
	struct sc96231 sc = {};
	unsigned long now = 1000 * HZ;

	KUNIT_EXPECT_FALSE(test, sc96231_can_charge(&sc));
	KUNIT_EXPECT_EQ(test, sc96231_deadline(&sc, now), 0UL);
	sc.docked = true;
	KUNIT_EXPECT_TRUE(test, sc96231_can_charge(&sc));
	sc.enabled = true;
	sc.started = now;
	KUNIT_EXPECT_EQ(test, sc96231_deadline(&sc, now), 20UL * HZ);
	sc.present = true;
	KUNIT_EXPECT_EQ(test, sc96231_deadline(&sc, now), 0UL);
	sc.soc_valid = true;
	sc.capacity = 100;
	sc.full_since = now;
	KUNIT_EXPECT_EQ(test, sc96231_deadline(&sc, now), 180UL * HZ);
	sc.full = true;
	KUNIT_EXPECT_FALSE(test, sc96231_can_charge(&sc));
	KUNIT_EXPECT_EQ(test, sc96231_deadline(&sc, now), 0UL);
	/* Resume/removal starts a new session: full is not a permanent latch. */
	sc.address[0] = 'x';
	sc96231_new_session(&sc);
	KUNIT_EXPECT_EQ(test, sc.address[0], '\0');
	KUNIT_EXPECT_TRUE(test, sc96231_can_charge(&sc));
	KUNIT_EXPECT_FALSE(test, sc.soc_valid);
	sc.suspended = true;
	KUNIT_EXPECT_FALSE(test, sc96231_can_charge(&sc));
	KUNIT_EXPECT_EQ(test, sc96231_deadline(&sc, now), 0UL);
	sc.suspended = false;
	sc.fault = true;
	KUNIT_EXPECT_FALSE(test, sc96231_can_charge(&sc));
	KUNIT_EXPECT_EQ(test, sc96231_deadline(&sc, now), 0UL);
	sc96231_new_session(&sc);
	sc.docked = false;
	KUNIT_EXPECT_FALSE(test, sc96231_can_charge(&sc));
	KUNIT_EXPECT_EQ(test, sc96231_deadline(&sc, now), 0UL);
}

static void sc96231_test_notifications(struct kunit *test)
{
	struct sc96231 sc = {};
	struct sc96231_visible a, b;

	a = sc96231_snapshot(&sc);
	b = sc96231_snapshot(&sc);
	KUNIT_EXPECT_FALSE(test, sc96231_changed(&a, &b));
	sc.present = sc.enabled = sc.soc_valid = true;
	sc.capacity = 50;
	b = sc96231_snapshot(&sc);
	KUNIT_EXPECT_TRUE(test, sc96231_changed(&a, &b));
	a = b;
	b = sc96231_snapshot(&sc);
	KUNIT_EXPECT_FALSE(test, sc96231_changed(&a, &b));
	strscpy(sc.address, "02:00:00:00:00:01", sizeof(sc.address));
	b = sc96231_snapshot(&sc);
	KUNIT_EXPECT_TRUE(test, sc96231_changed(&a, &b));
	a = b;
	sc.capacity = 51;
	b = sc96231_snapshot(&sc);
	KUNIT_EXPECT_TRUE(test, sc96231_changed(&a, &b));
}

struct sc96231_fixture {
	struct sc96231 sc;
	bool attached;
	u32 irq;
	u8 soc;
	int starts, stops, notifications, start_error;
};

static int sc96231_fake_docked(struct sc96231 *sc)
{
	return container_of(sc, struct sc96231_fixture, sc)->attached;
}

static int sc96231_fake_start(struct sc96231 *sc)
{
	struct sc96231_fixture *f = container_of(sc, struct sc96231_fixture, sc);

	f->starts++;
	if (f->start_error)
		return f->start_error;
	sc->enabled = true;
	sc->started = jiffies;
	return 0;
}

static void sc96231_fake_stop(struct sc96231 *sc)
{
	struct sc96231_fixture *f = container_of(sc, struct sc96231_fixture, sc);

	if (sc->enabled)
		f->stops++;
	sc->enabled = false;
}

static void sc96231_fake_notify(struct sc96231 *sc)
{
	container_of(sc, struct sc96231_fixture, sc)->notifications++;
}

static int sc96231_fake_read(struct sc96231 *sc, u16 reg, void *data, size_t size)
{
	struct sc96231_fixture *f = container_of(sc, struct sc96231_fixture, sc);
	u8 *bytes = data;

	memset(data, 0, size);
	if (reg == SC_IRQ_STATUS && size == 4)
		put_unaligned_le32(f->irq, data);
	else if (reg == SC_PEN_SOC && size == 4)
		bytes[0] = f->soc;
	else if (reg == SC_PEN_MAC && size == 8) {
		/* Synthetic locally administered address, not a device capture. */
		bytes[2] = 1;
		bytes[7] = 2;
	} else
		return -EINVAL;
	return 0;
}

static int sc96231_fake_write(struct sc96231 *sc, u16 reg, u32 value)
{
	struct sc96231_fixture *f = container_of(sc, struct sc96231_fixture, sc);

	if (reg != SC_IRQ_CLEAR || value != f->irq)
		return -EINVAL;
	f->irq = 0;
	return 0;
}

static void sc96231_fixture_free(void *data)
{
	struct sc96231_fixture *f = data;

	cancel_delayed_work_sync(&f->sc.work);
	input_free_device(f->sc.dock);
}

static void sc96231_test_worker(struct kunit *test)
{
	struct sc96231_fixture *f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);
	struct sc96231 *sc;
	int notifications;

	KUNIT_ASSERT_NOT_NULL(test, f);
	sc = &f->sc;
	sc->dev = kunit_device_register(test, "sc96231-state-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, sc->dev);
	dev_set_drvdata(sc->dev, sc);
	sc->dock = input_allocate_device();
	KUNIT_ASSERT_NOT_NULL(test, sc->dock);
	input_set_capability(sc->dock, EV_SW, SW_PEN_INSERTED);
	mutex_init(&sc->lock);
	INIT_DELAYED_WORK(&sc->work, sc96231_work);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, sc96231_fixture_free, f), 0);
	kunit_activate_static_stub(test, sc96231_docked, sc96231_fake_docked);
	kunit_activate_static_stub(test, sc96231_start, sc96231_fake_start);
	kunit_activate_static_stub(test, sc96231_stop, sc96231_fake_stop);
	kunit_activate_static_stub(test, sc96231_notify, sc96231_fake_notify);
	kunit_activate_static_stub(test, sc96231_read, sc96231_fake_read);
	kunit_activate_static_stub(test, sc96231_write_u32, sc96231_fake_write);

	/* Boot without a pen causes neither TX nor a future work item. */
	sc96231_work(&sc->work.work);
	KUNIT_EXPECT_EQ(test, f->starts, 0);
	KUNIT_EXPECT_EQ(test, f->notifications, 0);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&sc->work));

	f->attached = true;
	sc96231_work(&sc->work.work);
	KUNIT_EXPECT_EQ(test, f->starts, 1);
	KUNIT_EXPECT_TRUE(test, delayed_work_pending(&sc->work));
	cancel_delayed_work_sync(&sc->work);
	f->irq = SC_IRQ_CONFIG | SC_IRQ_SOC;
	f->soc = 50;
	sc96231_work(&sc->work.work);
	KUNIT_EXPECT_TRUE(test, sc->present);
	KUNIT_EXPECT_EQ(test, sc->capacity, 50);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&sc->work));
	notifications = f->notifications;
	f->irq = SC_IRQ_SOC;
	sc96231_work(&sc->work.work);
	KUNIT_EXPECT_EQ(test, f->notifications, notifications);

	f->soc = 100 | BIT(7);
	f->irq = SC_IRQ_SOC;
	sc96231_work(&sc->work.work);
	KUNIT_EXPECT_TRUE(test, sc->full);
	KUNIT_EXPECT_FALSE(test, sc->enabled);
	KUNIT_EXPECT_EQ(test, f->stops, 1);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&sc->work));

	KUNIT_ASSERT_EQ(test, sc96231_prepare(sc->dev), 0);
	sc96231_irq(0, sc);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&sc->work));
	/* Hold dispatch so the real complete callback can run in this test. */
	disable_delayed_work_sync(&sc->work);
	sc96231_complete(sc->dev);
	KUNIT_EXPECT_FALSE(test, sc->full);
	KUNIT_EXPECT_FALSE(test, sc->suspended);
	KUNIT_EXPECT_EQ(test, sc->address[0], '\0');
	enable_delayed_work(&sc->work);
	sc96231_work(&sc->work.work);
	cancel_delayed_work_sync(&sc->work);
	KUNIT_EXPECT_EQ(test, f->starts, 2);
	f->soc = 70;
	f->irq = SC_IRQ_CONFIG | SC_IRQ_SOC;
	sc96231_work(&sc->work.work);
	KUNIT_EXPECT_TRUE(test, sc->enabled);
	KUNIT_EXPECT_FALSE(test, sc->full);
	KUNIT_EXPECT_EQ(test, sc->capacity, 70);

	/* A provider failure must stop TX and cease reporting CHARGING. */
	disable_delayed_work_sync(&sc->work);
	KUNIT_EXPECT_EQ(test, sc96231_supply_event(&sc->supply_notifier,
						REGULATOR_EVENT_FAIL, NULL), NOTIFY_OK);
	enable_delayed_work(&sc->work);
	sc96231_work(&sc->work.work);
	KUNIT_EXPECT_TRUE(test, sc->fault);
	KUNIT_EXPECT_FALSE(test, sc->present);
	KUNIT_EXPECT_FALSE(test, sc->soc_valid);
	KUNIT_EXPECT_EQ(test, sc->address[0], '\0');
	KUNIT_EXPECT_FALSE(test, sc->enabled);
	f->attached = false;
	sc96231_work(&sc->work.work);
	f->attached = true;
	sc96231_work(&sc->work.work);
	cancel_delayed_work_sync(&sc->work);
	f->irq = SC_IRQ_CONFIG | SC_IRQ_SOC;
	sc96231_work(&sc->work.work);

	f->irq = SC_IRQ_FOD;
	sc96231_work(&sc->work.work);
	KUNIT_EXPECT_TRUE(test, sc->fault);
	KUNIT_EXPECT_FALSE(test, sc->present);
	KUNIT_EXPECT_FALSE(test, sc->soc_valid);
	KUNIT_EXPECT_EQ(test, sc->address[0], '\0');
	KUNIT_EXPECT_FALSE(test, sc->enabled);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&sc->work));
	sc96231_work(&sc->work.work);
	KUNIT_EXPECT_EQ(test, f->starts, 3);
	f->attached = false;
	sc96231_work(&sc->work.work);
	KUNIT_EXPECT_FALSE(test, sc->fault);
	KUNIT_EXPECT_FALSE(test, sc->present);
	KUNIT_EXPECT_EQ(test, sc->address[0], '\0');
	f->attached = true;
	f->start_error = -EBUSY;
	sc96231_work(&sc->work.work);
	KUNIT_EXPECT_TRUE(test, sc->fault);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&sc->work));
}

static struct kunit_case sc96231_cases[] = {
	KUNIT_CASE(sc96231_test_worker),
	KUNIT_CASE(sc96231_test_hall),
	KUNIT_CASE(sc96231_test_lifecycle),
	KUNIT_CASE(sc96231_test_notifications),
	{}
};

static struct kunit_suite sc96231_suite = {
	.name = "sc96231-charger",
	.test_cases = sc96231_cases,
};
kunit_test_suite(sc96231_suite);
