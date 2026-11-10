// TEMPORARY TEST CODE: pressing Button 1 on the DK pretends a dose was dispensed.
// It builds a log entry from the current pill templates and hands it to
// pill_service_log_dispense(), so the log read/notify can be tested before the
// real dispensing hardware exists. To remove it, delete this file from CMakeLists.txt.

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "../BLE/pill_service.h"

LOG_MODULE_REGISTER(test_dispense, LOG_LEVEL_INF);

// sw0 is Button 1 on the nRF52833 DK (see the board's .dts file)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb_data;

static void simulate_dispense(struct k_work *work)
{
	struct log_entry entry = {0};
	struct pill_template tmpl;

	// no RTC yet: seconds since boot stand in for the Unix timestamp
	entry.timestamp = (uint32_t)(k_uptime_get() / 1000);

	for (uint8_t i = 0; i < PILL_CONTAINERS; i++) {
		if (pill_service_get_template(i, &tmpl) == 0) {
			entry.pills[entry.num_pills].pill_id = tmpl.pill_id;
			entry.pills[entry.num_pills].disp_cnt = tmpl.dose;
			entry.num_pills++;
		}
	}

	if (entry.num_pills == 0) {
		LOG_WRN("Button pressed but no containers have a template; write one first");
		return;
	}

	LOG_INF("Simulating dispense from %u container(s)", entry.num_pills);
	pill_service_log_dispense(&entry);
}

K_WORK_DEFINE(dispense_work, simulate_dispense);

// Runs in interrupt context, where locking a mutex or sending Bluetooth data
// isn't allowed, so it only queues simulate_dispense() to run on the system work queue thread.
static void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_work_submit(&dispense_work);
}

static int test_dispense_init(void)
{
	if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("Button 1 GPIO not ready");
		return -ENODEV;
	}

	int err = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (err) {
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		return err;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	LOG_INF("Test mode: press Button 1 to simulate a dispense");
	return 0;
}

// runs automatically during boot, before main(), so main.c needs no changes
SYS_INIT(test_dispense_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
