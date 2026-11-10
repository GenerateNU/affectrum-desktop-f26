// #include <stdio.h>
#include <zephyr/kernel.h>
// zephyr/device.h is included eventually through kernel.h
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

// quotes (not <>) tell the compiler to look relative to this file first, which is how it finds our own headers
#include "BLE/BLE.h"
#include "load-cell/loadCell.h"

// This is accessing the DeviceTree node through its label (can see the entire devicetree in generated zephyr.dts file in the build directory)
#define LED2 DT_ALIAS(led0)

// 'gpios' refers to the property name within the led0 node, which is itself inside the leds parent node in the devicetree
// you can see this^^ in code if you search for the 'zephyr/boards/nordic/nrf54l15dk/nrf54l15dk_common.dtsi' file
static const struct gpio_dt_spec led_2 = GPIO_DT_SPEC_GET(LED2, gpios);

LOG_MODULE_REGISTER(app);

/* ---- Load Cell Configuration ------------------------------------------------------ */

/* Set to 1 to run the calibration routine at boot, then copy the printed
 * scale factor into CAL_COUNTS_PER_GRAM and set this back to 0. */
#define CALIBRATION_MODE        0

/* Mass you will place on the cell during calibration (grams). */
#define CAL_KNOWN_MASS_G        50.0f

/* Calibration countdowns (seconds): time to empty the cell before taring,
 * then time to place the known mass and let it stop wobbling. */
#define CAL_EMPTY_WAIT_S        5
#define CAL_SETTLE_S            10

/* Scale factor from a previous calibration run (counts per gram).
 * Replace this after calibrating. */
#define CAL_COUNTS_PER_GRAM     1.0f

#define TARE_SAMPLES            64      /* averaging for the zero point */
#define CAL_SAMPLES             64      /* averaging for calibration */
#define READ_SAMPLES            16      /* averaging for each reading */
#define SAMPLE_PERIOD_MS        100

int main(void)
{
    // start advertising first; the LED keeps blinking either way so we can tell the board is alive
    if (ble_init() != 0)
    {
        LOG_ERR("BLE setup failed, continuing without it\n");
    }

    int err;

	LOG_INF("Load cell demo starting");

	err = loadcell_init();
	if (err) {
		LOG_ERR("Load cell init failed (%d)", err);
		return err;
	}

#if CALIBRATION_MODE
	err = loadcell_calibrate_guided(CAL_KNOWN_MASS_G, CAL_EMPTY_WAIT_S,
					CAL_SETTLE_S, CAL_SAMPLES);
	if (err) {
		LOG_ERR("Calibration failed (%d)", err);
		return err;
	}

	LOG_INF("Set CAL_COUNTS_PER_GRAM to %.4f, set CALIBRATION_MODE to 0, "
		"and rebuild", (double)loadcell_get_scale());
#else
	loadcell_set_scale(CAL_COUNTS_PER_GRAM);

	LOG_INF("Keep the load cell empty, taring...");
	k_sleep(K_SECONDS(2));          /* let power and signal settle */

	err = loadcell_tare(TARE_SAMPLES);
	if (err) {
		LOG_ERR("Tare failed (%d)", err);
		return err;
	}
#endif

	LOG_INF("Reading weight (scale = %.4f counts/g)",
		(double)loadcell_get_scale());

	while (1) {
		float grams;

		err = loadcell_read_grams(READ_SAMPLES, &grams);
		if (err) {
			LOG_ERR("Read failed (%d)", err);
		} else {
			LOG_INF("Weight: %.2f g", (double)grams);
		}

		k_msleep(SAMPLE_PERIOD_MS);
	}

	return 0;
}

    // // led_0.port is used because you need to check if the gpio controller itself is ready to be used, if you just do led_0.pin you will get a number like '9'
    // if (!device_is_ready(led_2.port))
    // {
    //     return 0;
    // }

    // int state = 0;

    // int ret;
    // // configuration of led_0 pin
    // ret = gpio_pin_configure_dt(&led_2, GPIO_OUTPUT);

    // if (ret != 0)
    // {
    //     return 0;
    // }

    // while (true)
    // {
    //     state = !state;
    //     ret = gpio_pin_set_dt(&led_2, state);
    //     if (ret != 0)
    //     {
    //         return 0;
    //     }
    //     LOG_DBG("LED is now in state: %d", state);

    //     k_msleep(4000);
    // }

    // return 0;

