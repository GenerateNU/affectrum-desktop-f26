#include "loadCell.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(loadcell, LOG_LEVEL_INF);

static const struct adc_dt_spec adc_ch =
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);

static int16_t sample_buf;
static struct adc_sequence seq = {
	.buffer = &sample_buf,
	.buffer_size = sizeof(sample_buf),
};

static int32_t tare_offset;
static float counts_per_gram = 1.0f;
static bool initialized;

int loadcell_init(void)
{
	int err;

	if (!adc_is_ready_dt(&adc_ch)) {
		LOG_ERR("ADC device %s not ready", adc_ch.dev->name);
		return -ENODEV;
	}

	err = adc_channel_setup_dt(&adc_ch);
	if (err < 0) {
		LOG_ERR("ADC channel setup failed (%d)", err);
		return err;
	}

	err = adc_sequence_init_dt(&adc_ch, &seq);
	if (err < 0) {
		LOG_ERR("ADC sequence init failed (%d)", err);
		return err;
	}

	initialized = true;
	LOG_INF("Initialized (ADC channel %d, %d-bit)",
		adc_ch.channel_id, adc_ch.resolution);
	return 0;
}

int loadcell_read_raw(int n, int32_t *out)
{
	int64_t sum = 0;

	if (!initialized) {
		LOG_ERR("Read attempted before init");
		return -EACCES;
	}
	if (n < 1 || out == NULL) {
		LOG_ERR("Invalid arguments (n=%d, out=%p)", n, out);
		return -EINVAL;
	}

	for (int i = 0; i < n; i++) {
		int err = adc_read_dt(&adc_ch, &seq);

		if (err < 0) {
			LOG_ERR("adc_read failed on sample %d/%d (%d)",
				i + 1, n, err);
			return err;
		}
		sum += sample_buf; /* signed in differential mode */
		k_msleep(2);
	}

	*out = (int32_t)(sum / n);
	LOG_DBG("Raw average over %d samples: %d", n, *out);
	return 0;
}

int loadcell_tare(int n)
{
	int32_t raw;
	int err = loadcell_read_raw(n, &raw);

	if (err < 0) {
		LOG_ERR("Tare failed (%d)", err);
		return err;
	}
	tare_offset = raw;
	LOG_INF("Tared: offset = %d counts", tare_offset);
	return 0;
}

void loadcell_set_scale(float cpg)
{
	if (cpg == 0.0f) {
		LOG_WRN("Ignoring scale factor of 0");
		return;
	}
	counts_per_gram = cpg;
	LOG_INF("Scale set: %.4f counts/g", (double)counts_per_gram);
}

int loadcell_calibrate(float known_grams, int n)
{
	int32_t raw;
	int err;

	if (known_grams <= 0.0f) {
		LOG_ERR("Invalid calibration mass: %.2f g", (double)known_grams);
		return -EINVAL;
	}

	err = loadcell_read_raw(n, &raw);
	if (err < 0) {
		LOG_ERR("Calibration read failed (%d)", err);
		return err;
	}

	int32_t delta = raw - tare_offset;

	if (delta == 0) {
		LOG_WRN("No change from tare; is the mass on the cell?");
		return -ERANGE;
	}

	counts_per_gram = (float)delta / known_grams;
	LOG_INF("Calibrated: %.2f g -> %d counts, scale = %.4f counts/g",
		(double)known_grams, delta, (double)counts_per_gram);
	return 0;
}

int loadcell_read_grams(int n, float *grams)
{
	int32_t raw;
	int err;

	if (grams == NULL) {
		LOG_ERR("NULL output pointer");
		return -EINVAL;
	}

	err = loadcell_read_raw(n, &raw);
	if (err < 0) {
		return err;
	}

	*grams = (float)(raw - tare_offset) / counts_per_gram;
	LOG_DBG("raw=%d delta=%d -> %.2f g",
		raw, raw - tare_offset, (double)*grams);
	return 0;
}

int32_t loadcell_get_offset(void)
{
	return tare_offset;
}

void loadcell_set_offset(int32_t offset)
{
	tare_offset = offset;
	LOG_INF("Offset set: %d counts", tare_offset);
}

float loadcell_get_scale(void)
{
	return counts_per_gram;
}
static void countdown(const char *msg, int seconds)
{
	for (int s = seconds; s > 0; s--) {
		LOG_INF("%s (%d s)", msg, s);
		k_sleep(K_SECONDS(1));
	}
}

int loadcell_calibrate_guided(float known_grams, int empty_wait_s,
			      int settle_s, int n)
{
	int err;

	countdown("Remove everything from the load cell", empty_wait_s);

	err = loadcell_tare(n);
	if (err < 0) {
		return err;
	}

	LOG_INF("Place %.1f g on the load cell", (double)known_grams);
	countdown("Waiting for mass to settle", settle_s);

	err = loadcell_calibrate(known_grams, n);
	if (err < 0) {
		return err;
	}

	LOG_INF("Set your scale constant to %.4f counts/g",
		(double)counts_per_gram);
	return 0;
}