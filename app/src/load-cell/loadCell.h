#ifndef LOADCELL_H_
#define LOADCELL_H_

#include <stdint.h>

/**
 * Initialize the ADC channel. Call once before anything else.
 * Returns 0 on success, negative errno on failure.
 */
int loadcell_init(void);

/**
 * Read the averaged raw ADC value (signed counts).
 * @param n    number of readings to average (>= 1)
 * @param out  averaged raw counts
 */
int loadcell_read_raw(int n, int32_t *out);

/**
 * Zero the scale. Call with nothing on the load cell.
 * @param n  number of readings to average (64 is a good default)
 */
int loadcell_tare(int n);

/**
 * Set the calibration factor directly (counts per gram).
 */
void loadcell_set_scale(float counts_per_gram);

/**
 * Calibrate using a known mass currently on the load cell.
 * Requires loadcell_tare() to have been called first.
 * @param known_grams  mass on the cell, in grams (> 0)
 * @param n            number of readings to average
 */
int loadcell_calibrate(float known_grams, int n);

/**
 * Read the weight in grams (uses the tare offset and scale factor).
 */
int loadcell_read_grams(int n, float *grams);

/** Get/set the tare offset, e.g. to persist it across reboots. */
int32_t loadcell_get_offset(void);
void loadcell_set_offset(int32_t offset);

/** Get the current calibration factor (counts per gram). */
float loadcell_get_scale(void);

/**
 * Guided calibration for bring-up: logs prompts, tares, waits for you to
 * place the known mass, then calibrates. Blocks for roughly
 * (empty_wait_s + settle_s) seconds plus sampling time.
 *
 * @param known_grams   mass you will place on the cell
 * @param empty_wait_s  seconds to wait for the cell to be emptied first
 * @param settle_s      seconds to wait for the mass to settle
 * @param n             samples to average for tare and calibration
 */
int loadcell_calibrate_guided(float known_grams, int empty_wait_s,
			      int settle_s, int n);

#endif /* LOADCELL_H_ */