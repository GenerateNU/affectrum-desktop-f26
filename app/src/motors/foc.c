#include <math.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "foc.h"
#include "motors.h"

LOG_MODULE_REGISTER(foc, LOG_LEVEL_INF);

#define TWO_PI         6.2831853f
#define SQRT3_OVER_2   0.8660254f
#define ONE_OVER_SQRT3 0.5773503f

// Motor supply voltage (VM). Measure yours: the duty math assumes this is right.
#define FOC_VBUS_VOLTS 12.0f

// How many times per second the control loop runs. Each loop sends 3 SPI frames to the DRV8311P,
// which is what limits how fast this can go.
#define FOC_LOOP_HZ 1000

// Runs ahead of main and the BLE app code, but behind the Bluetooth controller (lower number = higher priority)
#define FOC_THREAD_PRIORITY 2
#define FOC_STACK_SIZE      2048

K_THREAD_STACK_DEFINE(foc_stack, FOC_STACK_SIZE);
static struct k_thread foc_thread_data;
K_TIMER_DEFINE(foc_timer, NULL, NULL);

// set by foc_open_loop_start(), read by the control thread
static float open_loop_vq;
static float open_loop_elec_hz;
static atomic_t running;

void foc_set_voltage(float vd, float vq, float theta_e, float vbus, struct foc_duty *out)
{
	// SVPWM can make at most vbus/sqrt(3) in any direction. Shrink the request if it asks for more,
	// keeping its direction so the field still points the right way.
	float v_limit = vbus * ONE_OVER_SQRT3;
	float v_mag = sqrtf(vd * vd + vq * vq);
	if (v_mag > v_limit) {
		vd *= v_limit / v_mag;
		vq *= v_limit / v_mag;
	}

	// inverse Park: rotor frame (d, q) -> stator frame (alpha, beta)
	float s = sinf(theta_e);
	float c = cosf(theta_e);
	float v_alpha = vd * c - vq * s;
	float v_beta = vd * s + vq * c;

	// inverse Clarke: x/y -> three phases 120 degrees apart
	float va = v_alpha;
	float vb = -0.5f * v_alpha + SQRT3_OVER_2 * v_beta;
	float vc = -0.5f * v_alpha - SQRT3_OVER_2 * v_beta;

	// SVPWM: the motor only feels the differences between phases, so shift all three
	// until the highest and lowest sit centered in 0..vbus
	float vmid = (fmaxf(va, fmaxf(vb, vc)) + fminf(va, fminf(vb, vc))) * 0.5f;
	out->a = (va - vmid) / vbus + 0.5f;
	out->b = (vb - vmid) / vbus + 0.5f;
	out->c = (vc - vmid) / vbus + 0.5f;
}

/**
 * @brief Control thread for open-loop spinning. Runs once per FOC_LOOP_HZ tick until foc_stop() is called.
 *
 * @param p1 Unused (Zephyr threads always take three arguments).
 * @param p2 Unused.
 * @param p3 Unused.
 */
static void open_loop_thread(void *p1, void *p2, void *p3)
{
	const float dt = 1.0f / FOC_LOOP_HZ;
	float theta_e = 0.0f;
	struct foc_duty duty;

	k_timer_start(&foc_timer, K_USEC(1000000 / FOC_LOOP_HZ), K_USEC(1000000 / FOC_LOOP_HZ));

	while (atomic_get(&running)) {
		// sleep until the next tick so every loop is the same length apart
		k_timer_status_sync(&foc_timer);

		// No sensor yet, so make up the angle: move it forward a little every tick and the rotor follows
		theta_e += TWO_PI * open_loop_elec_hz * dt;
		if (theta_e >= TWO_PI) {
			theta_e -= TWO_PI;
		} else if (theta_e < 0.0f) {
			theta_e += TWO_PI;
		}

		foc_set_voltage(0.0f, open_loop_vq, theta_e, FOC_VBUS_VOLTS, &duty);
		if (motor_set_duties(duty.a, duty.b, duty.c)) {
			LOG_ERR("Duty write failed, stopping");
			break;
		}

		int faults = motor_read_faults();
		if (faults != 0) {
			LOG_ERR("DRV8311 fault (DEV_STS1 = 0x%03x), stopping", faults);
			break;
		}
	}

	k_timer_stop(&foc_timer);
	motor_set_duties(0.5f, 0.5f, 0.5f);
	atomic_set(&running, false);
}

int foc_open_loop_start(float vq, float elec_hz)
{
	if (atomic_get(&running)) {
		return -EALREADY;
	}

	int err = motor_init();
	if (err) {
		return err;
	}
	err = motor_foc_enable();
	if (err) {
		return err;
	}

	open_loop_vq = vq;
	open_loop_elec_hz = elec_hz;
	atomic_set(&running, true);

	// K_FP_REGS: this thread does float math, so Zephyr must save the FPU registers when switching away from it
	k_thread_create(&foc_thread_data, foc_stack, K_THREAD_STACK_SIZEOF(foc_stack), open_loop_thread,
			NULL, NULL, NULL, FOC_THREAD_PRIORITY, K_FP_REGS, K_NO_WAIT);
	k_thread_name_set(&foc_thread_data, "foc");
	return 0;
}

void foc_stop(void)
{
	if (!atomic_get(&running)) {
		return;
	}
	atomic_set(&running, false);
	// wait for the thread to finish its last loop and park the phases at 50%
	k_thread_join(&foc_thread_data, K_FOREVER);
}
