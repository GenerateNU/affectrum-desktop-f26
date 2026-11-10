#ifndef FOC_H
#define FOC_H

/**
 * @brief Duty cycle for each of the three phases.
 *
 * 0.0 holds the phase at 0 V for the whole PWM period and 1.0 holds it at VM.
 * The average voltage on a phase is its duty times VM.
 */
struct foc_duty {
	float a; /**< Phase A duty, 0.0 to 1.0 */
	float b; /**< Phase B duty, 0.0 to 1.0 */
	float c; /**< Phase C duty, 0.0 to 1.0 */
};

/**
 * @brief Turns the voltage you want (seen from the rotor) into three phase duty cycles.
 *
 * Runs inverse Park, inverse Clarke, then SVPWM centering. If the request is bigger than the supply
 * can make (vbus / sqrt(3)), it is scaled down while keeping its direction.
 *
 * @param vd Voltage along the rotor magnet (d axis), in volts. Makes no torque; keep it 0 except for alignment.
 * @param vq Voltage 90 electrical degrees ahead of the rotor magnet (q axis), in volts. This makes torque;
 *           negative values push the other way.
 * @param theta_e Electrical angle of the rotor in radians (pole pairs x mechanical angle).
 * @param vbus Motor supply voltage (VM) in volts.
 * @param out Where to store the three duty cycles.
 */
void foc_set_voltage(float vd, float vq, float theta_e, float vbus, struct foc_duty *out);

/**
 * @brief Spins the motor open loop: no sensor, just a rotating field the rotor follows.
 *
 * Calls motor_init() and motor_foc_enable(), then starts a control thread that moves the electrical
 * angle forward at a fixed rate. Current flows the whole time, even if the rotor is not moving, so
 * the motor gets warm. Use a current-limited supply and don't leave it running.
 *
 * @param vq Voltage to apply on the q axis, in volts. Start with about phase resistance x 0.3 A.
 * @param elec_hz Electrical revolutions per second. The rotor turns at elec_hz / pole pairs revolutions
 *                per second. Negative values spin the other way; 0 holds the rotor in place.
 * @return 0 on success, -EALREADY if the loop is already running, or a negative error code from motor setup.
 */
int foc_open_loop_start(float vq, float elec_hz);

/**
 * @brief Stops the control loop. All phases go back to 50% duty, so no current flows.
 */
void foc_stop(void);

#endif
