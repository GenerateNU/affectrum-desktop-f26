#ifndef MOTORS_H
#define MOTORS_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Wakes the DRV8311P and sets up its internal PWM generator over SPI. All phases start off.
 *
 * @return 0 on success, or a negative error code from Zephyr if something failed.
 */
int motor_init(void);

/**
 * @brief Sets the duty cycle used by the phase that is switching during 6-step commutation.
 *
 * @param percent 0 = no power, 100 = full VM. Values above 100 are treated as 100.
 * @return 0 on success, or -EIO if an SPI write failed.
 */
int motor_set_speed(uint8_t percent);

/**
 * @brief Moves to the next of the 6 commutation steps. Calling this over and over is what makes the rotor turn.
 *
 * @param reverse true steps backward (the rotor turns the other way), false steps forward.
 * @return 0 on success, or a negative error code if the SPI write failed.
 */
int motor_step(bool reverse);

/**
 * @brief Turns on all three low-side MOSFETs, which shorts the windings and stops the rotor quickly.
 *
 * @return 0 on success, or a negative error code if the SPI write failed.
 */
int motor_brake(void);

/**
 * @brief Checks the driver for latched faults.
 *
 * @return The DEV_STS1 register (0 means no faults), or a negative error code if the SPI read failed.
 */
int motor_read_faults(void);

/**
 * @brief Switches all three phases to complementary PWM, which FOC needs.
 *
 * In this mode each phase's high-side and low-side MOSFETs take turns, so the phase can sit at any
 * average voltage between 0 V and VM. The duties are set to 50% first so all three phases start at
 * the same voltage and no current flows until you call motor_set_duties().
 *
 * @return 0 on success, or -EIO if an SPI write failed.
 */
int motor_foc_enable(void);

/**
 * @brief Sets each phase's duty cycle. Used by FOC once motor_foc_enable() has been called.
 *
 * The average voltage on a phase is its duty times VM.
 *
 * @param a Phase A duty, 0.0 (0 V) to 1.0 (VM). Values outside that range are clamped.
 * @param b Phase B duty, 0.0 to 1.0.
 * @param c Phase C duty, 0.0 to 1.0.
 * @return 0 on success, or -EIO if an SPI write failed.
 */
int motor_set_duties(float a, float b, float c);

#endif
