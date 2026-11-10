#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include "motors.h"

LOG_MODULE_REGISTER(motor, LOG_LEVEL_INF);

// the drv8311 node is defined in boards/nrf52833dk_nrf52833.overlay
#define DRV8311 DT_NODELABEL(drv8311)

// How this chip wants to be talked to (DRV8311 datasheet, section 8.5.1):
//   SPI_OP_MODE_MASTER: the nRF drives the clock and chip select
//   SPI_MODE_CPHA:      "SPI mode 1": clock idles low, the chip reads SDI on the falling edge
//   SPI_WORD_SET(8):    move bytes; one 32-bit tSPI frame is 4 bytes sent while CS stays low
//   SPI_TRANSFER_MSB:   most significant bit goes out first
#define DRV8311_SPI_OP (SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

static const struct spi_dt_spec drv_spi = SPI_DT_SPEC_GET(DRV8311, DRV8311_SPI_OP);
static const struct pwm_dt_spec sync_pwm = PWM_DT_SPEC_GET(DRV8311);
static const struct gpio_dt_spec sleep_pin = GPIO_DT_SPEC_GET(DRV8311, sleep_gpios);
static const struct gpio_dt_spec fault_pin = GPIO_DT_SPEC_GET(DRV8311, fault_gpios);

// tSPI lets up to 4 DRV8311Ps share one chip select; each answers only to its own ID (set by its AD1/AD0 pins)
#define DRV8311_TSPI_ID 0

// Every tSPI frame is 32 bits:
//   [31] 1 = read / 0 = write, [30:27] device ID, [26:19] register address,
//   [18:17] always 0, [16] header parity, [15] data parity, [14:0] data
// Parity checking is off until you turn it on (SYS_CTRL.SPI_PEN), so the parity bits are left at 0.
#define TSPI_READ      BIT(31)
#define TSPI_ID(id)    ((uint32_t)((id) & 0xF) << 27)
#define TSPI_ADDR(reg) ((uint32_t)((reg) & 0xFF) << 19)
#define TSPI_DATA_MASK 0x7FFF

// register addresses (datasheet section 9)
#define REG_DEV_STS1    0x00
#define REG_FLT_CLR     0x17
#define REG_PWMG_PERIOD 0x18
#define REG_PWMG_A_DUTY 0x19
#define REG_PWMG_B_DUTY 0x1A
#define REG_PWMG_C_DUTY 0x1B
#define REG_PWM_STATE   0x1C
#define REG_PWMG_CTRL   0x1D

// PWMG_CTRL fields
#define PWMG_CTRL_PWM_EN       BIT(10)    // turn the internal PWM generator on
#define PWMG_CTRL_OSC_SYNC_20K (0x5 << 5) // trim the internal 20 MHz clock using our 20 kHz PWM_SYNC signal

// In up/down counting mode, PWM frequency = 20 MHz / (2 * period), so 500 counts gives 20 kHz
#define PWMG_PERIOD_COUNTS 500

// PWM_STATE holds a 3-bit setting for each phase: A in bits 2-0, B in bits 6-4, C in bits 10-8
#define PHASE_OFF 0x0 // both MOSFETs off: the phase floats
#define PHASE_LOW 0x1 // low-side MOSFET on: the phase is tied to ground
#define PHASE_PWM 0x6 // high-side MOSFET switches at the duty cycle: the phase is pulled toward VM
#define PHASE_SYNC 0x7 // high side switches at the duty cycle and low side does the opposite: the phase can sit at any voltage (FOC)
#define PHASES(a, b, c) (((c) << 8) | ((b) << 4) | (a))

// 6-step commutation: each step pushes current into one phase and out of another,
// and each step turns the magnetic field 60 electrical degrees. The rotor follows the field.
static const uint16_t six_step[6] = {
	PHASES(PHASE_PWM, PHASE_LOW, PHASE_OFF), // A -> B
	PHASES(PHASE_PWM, PHASE_OFF, PHASE_LOW), // A -> C
	PHASES(PHASE_OFF, PHASE_PWM, PHASE_LOW), // B -> C
	PHASES(PHASE_LOW, PHASE_PWM, PHASE_OFF), // B -> A
	PHASES(PHASE_LOW, PHASE_OFF, PHASE_PWM), // C -> A
	PHASES(PHASE_OFF, PHASE_LOW, PHASE_PWM), // C -> B
};

static int step_index;

/**
 * @brief Sends one 32-bit tSPI frame to the DRV8311P and collects its reply.
 *
 * @param frame The full frame: read/write bit, device ID, register address, and data (see TSPI_* above).
 * @param reply Where to store the 15 data bits the chip sends back, or NULL to ignore them.
 * @return 0 on success, or a negative error code from the SPI driver.
 */
static int drv8311_transfer(uint32_t frame, uint16_t *reply)
{
	// SPI is full duplex: while tx shifts out on MOSI, rx shifts in on MISO, one bit per clock
	uint8_t tx[4] = {frame >> 24, frame >> 16, frame >> 8, frame};
	uint8_t rx[4] = {0};

	const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	const struct spi_buf rx_buf = {.buf = rx, .len = sizeof(rx)};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

	// pulls CS low, clocks out all 32 bits, then releases CS
	int err = spi_transceive_dt(&drv_spi, &tx_set, &rx_set);
	if (err) {
		return err;
	}

	// rx[0] arrives while the chip is still reading our header, rx[1] is its status byte,
	// and rx[2..3] hold the register contents
	if (reply != NULL) {
		*reply = ((rx[2] << 8) | rx[3]) & TSPI_DATA_MASK;
	}
	return 0;
}

/**
 * @brief Reads one DRV8311P register.
 *
 * @param reg Register address (one of the REG_* defines).
 * @param value Where to store the register's 15 data bits.
 * @return 0 on success, or a negative error code from the SPI driver.
 */
static int drv8311_read(uint8_t reg, uint16_t *value)
{
	return drv8311_transfer(TSPI_READ | TSPI_ID(DRV8311_TSPI_ID) | TSPI_ADDR(reg), value);
}

/**
 * @brief Writes one DRV8311P register.
 *
 * @param reg Register address (one of the REG_* defines).
 * @param value The 15 data bits to write. Bit 15 is dropped because it is the parity bit.
 * @return 0 on success, or a negative error code from the SPI driver.
 */
static int drv8311_write(uint8_t reg, uint16_t value)
{
	return drv8311_transfer(TSPI_ID(DRV8311_TSPI_ID) | TSPI_ADDR(reg) | (value & TSPI_DATA_MASK), NULL);
}

int motor_init(void)
{
	if (!spi_is_ready_dt(&drv_spi) || !pwm_is_ready_dt(&sync_pwm) ||
	    !gpio_is_ready_dt(&sleep_pin) || !gpio_is_ready_dt(&fault_pin)) {
		LOG_ERR("Motor hardware not ready");
		return -ENODEV;
	}

	int err = gpio_pin_configure_dt(&fault_pin, GPIO_INPUT);
	if (err) {
		return err;
	}

	// Start the 20 kHz, 50% duty PWM_SYNC signal. This is the only PWM the nRF produces;
	// the DRV8311P makes the real motor PWM itself.
	err = pwm_set_pulse_dt(&sync_pwm, sync_pwm.period / 2);
	if (err) {
		LOG_ERR("Failed to start PWM_SYNC (%d)", err);
		return err;
	}

	// Wake the chip (nSLEEP high). Its registers reset to defaults every time it wakes, so configure after this.
	err = gpio_pin_configure_dt(&sleep_pin, GPIO_OUTPUT_INACTIVE);
	if (err) {
		return err;
	}
	k_msleep(5); // datasheet t_WAKE is up to 3 ms

	// First SPI read: right after power-up the RESET bit (bit 7) should be set
	uint16_t status;
	err = drv8311_read(REG_DEV_STS1, &status);
	if (err) {
		LOG_ERR("SPI read failed (%d)", err);
		return err;
	}
	LOG_INF("DEV_STS1 = 0x%03x", status);

	// Clear the power-up flag and any other latched faults
	err = drv8311_write(REG_FLT_CLR, 1);

	// All phases off, 20 kHz period, 0% duty, then turn the PWM generator on
	err |= drv8311_write(REG_PWM_STATE, PHASES(PHASE_OFF, PHASE_OFF, PHASE_OFF));
	err |= drv8311_write(REG_PWMG_PERIOD, PWMG_PERIOD_COUNTS);
	err |= motor_set_speed(0);
	err |= drv8311_write(REG_PWMG_CTRL, PWMG_CTRL_PWM_EN | PWMG_CTRL_OSC_SYNC_20K);
	if (err) {
		LOG_ERR("SPI write failed");
		return -EIO;
	}

	// Read a register back to prove the SPI link works in both directions
	uint16_t period = 0;
	err = drv8311_read(REG_PWMG_PERIOD, &period);
	if (err || period != PWMG_PERIOD_COUNTS) {
		LOG_ERR("PWMG_PERIOD reads %u, expected %u; check SPI wiring, AD0/AD1, and that VM is powered",
			period, PWMG_PERIOD_COUNTS);
		return -EIO;
	}

	LOG_INF("DRV8311P ready");
	return 0;
}

int motor_set_speed(uint8_t percent)
{
	if (percent > 100) {
		percent = 100;
	}

	// Each phase compares its duty value against the PWM counter, so the same number works for all three.
	// Only the phase set to PHASE_PWM in the current step actually uses it.
	uint16_t duty = (PWMG_PERIOD_COUNTS * percent) / 100;

	int err = drv8311_write(REG_PWMG_A_DUTY, duty);
	err |= drv8311_write(REG_PWMG_B_DUTY, duty);
	err |= drv8311_write(REG_PWMG_C_DUTY, duty);
	return err ? -EIO : 0;
}

int motor_step(bool reverse)
{
	step_index = reverse ? (step_index + 5) % 6 : (step_index + 1) % 6;
	return drv8311_write(REG_PWM_STATE, six_step[step_index]);
}

int motor_brake(void)
{
	return drv8311_write(REG_PWM_STATE, PHASES(PHASE_LOW, PHASE_LOW, PHASE_LOW));
}

int motor_read_faults(void)
{
	// cheap check first: nFAULT is only pulled low when something is latched
	if (gpio_pin_get_dt(&fault_pin) == 0) {
		return 0;
	}

	uint16_t status;
	int err = drv8311_read(REG_DEV_STS1, &status);
	if (err) {
		return err;
	}
	return status;
}

/**
 * @brief Turns a 0.0-1.0 duty into PWMG_x_DUTY counts.
 *
 * @param duty Fraction of the PWM period the high side should be on. Clamped to 0.0-1.0.
 * @return Counts from 0 (always low) to PWMG_PERIOD_COUNTS (always high).
 */
static uint16_t duty_to_counts(float duty)
{
	duty = fminf(fmaxf(duty, 0.0f), 1.0f);
	// + 0.5f rounds to the nearest count instead of always rounding down
	return (uint16_t)(duty * PWMG_PERIOD_COUNTS + 0.5f);
}

int motor_foc_enable(void)
{
	// 50% on every phase puts all three at VM/2: no voltage difference, so no current
	int err = motor_set_duties(0.5f, 0.5f, 0.5f);
	if (err) {
		return err;
	}
	return drv8311_write(REG_PWM_STATE, PHASES(PHASE_SYNC, PHASE_SYNC, PHASE_SYNC)) ? -EIO : 0;
}

int motor_set_duties(float a, float b, float c)
{
	int err = drv8311_write(REG_PWMG_A_DUTY, duty_to_counts(a));
	err |= drv8311_write(REG_PWMG_B_DUTY, duty_to_counts(b));
	err |= drv8311_write(REG_PWMG_C_DUTY, duty_to_counts(c));
	return err ? -EIO : 0;
}
