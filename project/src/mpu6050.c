#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>

#include "mpu6050.h"

#define MPU6050_ADDRESS       0x68U
#define MPU6050_SMPLRT_DIV    0x19U
#define MPU6050_CONFIG        0x1AU
#define MPU6050_GYRO_CONFIG   0x1BU
#define MPU6050_GYRO_XOUT_H   0x43U
#define MPU6050_PWR_MGMT_1    0x6BU
#define MPU6050_WHO_AM_I      0x75U

#define GYRO_LSB_PER_DPS      32.8f
#define DEG_TO_RAD            0.01745329252f
#define CONTROL_DT_S          0.020f
#define CALIBRATION_SAMPLES   500U
#define GYRO_Z_SIGN           1.0f

#define I2C_PORT              PORTC
#define I2C_PIN               PINC
#define I2C_DDR               DDRC
#define I2C_SCL               PC0
#define I2C_SDA               PC1

static float gyro_z_bias;
static Pose gyro_pose;

/* Open-drain software I2C.  Releasing the line makes the external pull-up
 * drive it high; driving the DDR bit low pulls it down. */
static void scl_low(void)     { I2C_PORT &= ~(1U << I2C_SCL); I2C_DDR |= (1U << I2C_SCL); }
static void scl_release(void) { I2C_DDR &= ~(1U << I2C_SCL); }
static void sda_low(void)     { I2C_PORT &= ~(1U << I2C_SDA); I2C_DDR |= (1U << I2C_SDA); }
static void sda_release(void) { I2C_DDR &= ~(1U << I2C_SDA); }

static void i2c_delay(void) { _delay_us(5); }

static void i2c_begin(void)
{
	sda_release(); scl_release(); i2c_delay();
	sda_low(); i2c_delay();
	scl_low();
}

static void i2c_stop(void)
{
	sda_low(); i2c_delay();
	scl_release(); i2c_delay();
	sda_release(); i2c_delay();
}

static uint8_t i2c_write(uint8_t data)
{
	uint8_t mask;
	uint8_t nack;

	for (mask = 0x80U; mask != 0U; mask >>= 1) {
		if (data & mask) sda_release(); else sda_low();
		i2c_delay(); scl_release(); i2c_delay(); scl_low();
	}
	sda_release(); i2c_delay();
	scl_release(); i2c_delay();
	nack = (I2C_PIN & (1U << I2C_SDA)) ? 1U : 0U;
	scl_low();
	return nack;
}

static uint8_t i2c_read(uint8_t ack)
{
	uint8_t i;
	uint8_t data = 0U;

	sda_release();
	for (i = 0U; i < 8U; ++i) {
		data <<= 1;
		scl_release(); i2c_delay();
		if (I2C_PIN & (1U << I2C_SDA)) data |= 1U;
		scl_low(); i2c_delay();
	}
	if (ack) sda_low(); else sda_release();
	scl_release(); i2c_delay(); scl_low();
	sda_release();
	return data;
}

static uint8_t write_register(uint8_t reg, uint8_t value)
{
	uint8_t error;
	i2c_begin();
	error = i2c_write((MPU6050_ADDRESS << 1) | 0U);
	if (!error) error = i2c_write(reg);
	if (!error) error = i2c_write(value);
	i2c_stop();
	return error;
}

static uint8_t read_register(uint8_t reg, uint8_t *value)
{
	i2c_begin();
	if (i2c_write((MPU6050_ADDRESS << 1) | 0U)) { i2c_stop(); return 1U; }
	if (i2c_write(reg)) { i2c_stop(); return 1U; }
	i2c_begin();
	if (i2c_write((MPU6050_ADDRESS << 1) | 1U)) { i2c_stop(); return 1U; }
	*value = i2c_read(0U);
	i2c_stop();
	return 0U;
}

static int16_t read_gyro_z_raw(void)
{
	uint8_t high;
	uint8_t low;

	i2c_begin();
	if (i2c_write((MPU6050_ADDRESS << 1) | 0U)) { i2c_stop(); return 0; }
	if (i2c_write(MPU6050_GYRO_XOUT_H + 4U)) { i2c_stop(); return 0; }
	i2c_begin();
	if (i2c_write((MPU6050_ADDRESS << 1) | 1U)) { i2c_stop(); return 0; }
	high = i2c_read(1U);
	low = i2c_read(0U);
	i2c_stop();
	return (int16_t)(((uint16_t)high << 8) | low);
}

uint8_t mpu6050_init(Pose initial_pose)
{
	uint8_t who_am_i = 0U;

	/* PC0/PC1 avoid the PD0/PD1 encoder conflict in this robot project. */
	I2C_PORT &= ~((1U << I2C_SCL) | (1U << I2C_SDA));
	scl_release();
	sda_release();

	_delay_ms(100);
	
	if (read_register(MPU6050_WHO_AM_I, &who_am_i)) {
		return 1U;
	}

	if ((who_am_i != 0x68U) &&
	(who_am_i != 0x70U)) {
		return 1U;
	}
	
	if (write_register(MPU6050_PWR_MGMT_1, 0x01U)) return 1U;
	if (write_register(MPU6050_CONFIG, 0x03U)) return 1U;      /* 44 Hz DLPF */
	if (write_register(MPU6050_SMPLRT_DIV, 0x04U)) return 1U;  /* 200 Hz */
	if (write_register(MPU6050_GYRO_CONFIG, 0x00U)) return 1U; /* +/-250 dps */

	gyro_pose = initial_pose;
	gyro_z_bias = 0.0f;
	_delay_ms(50);
	return 0U;
}

void mpu6050_calibrate_gyro(void)
{
	uint16_t i;
	int32_t sum = 0;

	for (i = 0U; i < CALIBRATION_SAMPLES; ++i) {
		sum += read_gyro_z_raw();
		_delay_ms(2);
	}
	gyro_z_bias = (float)sum / (float)CALIBRATION_SAMPLES;
}

Pose gyro(void)
{
	float z_dps = GYRO_Z_SIGN * ((float)read_gyro_z_raw() - gyro_z_bias) / GYRO_LSB_PER_DPS;
	gyro_pose.phi += z_dps * DEG_TO_RAD * CONTROL_DT_S;

	while (gyro_pose.phi > M_PI) gyro_pose.phi -= 2.0f * M_PI;
	while (gyro_pose.phi < -M_PI) gyro_pose.phi += 2.0f * M_PI;
	return gyro_pose;
}
