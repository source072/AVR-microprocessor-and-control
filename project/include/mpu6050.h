#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "robot.h"

/* Returns 0 when WHO_AM_I and device configuration succeed. */
uint8_t mpu6050_init(Pose initial_pose);

/* Calibrate while the robot is completely stationary. */
void mpu6050_calibrate_gyro(void);

/* Integrate the Z gyro for one 20 ms control period and return it as Pose. */
Pose gyro(void);

#endif
