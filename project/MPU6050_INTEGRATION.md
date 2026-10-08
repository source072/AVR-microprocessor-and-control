# MPU6050 integration (ATmega128)

## Wiring

The ATmega128 hardware TWI pins PD0/PD1 are already occupied by encoder M1 in
this project. The added driver therefore uses software I2C.

| MPU6050 | ATmega128 | Note |
|---|---|---|
| SCL | PC0 | Add 4.7 kOhm pull-up to 3.3 V |
| SDA | PC1 | Add 4.7 kOhm pull-up to 3.3 V |
| GND | GND | Common ground is required |
| VCC | 3.3 V | For a bare MPU6050; follow the module specification |
| AD0 | GND | Selects address 0x68 |

Do not pull SDA/SCL up to 5 V. Change the pin macros in `src/mpu6050.c` if PC0
or PC1 is used by hardware not visible in this project.

## Startup and tuning

- Keep the robot completely still for about 1.2 seconds after reset. The driver
  uses 500 stationary samples to calculate Z-axis bias.
- If positive robot rotation makes the reported heading decrease, change
  `GYRO_Z_SIGN` in `src/mpu6050.c` from `1.0f` to `-1.0f`.
- The control loop stays at 20 ms (50 Hz). The MPU6050 is configured for a
  200 Hz sample rate, +/-250 deg/s range, and DLPF setting 3.
- `gyro_weight` in `src/kinematics.c` is the gyro-heading share.
  The default 0.98 means 98% gyro and 2% wheel heading per update. Decrease it
  if long-term gyro drift dominates; decrease it if wheel slip dominates.
- If MPU6050 initialization or a sample read fails, pose estimation falls back
  to encoder odometry instead of blocking the control loop.

## Data flow

```c
wheel_velocity wheel_v = estimation_wheel_v();
world_velocity odometry_v = odometry_update(&car, wheel_v);
Pose gyro_pose = gyro();
pose_update(&car, odometry_v, gyro_pose);
```

Only `gyro_pose.phi` is used. Position (`x`, `y`) continues to come from wheel
odometry, while heading (`phi`) is fused from the gyro and wheel odometry.
