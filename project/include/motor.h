// 260820_estimate_robot_pose_final/UART0.h

#ifndef MOTOR_H
#define MOTOR_H

//function
void disable_jtag(void);
void motor_io_init(void);
void pwm_init(void);


void motor_direction(uint8_t motor, int16_t speed);
void motor_pwm(uint8_t motor, uint8_t duty);

void motor_set(uint8_t motor, int16_t speed);
void motors_set_all(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
void motors_stop_all(void);

void motor_set_target();

#endif