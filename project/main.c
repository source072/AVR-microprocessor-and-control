	/*
	 * 260820_estimate_robot_pose_final/main.c
	 *
	 * Created: 2026-08-20 오전 12:53:00
	 * Author : ijaew
  	 
	 */

	#define F_CPU 16000000UL


	#include <avr/io.h>
	#include <math.h>
	#include <stdio.h>
	#include <util/delay.h>
	#include <avr/interrupt.h>

	#include "kinematics.h"
	#include "encoder.h"
	#include "timer.h"
	#include "UART0.h"
	#include "motor.h"
	#include "trajectory.h"
	#include "control.h"
	#include "mpu6050.h"

	uint32_t control_count = 0;
	volatile uint8_t control_flag = 0;
	volatile uint32_t system_ms = 0;

	Pose log_target_pose = {0};
	wheel_velocity log_target_v = {0};
	uint32_t log_control_exec_ms = 0;
	uint32_t last_log_ms = 0;
	volatile uint8_t timer2_cnt = 0;

	ISR(TIMER2_COMP_vect)
	{
		system_ms++;

		timer2_cnt++;

		if (timer2_cnt >= 20)
		{
			timer2_cnt = 0;
			control_flag = 1;
		}
	}

	float normalize_angle_0_2pi(float phi)
	{
		phi = fmodf(phi, 2.0f * M_PI);

		if (phi < 0.0f)
		phi += 2.0f * M_PI;

		return phi;
	}

	int main(){
		//0. config
		uart_init();

		Pose start_pose = {0,0,0}; //while 조건이 car.pose.phi여서, 테스트하고자하는 구간 별로 start_pose도 조정해줘야 함
		Robot car;
		Robot_init(&car, start_pose);

		/* Initialize/calibrate before encoder_init() enables interrupts, so the
		 * software-I2C timing cannot be disturbed. Keep the robot stationary. */
		uint8_t gyro_available = (mpu6050_init(start_pose) == 0U);
		if (gyro_available) {
			mpu6050_calibrate_gyro();
		}

		encoder_init(); 
		motor_io_init();
		pwm_init();
		timer_init();

		Pose goal_pose = {0, 3000, 3000}; //단위 mm,rad

		Pose* Poses = make_goal(start_pose, goal_pose);
		float* total_T = estimate_total_T( Poses,0.5,1000);

		//sei()?
		
		for (int step =0; step<3; step++){
			// step 0~1: first_rotation	// T0
			// step 1~2: translation	// T1
			// step 2~3: final rotation	// T2
		
			//int step= 0;
			control_count = 0;
		
			Pose step_start_pose = Poses[step];
			Pose step_goal_pose = Poses[step + 1]; //정상
			float step_total_T =total_T[step];
	
					while (
					fabs(car.pose.x - Poses[step + 1].x) > 20.0f ||
					fabs(car.pose.y - Poses[step + 1].y) > 20.0f ||
					fabs(car.pose.phi - Poses[step + 1].phi) > 0.02f
					)
					{
						if(control_flag){
							uint32_t control_start_ms = system_ms;
						
							control_flag = 0;
							control_count++;

							float dt = 0.020f;
							float t  = control_count * dt;

							// 현재 pose 추정
							wheel_velocity wheel_v = estimation_wheel_v();
							world_velocity world_v = odometry_update(&car, wheel_v);
							Pose gyro_estimate;

							if (gyro_available) {
								gyro_estimate = gyro();
							} else {
								/* Sensor failure: make fusion reduce to encoder heading. */
								gyro_estimate = car.pose;
								gyro_estimate.phi += dt * world_v.phi_dot;
							}
							
							pose_update(&car, world_v, gyro_estimate);
							
							// 각도 정규화
							car.pose.phi = normalize_angle_0_2pi(car.pose.phi);
					
							// target_pose 생성
							Pose* target_pose = make_target(step_start_pose,step_goal_pose,t,step_total_T);

							//motor 출력 및 제어
							wheel_velocity target_v = control(&car,car.pose,*target_pose);
							motor_set_target(target_v);
						
							// log 출력
							print_pose(&car);	
							
							uint32_t control_end_ms = system_ms;
							log_control_exec_ms = control_end_ms - control_start_ms;
							
							printf("control time : %lu ms\n\n", log_control_exec_ms);
							
					
					}
				}
			
		}
	}


