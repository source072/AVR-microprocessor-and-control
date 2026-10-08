// 260820_estimate_robot_pose_final/motor.c

#include <avr/io.h>
#include <math.h>
#include "motor.h"
#include "robot.h"

#define M1_IN1 PF0 // Motor 1
#define M1_IN2 PF1
#define M2_IN1 PF2 // Motor 2
#define M2_IN2 PF3
#define M3_IN1 PF4 // Motor 3
#define M3_IN2 PF5
#define M4_IN1 PF6 // Motor 4 
#define M4_IN2 PF7

#define PWM_OFFSET_M1   30
#define PWM_OFFSET_M2	0
#define PWM_OFFSET_M3   15
#define PWM_OFFSET_M4   0

void disable_jtag(void)
{
	#ifdef JTD
	MCUCSR |= (1 << JTD);
	MCUCSR |= (1 << JTD);
	#endif
}

void motor_io_init(void)
{
	disable_jtag();

	// PF0~PF7 : L298N IN 핀 출력
	DDRF = 0xFF;
	PORTF = 0x00;

	// PB4~PB7 : PWM 출력
	DDRB |= (1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7);

	// PWM 초기 duty 0
	OCR0  = 0;
	OCR1A = 0;
	OCR1B = 0;
	OCR1C = 0;
}

void pwm_init(void)
{
	//timer0 init
	//ASSR &= ~(1 << AS0); 
	
	TCCR0 = 0x00;
	TCNT0 = 0;

	TCCR0 |= (1 << WGM00) | (1 << WGM01);			// Fast PWM
	TCCR0 |= (1 << COM01);							// Non-inverting PWM
	TCCR0 |= (1<<CS02 |0 << CS01) | (0 << CS00);    // Prescaler 64 <-- 100이 64임. 011은 32다..

	//timer1 init
	TCCR1A = 0x00;
	TCCR1B = 0x00;
	TCNT1 = 0;
	
	TCCR1A |= (1 << WGM10); // 8-bit Fast PWM mode: WGM13:0 = 0101
	TCCR1B |= (1 << WGM12);
	
	TCCR1A |= (1 << COM1A1); // Non-inverting PWM on OC1A, OC1B, OC1C
	TCCR1A |= (1 << COM1B1);
	TCCR1A |= (1 << COM1C1);
	
	TCCR1B |= (1 << CS11) | (1 << CS10);// Prescaler 64
}

static uint8_t abs_speed(int16_t speed)
{
	if (speed > 250) speed = 250;
	if (speed < -250) speed = -250;

	if (speed < 0) return (uint8_t)(-speed);
	return (uint8_t)speed;
}

void motor_direction(uint8_t motor, int16_t speed)
{
	switch (motor)
	{
		case 1:
		PORTF &= ~((1 << M1_IN1) | (1 << M1_IN2));

		if (speed > 0)
		PORTF |= (1 << M1_IN1);
		else if (speed < 0)
		PORTF |= (1 << M1_IN2);
		break;

		case 2:
		PORTF &= ~((1 << M2_IN1) | (1 << M2_IN2));

		if (speed > 0)
		PORTF |= (1 << M2_IN2);
		else if (speed < 0)
		PORTF |= (1 << M2_IN1);
		break;

		case 3:
		PORTF &= ~((1 << M3_IN1) | (1 << M3_IN2));

		if (speed > 0)
		PORTF |= (1 << M3_IN1);
		else if (speed < 0)
		PORTF |= (1 << M3_IN2);
		break;

		case 4:
		PORTF &= ~((1 << M4_IN1) | (1 << M4_IN2));

		if (speed > 0)
		PORTF |= (1 << M4_IN1);
		else if (speed < 0)
		PORTF |= (1 << M4_IN2);
		break;
	}
}

void motor_pwm(uint8_t motor, uint8_t duty)
{
	switch (motor)
	{
		case 1:
		OCR0 = duty + PWM_OFFSET_M1;
		break;

		case 2:
		OCR1A = duty + PWM_OFFSET_M2;
		break;

		case 3:
		OCR1B = duty + PWM_OFFSET_M3;
		break;

		case 4:
		OCR1C = duty + PWM_OFFSET_M4;;
		break;
	}
}

//모터 별 속도 설정(u_1,2,3,4)
void motor_set(uint8_t motor, int16_t speed)
{
	uint8_t duty = abs_speed(speed);

	motor_direction(motor, speed);
	motor_pwm(motor, duty);
}


void motors_set_all(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
	// 방향 설정
	motor_direction(1, m1);
	motor_direction(2, m2);
	motor_direction(3, m3);
	motor_direction(4, m4);

	// PWM duty 설정
    motor_pwm(1, abs_speed(m1));
    motor_pwm(2, abs_speed(m2));
    motor_pwm(3, abs_speed(m3));
    motor_pwm(4, abs_speed(m4));
}

void motors_stop_all(void)
{
	OCR0  = 0;
	OCR1A = 0;
	OCR1B = 0;
	OCR1C = 0;

	PORTF = 0x00;
}


static int16_t velocity_to_pwm_left(float u_L)
{
	if (u_L == 0.0f)
	return 0;

	float pwm =
	 5.556f * fabsf(u_L)
	 + 60.0f;

	if (u_L < 0.0f)
	pwm = -pwm;

	return (int16_t)pwm;
}


static int16_t velocity_to_pwm_right(float u_R)
{
	if (u_R == 0.0f)
	return 0;

	float pwm =
	5.556f * fabsf(u_R)
	+ 60.04f;

	if (u_R < 0.0f)
	pwm = -pwm;

	return (int16_t)pwm;
}

void motor_set_target(wheel_velocity target_v)
{
	int16_t pwm_L =
	velocity_to_pwm_left(target_v.u_L);

	int16_t pwm_R =
	velocity_to_pwm_right(target_v.u_R);

	motors_set_all(
	pwm_L,    // M1
	pwm_R,    // M2
	pwm_L,    // M3
	pwm_R     // M4
	);
}