// 260820_estimate_robot_pose_final/timer.c

#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"

volatile long timer = 0;


void timer_init(void)
{
	
	TCCR2 = 0x00; // Timer2 정지
	TCNT2 = 0; // 카운터 초기화
	OCR2 = 249; // count 목표
	TCCR2 |= (1 << WGM21);// CTC Mode
	TIMSK |= (1 << OCIE2); // interrupt 활성화
	TCCR2 |= (1 << CS22); // Prescaler = 64, 1ms마다 overflow 발생
	
}

