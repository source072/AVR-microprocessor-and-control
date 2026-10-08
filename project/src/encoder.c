// 260820_estimate_robot_pose_final/encoder.c

/*
==================== 회전 감지 ISR ====================
	0.인터럽트 관련 레지스터
		- EIMSK: 개별 인터럽트 활성화
		- EIFR: 인터럽트 확인 플래그
		- EICRA,EICRB: 인터럽트 기준점 설정
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#include "robot.h"
#include "timer.h"
#include "encoder.h"

#define count_per_rev 330 //모터 회전 한 바퀴당 count 1320번
#define rad_per_count ((2.0*M_PI)/count_per_rev) // count 당 회전 정도 radian

volatile long counter_M1,counter_M2,counter_M3,counter_M4;




//M1
ISR(INT0_vect)
{
	if ((PIND & (1 << PD1)) == 0)
		counter_M1++;
	else
		counter_M1--;
}
/*
ISR(INT1_vect)
{
	if ((PIND & (1 << PD0)) == 0)
	counter_M1--;
	else
	counter_M1++;
}
*/

//M2

ISR(INT2_vect)
{
	if ((PIND & (1 << PD3)) == 0)
	counter_M2--;
	else
	counter_M2++;
}

/*
ISR(INT3_vect)
{
	if ((PIND & (1 << PD2)) == 0)
	counter_M2++;
	else
	counter_M2--;
}
*/

//M3

ISR(INT4_vect)
{
	if ((PINE & (1 << PE5)) == 0)
	counter_M3++;
	else
	counter_M3--;
}
/*
ISR(INT5_vect)
{
	if ((PINE & (1 << PE4)) == 0)
	counter_M3--;
	else
	counter_M3++;
}
*/

//M4

ISR(INT6_vect)
{
	if ((PINE & (1 << PE7)) == 0)
	counter_M4--;
	else
	counter_M4++;
}
/*
ISR(INT7_vect)
{
	
	if ((PINE & (1 << PE6)) == 0)
	counter_M4++;
	else
	counter_M4--;
}
*/




	
void encoder_init(){
  
  counter_M1 = 0;
  counter_M2 = 0;
  counter_M3 = 0;
  counter_M4 = 0;
  
  EICRA &= ~((3 << ISC00) |(3 << ISC10) |(3 << ISC20) | (3 << ISC30));  // EICRA,EICRB: 인터럽트 기준점 설정 레지스터/ INT0~7 rising edge
  EICRA |=  ((3 << ISC00) |(3 << ISC10) |(3 << ISC20) | (3 << ISC30));
  EICRB &= ~((3 << ISC40) |(3 << ISC50) |(3 << ISC60) | (3 << ISC70));
  EICRB |=  ((3 << ISC40) |(3 << ISC50) |(3 << ISC60) | (3 << ISC70));
  
  EIFR = 0xFF;  // EIFR: 인터럽트 확인 플래그 레지스터/ clear interrupt flags
  //EIMSK = 0xFF; // EIMSK: 개별 인터럽트 활성화 레지스터/ interrupt0~7 활성화
  EIMSK = (1 << INT0) | (1 << INT2) |(1 << INT4) |(1 << INT6);
  DDRD = 0x00;  // 포트 입력모드 설정
  sei();
}



wheel_velocity estimation_wheel_v(){
	
	static long prev_counter_M1 =0; // static 변수의 첫 초기값 설정. 초기값 설정은 단 한 번만 작동
	static long prev_counter_M2 =0;
	static long prev_counter_M3 =0;
	static long prev_counter_M4 =0;
	
	cli(); // 전역 interrupt 비활성화
	
	long now_counter_M1 = counter_M1;
	long now_counter_M2 = counter_M2;
	long now_counter_M3 = counter_M3;
	long now_counter_M4 = counter_M4;

	sei(); // 전역 interrupt 활성화
	
	float dt = 20.0f; 

	float u_1 = (float)(now_counter_M1-prev_counter_M1)/dt;
	float u_2 = (float)(now_counter_M2-prev_counter_M2)/dt;
	float u_3 = (float)(now_counter_M3-prev_counter_M3)/dt;
	float u_4 = (float)(now_counter_M4-prev_counter_M4)/dt;
	
	wheel_velocity wheel_v;
	wheel_v.u_L = ((u_1 + u_3) / 2.0f) * rad_per_count * 1000.0f;
	wheel_v.u_R = ((u_2 + u_4) / 2.0f) * rad_per_count * 1000.0f; //1ms에 1count, 즉 ms단위를 s로 변환시켜줘야한다.
	
	prev_counter_M1 = now_counter_M1;
	prev_counter_M2 = now_counter_M2;
	prev_counter_M3 = now_counter_M3;
	prev_counter_M4 = now_counter_M4;
	
	return wheel_v;
}

