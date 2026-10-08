// 260820_estimate_robot_pose_final/UART0.c

#include <avr/io.h>
#include <stdio.h>
#include "robot.h"
#include "UART0.h"

int uart_putchar(char c, FILE *stream)
{
	
	if (c == '\n')
	uart_putchar('\r', stream);
	
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;

	return 0;
}

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void uart_init(){
	    UCSR0A = 0x00;   /* normal speed mode */
		//UCSR0A = (1 << U2X0); /*double speed mode*/
	    UCSR0B = 0x18;   /* RX enable, TX enable */
	    UCSR0C = 0x06;   /* 8 data bits, no parity, 1 stop bit */
	    UBRR0H = 0x00;
	    UBRR0L = 0x67;   /* F_CPU: 16 MHz,baud_rate : 9600 bps */
		//UBRR0L = 0x10;		/*115200 bps*/
		//UBRR0L = 34;     // 57600 bps
		
		
		stdout = &uart_output;
}

void print_pose(Robot* robot){
		int x_int = (int)robot->pose.x;
		int y_int = (int)robot->pose.y;
		int phi_int = (int)(robot->pose.phi*10000.0f); //radian
		//int phi_int = (int)(robot->pose.phi * 180.0 /3.14); //degree
		
		printf("x=%d mm, y=%d mm, phi=%d radian\n", // world ÁÂÇ¥, ·Îº¿ pose Ãâ·Â
		x_int,
		y_int, 
		phi_int); 
}


