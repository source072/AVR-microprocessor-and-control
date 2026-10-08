// 260820_estimate_robot_pose_final/UART0.h

#ifndef UART0_H
#define UART0_H

int uart_putchar(char c, FILE *stream);
void uart_init();
void print_pose(Robot* robot);

#endif
