/*********************************************************************************************************************
* Screen print helper
*********************************************************************************************************************/

#ifndef _screen_print_h_
#define _screen_print_h_

#include "zf_common_headfile.h"

void screen_print_init(void);
void screen_print_string(const char *str);
void screen_print_line(uint8 line, const char *str);
void screen_print_motor_duty(int16 motor1_duty, int16 motor2_duty, int16 motor3_duty, int16 motor4_duty);
void screen_print_openart_packet_status(void);

#endif
