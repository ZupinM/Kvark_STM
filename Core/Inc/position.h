#ifndef __POSITION__
#define __POSITION__

#include <stdlib.h>
#include "gpio.h"

#define ZEROSPEED 			20000
#define MOVE_DIR_IN 		0x01
#define MOVE_DIR_OUT 		0x02

#define HALL_LOST_DETECT  		10


#define HALL_ENABLED_A		0
#define HALL_ENABLED_B		1
#define HALL_ENABLED_BOTH	2
#define HALL_ENABLED_SINGLE	3

extern char motor_break;
extern unsigned int hall_enable; // bitwise A motor: bits 0, 1; B motor: bits 16, 17.

void position_handling(void);
void motor_handler(void);
void DC_QuadEncoder(void);
GPIO_PinState getHallState1(unsigned char index);
GPIO_PinState getHallState2(unsigned char index);
unsigned char get_Hall_Enable(uint8_t motor, uint8_t hall_A_B);

#endif
