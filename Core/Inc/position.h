#ifndef __POSITION__
#define __POSITION__

#include <stdlib.h>

#define ZEROSPEED 			12000
#define MOVE_DIR_IN 		0x01
#define MOVE_DIR_OUT 		0x02

extern char motor_break;

void position_handling(void);
void motor_handler(void);
void DC_QuadEncoder(void);

#endif
