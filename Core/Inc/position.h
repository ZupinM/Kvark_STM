#ifndef __POSITION__
#define __POSITION__

#include <stdlib.h>

#define ZEROSPEED 			1200
#define MOVE_DIR_IN 		0x01
#define MOVE_DIR_OUT 		0x02
//		flags;
#define pwm_1                 0           //"1" = pwm 1 enable
#define pwm_12                1           //"1" = pwm 12 enable
#define pwm_2                 2           //"1" = pwm 2 enable
#define tick_1ms              3           //"1" = 20ms tick appear
#define multi_key             4			//"1" = one button is already pressed
#define manual_drive          5			//"1" = motor is moving manualy
#define do_ref_A              6			//"1" = do (doing) reference
#define do_ref_B              7			//"1" = do (doing) reference
#define full_pwm              8			//"1" = pwm je na 100% (motor je koncal z pospesevanjem - uporabno v go_ref() )
#define select_B_motor        9			//"1" = izbrana je druga os
#define flash_erase_done      10 			//"1" = flash erase for settings is done
#define reset_it              11			//"1" = do reset

#define overvoltage           12
#define undervoltage          13
#define buttonstuck           14
#define flash_write_done      15
#define tracker_shutting_down 16
#define lineR_measure         17
#define lineR_init            18
#define Modbus_timeout        19
#define select_AB_motor       20

extern char motor_break;

void position_handling(void);
void motor_handler(void);
static inline void motor_accelerate(int pid_out);
static inline void motor_brake(int pid_out);
void DC_QuadEncoder(void);

#endif
