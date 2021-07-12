/**************************************************************************
 *		"Pico" solar positioner software
 *
 *		filename: exception_handlers.c  
 *		pcb: tiv29B
 *
 *		Copyright(C) 2011, Sat Control d.o.o.
 *		All rights reserved.
**************************************************************************/


/******************************************************************************
  SysTick_Handler		8kHz (125us)
*******************************************************************************/
int HallCnt1A, HallCnt1B, HallCnt2A, HallCnt2B;

#include "position.h"
#include "main.h"
#include "gpio.h"
#include "adc.h"
#include "RTT/SEGGER_RTT.h"
#include <stdlib.h>

#define HALL_DEBOUNCE 6
#define DOWN_POS_MIN 64 // 200ms
#define DOWN_POS_DEFAULT 160 // 500ms
#define MOTOR_START_RAMP 40

extern volatile int slaveCommandTimeout;

// variables for terminating ramp
char first_moving = 1;
unsigned int stop_timeout = 0;
char motor_break = 0;
int measureDelay = 0;
uint8_t debounce_cntA;
uint8_t debounce_cntB;

extern volatile unsigned int SysTics;
extern volatile unsigned int tick_1ms_cnt;
extern volatile unsigned int flags;
extern unsigned int speedA;
extern unsigned int speedB;
extern unsigned int hall_enable; // bitwise A motor: bits 0, 1; B motor: bits 16, 17.
extern unsigned int error_hall_A;
extern unsigned int error_hall_B;
extern unsigned int A_enc_new;
extern unsigned int A_enc_old;
extern unsigned int A_enc_temp;
extern unsigned int B_enc_new;
extern unsigned int B_enc_old;
extern unsigned int B_enc_temp;
extern unsigned int Hall_CntDown; 
extern volatile signed int position_A;
extern volatile signed int position_B;
extern char first_movingA;
extern char first_movingB;
extern char moving_dirA;
extern char moving_dirB;
extern signed int destination_A;
extern signed int destination_B;
extern unsigned char move_direction;
extern unsigned int countMoving;
uint32_t stopTime;
extern unsigned int accel, acceleration;
extern volatile uint32_t adc_select;
extern volatile uint32_t adc_error;
extern unsigned int adc0_N, adc1_N, adc2_N, adc3_N, adc5_N, adc6_N, adc7_N;
extern unsigned int adc0_SUM, adc1_SUM, adc2_SUM, adc3_SUM, adc5_SUM, adc6_SUM, adc7_SUM; 
extern unsigned int adc0_VAL, adc1_VAL, adc2_VAL, adc3_VAL, adc5_VAL, adc6_VAL, adc7_VAL;
extern float MotorA_ramp;
extern float MotorB_ramp;

uint8_t Pgain = 40;
//float Pgain_neg = 0.5;  //Negative PWM PID
#define P_GAIN_NEG (1/2)  //Negative PWM PID 
//float Igain = 0; 
//float Dgain = 60;
int32_t speed_err_prev = 0;
int32_t pid_out_prev;

#define PWM_MAX_NEG 800
//#define USE_D_IN_PID 1
//#define USE_I_IN_PID 1

uint32_t speed_freewheel;
extern uint32_t speed_freewheelA;
extern uint32_t speed_freewheelB;
uint32_t speed_onStart;
int32_t speed_set;
uint8_t stop_started;
int32_t pid_out;
int32_t pid_P;
int32_t pid_I;
//int32_t pid_D;
uint32_t stopTime_cnt;
uint32_t stopTime_real;

uint8_t was_full_speed_move;
uint8_t brakeStarted;

uint16_t hallCnt1A_S;
uint16_t hallCnt1A_N;
uint16_t hallCnt2A_S;
uint16_t hallCnt2A_N;
uint8_t hallState_prev_A1;
uint8_t hallState_prev_A2;
uint8_t hallState_prev_B1;
uint8_t hallState_prev_B2;

uint16_t speed_h; // high refresh rate speed (period)
uint16_t speed_real; // period converted to speed
int32_t speed_err;
uint16_t speed_err_new; //pid_D saved speed
int32_t err_position_start;
uint8_t motorStarting;
uint8_t OneHallUsed;

void position_handling(void) {


  if(bldc_cm->state & BLDC_MOTOR_STATE_DC_OPERATION){
	  DC_QuadEncoder();
  }


/******************************************************
    accelerating / breaking motors (for A and B axis)
******************************************************/
  if(stop_timeout > 10){      // Wait on stop_timeout
    if(speed_h == ZEROSPEED)
      stop_timeout --;
    motor_break = 1;
    measureDelay = 10000;
    return;
  }else if (stop_timeout > 0){  //speed = 0 & stop timeout reached
    stop_timeout = 0;
    stop_motor();
    stop_started = 0;
    brakeStarted = 0;
    pid_I = 0;
    if (!(flags&(1<<select_B_motor)) ){
      destination_A = position_A;
    }else{
      destination_B = position_B;
    }
    return;
  }

  if (move_direction != 0) {

    int32_t err_position;

    if (!(flags&(1<<select_B_motor)) ){
      err_position = destination_A - position_A;
      stopTime = MotorA_ramp * 10;
      speed_freewheel = speed_freewheelA;
    }
    else{
      err_position = destination_B - position_B;
      stopTime = MotorB_ramp * 10;
      speed_freewheel = speed_freewheelB;
    }
    if(move_direction == MOVE_DIR_OUT && err_position < -100 && //Stop when direction of destination is changed
       (!(flags& (1 << do_ref_A) && !(flags&(1<<select_B_motor)))) && (!(flags& (1 << do_ref_B) && (flags&(1<<select_B_motor)))) ) //not when homing
    {       
      stop_motor();
      return;
    }
    else if(move_direction == MOVE_DIR_IN && err_position > 100 && //Stop when direction of destination is changed
            (!(flags& (1 << do_ref_A) && !(flags&(1<<select_B_motor)))) && (!(flags& (1 << do_ref_B) && (flags&(1<<select_B_motor)))) ) //not when homing
    {
      stop_motor();
      return;
    }
    err_position = abs(err_position);

    if(err_position > 1000){
      was_full_speed_move = 1; // Enable Stop time measuring only on long Moves
    }
  
    if(stopTime == 0){
      stopTime_real = 1000;     //default on uninitialized stopTime
    }else{
      stopTime_real = stopTime * speed_freewheel/speed_h * speed_freewheel/speed_h; //* change of speed squared
    }

    speed_real = (ZEROSPEED * 100)/speed_h; // period converted to speed

    if((err_position * speed_h < (stopTime_real) ) && !stop_started && (!(flags & (1 << manual_drive))) && //speed_h  je perioda (1/v), za manj racunanja
      (!(flags& (1 << do_ref_A) && !(flags&(1<<select_B_motor)))) && (!(flags& (1 << do_ref_B) && (flags&(1<<select_B_motor)))) ) // Dont stop when homing
    { 
      stop_started = 1;
      speed_onStart = speed_real;
      err_position_start = err_position;
      pid_out = 0;
    }

    if(stop_started){ //MOTOR STOPING

      if(stopTime == 0 && was_full_speed_move){    //Measure stop time on 1st stop
        if(stopTime_cnt == 0){ // Measure freewheel speed on start of stoping time measurement
          speed_freewheel = speed_h;
        }
        motor_accelerate(0);
        destination_A = position_A;
        destination_B = position_B;
        stopTime_cnt ++;
        if(speed_h == ZEROSPEED){
          stopTime = stopTime_cnt;
          if (!(flags&(1<<select_B_motor)) ){
            MotorA_ramp = stopTime /10;
            speed_freewheelA = speed_freewheel;
          }
          else{
            MotorB_ramp = stopTime /10;
            speed_freewheelB = speed_freewheel;
          }
        }
        return;
      }
      was_full_speed_move = 0;
      stopTime_cnt = 0;

      flags&=~(1<<full_pwm);

      //HARD ACTIVE BRAKING
      if(err_position < 1 || brakeStarted ||   (err_position <= 2 && (speed_h < speed_freewheel*6 || OneHallUsed)) ){ 
        motor_brake(PWM_MAX_NEG); //Hard braking
        brakeStarted = 1;
        //destination_A = position_A;
        stop_timeout = 320; //40ms hard braking
        return;
      }

      if(speed_h == ZEROSPEED){ // if motor stopped before destination is reached
        stop_timeout = 320;
        motor_accelerate(0); //turn-off PWM
        return;
      }

      //PID
      speed_set = speed_onStart *  (err_position + err_position_start/4) / (err_position_start * 5/4); // Ramp aimed 25% longer, to avoid too low comutation speed
      speed_err = speed_set - speed_real;

      if(speed_err < 0){
        pid_P = P_GAIN_NEG * speed_err;
      }else{
        pid_P = Pgain * speed_err;
      }
      pid_out = pid_P;
#ifdef USE_I_IN_PID
      if(pid_I > 100 && speed_err < 1)
        pid_I -= 4;
      if(pid_I < -100 && speed_err > 1) //Unwind pid_I faster when sign changes
        pid_I += 4;  
      pid_I += Igain * speed_err;//Subtract pid_I faster than add
      if(pid_I > 20000)
        pid_I = 20000;     //PID_I limits
      if(pid_I < -PWM_MAX_NEG)
        pid_I = -PWM_MAX_NEG;

      pid_out += pid_I;
#endif     
      
#ifdef USE_D_IN_PID
      pid_D = (speed_err_new - speed_err_prev) * Dgain;
      pid_out += pid_D;
      if(speed_err != speed_err_new){ //Slow motor response -> in most loops diff=0
        speed_err_prev = speed_err_new; //keep derivative in all loops
        speed_err_new = speed_err;
      }
#endif
      if(pid_out > 0){ //ACCELERATE
          if(pid_out > PWM_PERIODA_MR3){ //Limits
            pid_out = PWM_PERIODA_MR3;
          }
          motor_accelerate(pid_out); 
      }
      else{ //BRAKE
          if(pid_out < -2*PWM_MAX_NEG)  //Limits
            pid_out = -2*PWM_MAX_NEG;
          motor_brake(-1*pid_out);
      }
    }

    else{   //Motor start
      pid_out += MOTOR_START_RAMP;
      if(pid_out > PWM_PERIODA_MR3){    //Max Limit
        pid_out = PWM_PERIODA_MR3;
        flags|=(1<<full_pwm);
        motorStarting = 0;
      }else if (pid_out < PWM_START_MR){//Min Limit
        pid_out = PWM_START_MR;
        motorStarting = 1;//prevent debounce filtering on startup
      }
      motor_accelerate(pid_out);
    }
  }else{ //move_direction == 0
    pid_out = 0;
    motorStarting = 0;
  }
}



static inline void motor_accelerate(int pid_out){

	if(!(bldc_cm->state & BLDC_MOTOR_STATE_DC_OPERATION)){
		bldc_cm->state &= ~BLDC_MOTOR_STATE_BRAKING;
	}
	else //DC operation
	{
	  //A MOTOR
	  if (!(flags&(1<<select_B_motor)) || (flags&(1<<select_AB_motor)) ){
		if(flags&(1<<pwm_1)){
		  LPC_TMR16B0->MR1 = 0;
		  GPIOSetValue(PORT_3, 1, clear);           //disable MOTOR_EN_2
		  GPIOSetValue(PORT_3, 3, clear);     	//disable MOTOR_EN_1
		  LPC_TMR16B0->MR0 = pid_out;
		  GPIOSetValue(PORT_3, 2, set);		//enable  MOTOR_EN_12
		}
		else if(flags&(1<<pwm_12)){
		  LPC_TMR16B0->MR0 = 0;
		  GPIOSetValue(PORT_3, 1, clear);            //disable MOTOR_EN_2
		  GPIOSetValue(PORT_3, 2, clear);     	//disable MOTOR_EN_12
		  LPC_TMR16B0->MR1 = pid_out;
		  GPIOSetValue(PORT_3, 3, set);		//enable MOTOR_EN_1
		}
	  }
	  //B MOTOR
	  if (flags&(1<<select_B_motor)){
		if(flags&(1<<pwm_2)){
		LPC_TMR16B0->MR1 = 0;
		  GPIOSetValue(PORT_3, 1, clear);           //disable MOTOR_EN_2
		  GPIOSetValue(PORT_3, 3, clear);     	//disable MOTOR_EN_1
		  LPC_TMR16B1->MR0 = pid_out;
		  GPIOSetValue(PORT_3, 2, set);		//enable  MOTOR_EN_12
		}
		else if(flags&(1<<pwm_12)){
		  LPC_TMR16B1->MR0 = 0;
		  GPIOSetValue(PORT_3, 3, clear);            //disable MOTOR_EN_1
		  GPIOSetValue(PORT_3, 2, clear);     	//disable MOTOR_EN_12
		  LPC_TMR16B0->MR1 = pid_out;
		  GPIOSetValue(PORT_3, 1, set);		//enable MOTOR_EN_2
		}
	  }
	}
}

static inline void motor_brake(int pid_out){

	if(!(bldc_cm->state & BLDC_MOTOR_STATE_DC_OPERATION)){
		bldc_cm->state |= BLDC_MOTOR_STATE_BRAKING;
	}
	else //DC operation
	{
	  //A MOTOR
	  if (!(flags&(1<<select_B_motor)) || (flags&(1<<select_AB_motor)) ){
		if(flags&(1<<pwm_12)){
		  LPC_TMR16B0->MR1 = 0;
		  GPIOSetValue(PORT_3, 1, clear);           //disable MOTOR_EN_2
		  GPIOSetValue(PORT_3, 3, clear);     	//disable MOTOR_EN_1
		  LPC_TMR16B0->MR0 = pid_out;
		  GPIOSetValue(PORT_3, 2, set);		//enable  MOTOR_EN_12
		}
		else if(flags&(1<<pwm_1)){
		  LPC_TMR16B0->MR0 = PWM_MAX_NEG;
		  GPIOSetValue(PORT_3, 1, clear);           //disable MOTOR_EN_2
		  GPIOSetValue(PORT_3, 2, clear);     	//disable MOTOR_EN_12
		  LPC_TMR16B0->MR1 = pid_out;
		  GPIOSetValue(PORT_3, 3, set);		//enable MOTOR_EN_1
		}
	  }
	  //B MOTOR
	  else if (flags&(1<<select_B_motor)){
		if(flags&(1<<pwm_12)){
		  LPC_TMR16B0->MR1 = 0;
		  GPIOSetValue(PORT_3, 1, clear);           //disable MOTOR_EN_2
		  GPIOSetValue(PORT_3, 3, clear);     	//disable MOTOR_EN_1
		  LPC_TMR16B1->MR0 = pid_out;
		  GPIOSetValue(PORT_3, 2, set);		//enable  MOTOR_EN_12
		}
		else if(flags&(1<<pwm_2)){
		  LPC_TMR16B1->MR0 = 0;
		  GPIOSetValue(PORT_3, 2, clear);     	//disable MOTOR_EN_12
		  GPIOSetValue(PORT_3, 3, clear);		//disable MOTOR_EN_1
		  LPC_TMR16B0->MR1 = pid_out;
		  GPIOSetValue(PORT_3, 1, set);             //enable MOTOR_EN_2
		}
	  }
	}
}







void DC_QuadEncoder(void){
	//----------A-MOTOR Fast Hall Read
	  if (!(flags&(1<<select_B_motor)) || (flags&(1<<select_AB_motor)) ){
	    if((hall_enable & 0x03) == 0x03 || (hall_enable & 0x03) == 0x01) { //Read motor speed 4x per turn
	      //----- Hall A1 -----
	      if (GPIOReadValue(PORT_2,7)) {  				// *** hall 1 = 1 ***
	        if(hallState_prev_A1 == 0){
	          if(hallCnt1A_S > (speed_h/2) || (++debounce_cntA > HALL_DEBOUNCE)){ //FILTER: ignore too short pulses, or deounce counter
	            speed_h = hallCnt1A_S;     //South pole magnet counter
	            HallCnt1A++;
	            hallCnt1A_S = 0;
	            A_enc_new |= (1<<0);
	            debounce_cntA = 0;
	            hallState_prev_A1 = 1;
	          }
	        }
	        else{ //hallState_prev_A1 == 1
	          debounce_cntA--;
	        }
	      } else {                         				// *** hall 1 = 0 ***
	        if(hallState_prev_A1 == 1){
	          if(hallCnt1A_N > (speed_h/2) || (++debounce_cntA > HALL_DEBOUNCE)){ //ignore too short pulses (debouncing)
	            speed_h = hallCnt1A_N;     //North pole magnet counter
	            hallCnt1A_N = 0;
	            A_enc_new &= ~(1<<0);
	            debounce_cntA = 0;
	            hallState_prev_A1 = 0;
	          }
	        }
	        else{ //hallState_prev_A1 == 0
	          debounce_cntA--;
	        }
	      }

	    if(hallCnt1A_S < ZEROSPEED)
	      hallCnt1A_S++;
	    else
	      speed_h = ZEROSPEED;
	    if(hallCnt1A_N < ZEROSPEED)
	      hallCnt1A_N++;
	    }

	    if((hall_enable & 0x03) == 0x03 || (hall_enable & 0x03) == 0x02) {
	      //----- Hall A2 -----
	      if (GPIOReadValue(PORT_2,6)) {  				// *** hall 1 = 1 ***
	        if(hallState_prev_A2 == 0){
	          if(hallCnt2A_S > (speed_h/2) || (++debounce_cntA > HALL_DEBOUNCE)){ //ignore too short pulses (debouncing)
	            speed_h = hallCnt2A_S;     //South pole magnet counter
	            hallCnt2A_S = 0;
	            HallCnt1B++;
	            A_enc_new |= (1<<1);
	            debounce_cntA = 0;
	            hallState_prev_A2 = 1;
	          }
	        }
	        else{ //hallState_prev_A2 == 1
	          debounce_cntA--;
	        }
	      } else {                         				// *** hall 1 = 0 ***
	        if(hallState_prev_A2 == 1){
	          if(hallCnt1A_N > (speed_h/2) || (++debounce_cntA > HALL_DEBOUNCE)){ //ignore too short pulses (debouncing)
	            speed_h = hallCnt2A_N;     //North pole magnet counter
	            hallCnt2A_N = 0;
	            A_enc_new &= ~(1<<1);
	            debounce_cntA = 0;
	            hallState_prev_A2 = 0;
	          }
	        }
	        else{ //hallState_prev_A2 == 0
	          debounce_cntA--;
	        }
	      }
	    if(hallCnt2A_S < ZEROSPEED)
	      hallCnt2A_S++;
	    else
	      speed_h = ZEROSPEED;
	    if(hallCnt2A_N < ZEROSPEED)
	      hallCnt2A_N++;
	    }
	  }

	  //----------B-MOTOR Fast Hall Read
	  else if (flags&(1<<select_B_motor)){
	    if(((hall_enable & 0x03<<16) == 0x03<<16) || ((hall_enable & 0x03<<16) == 0x01<<16)) { //Read motor speed 4x per turn
	      //----- Hall B1 -----
	      if (GPIOReadValue(PORT_0,5)) {  				// *** hall 1 = 1 ***
	        if(hallState_prev_B1 == 0){
	          if(hallCnt1A_S > (speed_h/2) || (++debounce_cntB > HALL_DEBOUNCE)){ //ignore too short pulses (debouncing)
	            speed_h = hallCnt1A_S;     //South pole magnet counter
	            hallCnt1A_S = 0;
	            HallCnt2A++;
	            B_enc_new |= (1<<0);
	            debounce_cntB = 0;
	            hallState_prev_B1 = 1;
	          }
	        }
	        else{ //hallState_prev_B1 == 1
	          debounce_cntB--;
	        }
	      } else {                         				// *** hall 1 = 0 ***
	        if(hallState_prev_B1 == 1){
	          if(hallCnt1A_N > (speed_h/2) || (++debounce_cntB > HALL_DEBOUNCE)){ //ignore too short pulses (debouncing)
	            speed_h = hallCnt1A_N;     //North pole magnet counter
	            hallCnt1A_N = 0;
	            B_enc_new &= ~(1<<0);
	            debounce_cntB = 0;
	            hallState_prev_B1 = 0;
	          }
	        }
	        else{ //hallState_prev_B1 == 0
	          debounce_cntB--;
	        }
	      }

	    if(hallCnt1A_S < ZEROSPEED)
	      hallCnt1A_S++;
	    else
	      speed_h = ZEROSPEED;
	    if(hallCnt1A_N < ZEROSPEED)
	      hallCnt1A_N++;

	    }

	    if(((hall_enable & 0x03<<16) == 0x03<<16) || ((hall_enable & 0x03<<16) == 0x02<<16)) {
	      //----- Hall B2 -----
	      if (GPIOReadValue(PORT_0,4)) {  				// *** hall 1 = 1 ***
	        if(hallState_prev_B2 == 0){
	          if(hallCnt2A_S > (speed_h/2) || (++debounce_cntB > HALL_DEBOUNCE)){ //ignore too short pulses (debouncing)
	            speed_h = hallCnt2A_S;     //South pole magnet counter
	            hallCnt2A_S = 0;
	            HallCnt2B++;
	            B_enc_new |= (1<<1);
	            debounce_cntB = 0;
	            hallState_prev_B2 = 1;
	          }
	        }
	        else{ //hallState_prev_B2 == 1
	          debounce_cntB--;
	        }
	      } else {                         				// *** hall 1 = 0 ***
	        if(hallState_prev_B2 == 1){
	          if(hallCnt1A_N > (speed_h/2) || (++debounce_cntB > HALL_DEBOUNCE)){ //ignore too short pulses (debouncing)
	            speed_h = hallCnt2A_N;     //North pole magnet counter
	            hallCnt2A_N = 0;
	            B_enc_new &= ~(1<<1);
	            debounce_cntB = 0;
	            hallState_prev_B2 = 0;
	          }
	        }
	        else{ //hallState_prev_B2 == 0
	          debounce_cntB--;
	        }
	      }
	      if(hallCnt2A_S < ZEROSPEED)
	        hallCnt2A_S++;
	      else
	        speed_h = ZEROSPEED;
	      if(hallCnt2A_N < ZEROSPEED)
	        hallCnt2A_N++;

	    }
	  }

	/********************************************
	    pulse count on A axis ---
	*********************************************/
	  if((hall_enable & 0x03) == 0x03) {
	    switch (A_enc_old) {                                          //swap bits  0...0000 00xx
	      case 0:
	        A_enc_temp=0;
	        break;
	      case 1:
	        A_enc_temp=2;
	        break;
	      case 2:
	        A_enc_temp=1;
	        break;
	      case 3:
	        A_enc_temp=3;
	        break;
	    }
	  //--------------------
	    A_enc_old=A_enc_new;        //new->old
	    A_enc_temp^=A_enc_new;      //swapped_old EXOR new
	  //    BKP_WriteBackupRegister(BKP_DR6,H_enc_old);	            //backup - da reset virtualno ne steje impulza

	    switch (A_enc_temp){                                          // direction
	      case 0: break;                                              // 00 = no change
	      case 1: {
	        if(!(Hall_CntDown&1))
	          position_A++;                                           // 01 = up
	          else
	            position_A--;
	          break;
	      }
	      case 2: {                                                   // 10 = down
	        if(!(Hall_CntDown&1))
	          position_A--;                                           // 01 = up
	        else
	          position_A++;
	        break;
	      }
	      case 3:
	        break;                                                    // 11 = no change
	    }
	  }

	  if((((hall_enable & 0x03) == 0x01) || ((hall_enable & 0x03) == 0x02))) {
	    if(A_enc_new != A_enc_old) {

	      if(first_movingA == 1) {
	        first_movingA = 0;
	        if (destination_A < position_A || (flags & (1 << do_ref_A)))
	          moving_dirA = 0;
	        else if(destination_A > position_A)
	          moving_dirA = 1;
	      }

	      if(moving_dirA == 1)
	        position_A += 2;
	      else if (moving_dirA == 0)
	        position_A -= 2;
	    }

	    A_enc_old = A_enc_new;
	    OneHallUsed = 1;
	  }


	/********************************************
	    pulse count on B axis ---
	*********************************************/
	  if((hall_enable & 0x03<<16) == 0x03<<16) {
	    switch (B_enc_old) {                                         // swap bits  0...0000 00xx
	      case 0:
	        B_enc_temp=0;
	        break;
	      case 1:
	        B_enc_temp=2;
	        break;
	      case 2:
	        B_enc_temp=1;
	        break;
	      case 3:
	        B_enc_temp=3;
	        break;
	    }
	  //--------------------
	    B_enc_old = B_enc_new;        //new->old
	    B_enc_temp ^= B_enc_new;      //swapped_old EXOR new
	  //    BKP_WriteBackupRegister(BKP_DR6,H_enc_old);	            //backup - da reset virtualno ne steje impulza

	    switch (B_enc_temp) {                                         // direction
	      case 0:
	        break;                                                    // 00 = no change
	      case 1: {
	        if(!(Hall_CntDown&2))
	          position_B++;                                           // 01 = up
	        else
	          position_B--;                                           // 01 = up
	        break;
	      }
	      case 2: {                                                   // 10 = down
	        if(!(Hall_CntDown&2))
	          position_B--;                                           // 01 = up
	        else
	          position_B++;
	        break;
	      }
	      case 3:
	        break;                                                    // 11 = no change
	    }
	  }

	  if(((hall_enable & 0x03<<16) == 0x01<<16) || ((hall_enable & 0x03<<16) == 0x02<<16)) {
	    if(B_enc_new != B_enc_old) {

	      if(first_movingB == 1) {
	        first_movingB = 0;
	        if (destination_B < position_B || (flags & (1 << do_ref_B)))
	          moving_dirB = 0;
	        else if(destination_B > position_B)
	          moving_dirB = 1;
	      }
	      if(moving_dirB == 1)
	        position_B += 2;
	      else if (moving_dirB == 0)
	        position_B -= 2;
	    }
	    B_enc_old = B_enc_new;
	    OneHallUsed = 1;
	  }

	  if(((hall_enable & 0x03) == 0x01) || ((hall_enable & 0x03) == 0x02) ||
	    ((hall_enable & 0x03<<16) == 0x01<<16) || ((hall_enable & 0x03<<16) == 0x02<<16)
	    ) {
	    if (move_direction == 0) {
	      countMoving++;
	      if(countMoving > 8000) {
	        first_movingA = 1;
	        first_movingB = 1;
	        moving_dirA = 0;
	        moving_dirB = 0;
	        countMoving = 0;
	      }
	    }
	  }
}

























