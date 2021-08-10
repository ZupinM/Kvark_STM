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

#include "position.h"
#include "main.h"
#include "Shared_Libraries/bldc.h"
#include "adc.h"
#include "tim.h"
#include "RTT/SEGGER_RTT.h"
#include <stdlib.h>

#define HALL_DEBOUNCE 6
#define DOWN_POS_MIN 64 // 200ms
#define DOWN_POS_DEFAULT 160 // 500ms
#define MOTOR_START_RAMP 4

extern volatile int slaveCommandTimeout;

// variables for terminating ramp
unsigned int stop_timeout = 0;
char motor_break = 0;
int measureDelay = 0;
uint8_t debounce_cntA[BLDC_MOTOR_COUNT];

extern volatile unsigned int SysTics;
extern volatile unsigned int tick_1ms_cnt;
extern volatile unsigned int flags;
extern unsigned int speedA;
extern unsigned int speedB;
extern unsigned int error_hall_A;
extern unsigned int error_hall_B;
extern unsigned int moving_counter[4];
unsigned int A_enc_new[BLDC_MOTOR_COUNT];
unsigned int A_enc_old[BLDC_MOTOR_COUNT];
unsigned int A_enc_temp[BLDC_MOTOR_COUNT];
extern unsigned char move_direction;
unsigned int countMoving = 0;
uint32_t stopTime;

float Pgain;
float Pgain_neg;
uint16_t pid_neg_max;
//float Pgain_neg = 0.5;  //Negative PWM PID
#define P_GAIN		40
#define P_GAIN_DC	3//12
#define P_GAIN_NEG 2  	 //Negative PWM PID divider BLDC
#define P_GAIN_NEG_DC 4  //Negative PWM PID divider
//float Igain = 0; 
//float Dgain = 60;
int32_t speed_err_prev = 0;
int32_t pid_out_prev;

#define PWM_MAX_NEG MOTOR_PWM_PERIOD
#define PWM_MAX_NEG_DC MOTOR_PWM_PERIOD
#define PWM_MAX_NEG_ACTIVE 80
//#define USE_D_IN_PID 1
//#define USE_I_IN_PID 1

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

uint16_t hallCntA_S[BLDC_MOTOR_COUNT];
uint16_t hallCntA_N[BLDC_MOTOR_COUNT];
uint16_t hallCntB_S[BLDC_MOTOR_COUNT];
uint16_t hallCntB_N[BLDC_MOTOR_COUNT];
uint8_t hallState_prev_A1[BLDC_MOTOR_COUNT];
uint8_t hallState_prev_A2[BLDC_MOTOR_COUNT];

int HallCntA[BLDC_MOTOR_COUNT];
int HallCntB[BLDC_MOTOR_COUNT];

uint16_t speed_real; // period converted to speed
int32_t speed_err;
uint16_t speed_err_new; //pid_D saved speed
int32_t err_position_start;
uint8_t motorStarting;




static inline void motor_accelerate(int pid_out){

	if(!(bldc_cm->state & BLDC_MOTOR_STATE_DC_OPERATION)){
		bldc_cm->state &= ~BLDC_MOTOR_STATE_BRAKING;
	}
	else //DC operation
	{
	  //Motor 0 & Motor 2
	  if (bldc_cm->index == 0 || bldc_cm->index == 2){
		if(!(bldc_cm->status & BLDC_STATUS_CCW)){
		  //LPC_TMR16B0->MR1 = 0;
		  setGPIO_Function(MOTOR_2H_PORT(bldc_cm->index), MOTOR_2H_PIN(bldc_cm->index), MODE_OUTPUT);
		  HAL_GPIO_WritePin(MOTOR_3L_PORT(bldc_cm->index), MOTOR_3L_PIN(bldc_cm->index), GPIO_PIN_RESET);	//disable MOTOR_EN_2
		  HAL_GPIO_WritePin(MOTOR_1L_PORT(bldc_cm->index), MOTOR_1L_PIN(bldc_cm->index), GPIO_PIN_RESET);	//disable MOTOR_EN_1
		  //LPC_TMR16B0->MR0 = pid_out;
		  setGPIO_Function(MOTOR_1H_PORT(bldc_cm->index), MOTOR_1H_PIN(bldc_cm->index), MODE_ALTERNATE);
		  HAL_GPIO_WritePin(MOTOR_2L_PORT(bldc_cm->index), MOTOR_2L_PIN(bldc_cm->index), GPIO_PIN_SET);		//enable  MOTOR_EN_12
		}
		else{
		  //LPC_TMR16B0->MR0 = 0;
		  setGPIO_Function(MOTOR_1H_PORT(bldc_cm->index), MOTOR_1H_PIN(bldc_cm->index), MODE_OUTPUT);
		  HAL_GPIO_WritePin(MOTOR_3L_PORT(bldc_cm->index), MOTOR_3L_PIN(bldc_cm->index), GPIO_PIN_RESET);	//disable MOTOR_EN_2
		  HAL_GPIO_WritePin(MOTOR_2L_PORT(bldc_cm->index), MOTOR_2L_PIN(bldc_cm->index), GPIO_PIN_RESET);	//disable MOTOR_EN_12
		  //LPC_TMR16B0->MR1 = pid_out;
		  setGPIO_Function(MOTOR_2H_PORT(bldc_cm->index), MOTOR_2H_PIN(bldc_cm->index), MODE_ALTERNATE);
		  HAL_GPIO_WritePin(MOTOR_1L_PORT(bldc_cm->index), MOTOR_1L_PIN(bldc_cm->index), GPIO_PIN_SET);		//enable  MOTOR_EN_1
		}
	  }
	  //Motor 1 & Motor 3
	  if (bldc_cm->index == 1 || bldc_cm->index == 3) {
		  if(!(bldc_cm->status & BLDC_STATUS_CCW)){
		  //LPC_TMR16B0->MR1 = 0;
		  setGPIO_Function(MOTOR_2H_PORT(bldc_cm->index -1), MOTOR_2H_PIN(bldc_cm->index -1), MODE_OUTPUT);
		  HAL_GPIO_WritePin(MOTOR_3L_PORT(bldc_cm->index -1), MOTOR_3L_PIN(bldc_cm->index -1), GPIO_PIN_RESET);	//disable MOTOR_EN_2
		  HAL_GPIO_WritePin(MOTOR_1L_PORT(bldc_cm->index -1), MOTOR_1L_PIN(bldc_cm->index -1), GPIO_PIN_RESET);	//disable MOTOR_EN_1
		  //LPC_TMR16B1->MR0 = pid_out;
		  setGPIO_Function(MOTOR_3H_PORT(bldc_cm->index -1), MOTOR_3H_PIN(bldc_cm->index -1), MODE_ALTERNATE);
		  HAL_GPIO_WritePin(MOTOR_2L_PORT(bldc_cm->index -1), MOTOR_2L_PIN(bldc_cm->index -1), GPIO_PIN_SET);		//enable  MOTOR_EN_12
		}
		else{
		  //LPC_TMR16B1->MR0 = 0;
		  setGPIO_Function(MOTOR_3H_PORT(bldc_cm->index -1), MOTOR_3H_PIN(bldc_cm->index -1), MODE_OUTPUT);
		  HAL_GPIO_WritePin(MOTOR_1L_PORT(bldc_cm->index -1), MOTOR_1L_PIN(bldc_cm->index -1), GPIO_PIN_RESET);	//disable MOTOR_EN_1
		  HAL_GPIO_WritePin(MOTOR_2L_PORT(bldc_cm->index -1), MOTOR_2L_PIN(bldc_cm->index -1), GPIO_PIN_RESET);	//disable MOTOR_EN_12

		  setGPIO_Function(MOTOR_2H_PORT(bldc_cm->index -1), MOTOR_2H_PIN(bldc_cm->index -1), MODE_ALTERNATE);	//LPC_TMR16B0->MR1 = pid_out;
		  HAL_GPIO_WritePin(MOTOR_3L_PORT(bldc_cm->index -1), MOTOR_3L_PIN(bldc_cm->index -1), GPIO_PIN_SET);		//enable  MOTOR_EN_2
		  //enable MOTOR_EN_2
		}
	  }
	}
	bldc_update_pwm(pid_out);
	if(bldc_cm->speed == ZEROSPEED){
		bldc_cm->status &= ~BLDC_STATUS_ACTIVE; // Required to start ActivateDrivers(1)
		ActivateDrivers(1);
	}
}

static inline void motor_brake(int pid_out){

	if(!(bldc_cm->state & BLDC_MOTOR_STATE_DC_OPERATION)){
		bldc_cm->state |= BLDC_MOTOR_STATE_BRAKING;
	}
	else //DC operation
	{
		  //A MOTOR
		  if (bldc_cm->index == 0 || bldc_cm->index == 2){
			if(bldc_cm->status & BLDC_STATUS_CCW){
			  setGPIO_Function(MOTOR_2H_PORT(bldc_cm->index), MOTOR_2H_PIN(bldc_cm->index), MODE_OUTPUT);		//LPC_TMR16B0->MR1 = 0;
			  setGPIO_Function(MOTOR_1L_PORT(bldc_cm->index), MOTOR_1L_PIN(bldc_cm->index), MODE_OUTPUT);		//LPC_TMR16B0->MR1 = 0;
			  HAL_GPIO_WritePin(MOTOR_3L_PORT(bldc_cm->index), MOTOR_3L_PIN(bldc_cm->index), GPIO_PIN_RESET);	//disable MOTOR_EN_2
			  HAL_GPIO_WritePin(MOTOR_1L_PORT(bldc_cm->index), MOTOR_1L_PIN(bldc_cm->index), GPIO_PIN_RESET);	//disable MOTOR_EN_1

			  setGPIO_Function(MOTOR_1H_PORT(bldc_cm->index), MOTOR_1H_PIN(bldc_cm->index), MODE_ALTERNATE);	//LPC_TMR16B0->MR0 = pid_out;
			  if(pid_out > PWM_MAX_NEG_DC){ // back-EMF braking on first half + active braking on second
				  HAL_GPIO_WritePin(MOTOR_2L_PORT(bldc_cm->index), MOTOR_2L_PIN(bldc_cm->index), GPIO_PIN_SET);		//enable  MOTOR_EN_12
			  }
			  else{
				  //setGPIO_Function(MOTOR_2L_PORT(bldc_cm->index), MOTOR_2L_PIN(bldc_cm->index), MODE_ALTERNATE);
			  }
			}
			else{
			  setGPIO_Function(MOTOR_1H_PORT(bldc_cm->index), MOTOR_1H_PIN(bldc_cm->index), MODE_OUTPUT);		//LPC_TMR16B0->MR0 = 0;
			  setGPIO_Function(MOTOR_2L_PORT(bldc_cm->index), MOTOR_2L_PIN(bldc_cm->index), MODE_OUTPUT);		//LPC_TMR16B0->MR0 = 0;
			  HAL_GPIO_WritePin(MOTOR_3L_PORT(bldc_cm->index), MOTOR_3L_PIN(bldc_cm->index), GPIO_PIN_RESET);	//disable MOTOR_EN_2
			  HAL_GPIO_WritePin(MOTOR_2L_PORT(bldc_cm->index), MOTOR_2L_PIN(bldc_cm->index), GPIO_PIN_RESET);	//disable MOTOR_EN_12

			  setGPIO_Function(MOTOR_2H_PORT(bldc_cm->index), MOTOR_2H_PIN(bldc_cm->index), MODE_ALTERNATE);	//LPC_TMR16B0->MR1 = pid_out;
			  if(pid_out > PWM_MAX_NEG_DC){
				  HAL_GPIO_WritePin(MOTOR_1L_PORT(bldc_cm->index), MOTOR_1L_PIN(bldc_cm->index), GPIO_PIN_SET); 		//enable  MOTOR_EN_1
			  }else{
				  //setGPIO_Function(MOTOR_1L_PORT(bldc_cm->index), MOTOR_1L_PIN(bldc_cm->index), MODE_ALTERNATE);
			  }
			}
		  }
		  //B MOTOR
		  if (bldc_cm->index == 1 || bldc_cm->index == 3) {
			if(bldc_cm->status & BLDC_STATUS_CCW){
			  setGPIO_Function(MOTOR_2H_PORT(bldc_cm->index -1), MOTOR_2H_PIN(bldc_cm->index -1), MODE_OUTPUT);		//LPC_TMR16B0->MR1 = 0;
			  HAL_GPIO_WritePin(MOTOR_3L_PORT(bldc_cm->index -1), MOTOR_3L_PIN(bldc_cm->index -1), GPIO_PIN_RESET);	//disable MOTOR_EN_2
			  HAL_GPIO_WritePin(MOTOR_1L_PORT(bldc_cm->index -1), MOTOR_1L_PIN(bldc_cm->index -1), GPIO_PIN_RESET);	//disable MOTOR_EN_1

			  setGPIO_Function(MOTOR_3H_PORT(bldc_cm->index -1), MOTOR_3H_PIN(bldc_cm->index -1), MODE_ALTERNATE);	//LPC_TMR16B1->MR0 = pid_out;
			  HAL_GPIO_WritePin(MOTOR_2L_PORT(bldc_cm->index -1), MOTOR_2L_PIN(bldc_cm->index -1), GPIO_PIN_SET);		//enable  MOTOR_EN_12
			}
			else{
			  setGPIO_Function(MOTOR_3H_PORT(bldc_cm->index -1), MOTOR_3H_PIN(bldc_cm->index -1), MODE_OUTPUT);		//LPC_TMR16B1->MR0 = 0;
			  HAL_GPIO_WritePin(MOTOR_1L_PORT(bldc_cm->index -1), MOTOR_1L_PIN(bldc_cm->index -1), GPIO_PIN_RESET);	//disable MOTOR_EN_1
			  HAL_GPIO_WritePin(MOTOR_2L_PORT(bldc_cm->index -1), MOTOR_2L_PIN(bldc_cm->index -1), GPIO_PIN_RESET);	//disable MOTOR_EN_12

			  setGPIO_Function(MOTOR_2H_PORT(bldc_cm->index -1), MOTOR_2H_PIN(bldc_cm->index -1), MODE_ALTERNATE);	//LPC_TMR16B0->MR1 = pid_out;
			  HAL_GPIO_WritePin(MOTOR_3L_PORT(bldc_cm->index -1), MOTOR_3L_PIN(bldc_cm->index -1), GPIO_PIN_SET);		//enable  MOTOR_EN_2
			}
		  }
	}
	if(pid_out > PWM_MAX_NEG_DC){ // back-EMF braking on first half + active braking on second
		pid_out -= PWM_MAX_NEG_DC;
	}
	if(pid_out < 10){
		pid_out = 10;
	}
	bldc_update_pwm(pid_out);
	if(bldc_cm->speed == ZEROSPEED){
		bldc_cm->status &= ~BLDC_STATUS_ACTIVE; // Required to start ActivateDrivers(1)
		ActivateDrivers(1);
	}
}
uint8_t active_status_old;
uint8_t cnt_print;
void position_handling(void) {


  if(bldc_cm->state & BLDC_MOTOR_STATE_DC_OPERATION){
	  DC_QuadEncoder();
	  Pgain = P_GAIN_DC;
	  Pgain_neg = P_GAIN_NEG_DC;
	  pid_neg_max = PWM_MAX_NEG_DC;
  }
  else{
	  Pgain = P_GAIN;
	  Pgain_neg = P_GAIN_NEG;
	  pid_neg_max = PWM_MAX_NEG;
	  if (bldc_cm->speed == bldc_cm->speed_old){
		  if(bldc_cm->inactivity_cnt++ > 500 && bldc_cm->speed != ZEROSPEED){
			  bldc_cm->speed = ZEROSPEED;
			  bldc_cm->inactivity_cnt = 0;
		  }
	  }else{
		  bldc_cm->speed_old = bldc_cm->speed;
		  bldc_cm->inactivity_cnt = 0;
	  }
  }


/******************************************************
    accelerating / breaking motors (for A and B axis)
******************************************************/
  if(stop_timeout > 10){      // Wait on stop_timeout
    if(bldc_cm->speed == ZEROSPEED)
      stop_timeout --;
    motor_break = 1;
    measureDelay = 10000;
    return;
  }else if (stop_timeout > 0){  //speed = 0 & stop timeout reached
    stop_timeout = 0;
    bldc_Stop(0);
    stop_started = 0;
    brakeStarted = 0;
    pid_I = 0;
    bldc_cm->target = bldc_cm->position;
    return;
  }

  /*  uint8_t active_status = bldc_cm->status & BLDC_STATUS_ACTIVE;
  if(active_status != active_status_old)
	  SEGGER_RTT_printf(0, "switchr\r\n");
  active_status_old = active_status;
*/
  if (bldc_cm->status & BLDC_STATUS_ACTIVE) {
    int32_t err_position;

	err_position = bldc_cm->target - bldc_cm->position;
	stopTime = bldc_cm->ramp * 10;

    if(move_direction == MOVE_DIR_OUT && err_position < -100 && //Stop when direction of destination is changed
       (!(bldc_cm->ctrl & BLDC_CTRL_HOMING ))  ) //not when homing
    {       
    	bldc_Stop(0);
        return;
    }
    else if(move_direction == MOVE_DIR_IN && err_position > 100 && //Stop when direction of destination is changed
    		(!(bldc_cm->ctrl & BLDC_CTRL_HOMING )) ) //not when homing
    {
    	bldc_Stop(0);
        return;
    }
    err_position = abs(err_position);

    if(err_position > 1000){
      was_full_speed_move = 1; // Enable Stop time measuring only on long Moves
    }
  
    if(stopTime == 0){
      stopTime_real = 1000;     //default on uninitialized stopTime
    }else{
      stopTime_real = stopTime * bldc_cm->speed_freewheel/bldc_cm->speed * bldc_cm->speed_freewheel/bldc_cm->speed; //* change of speed squared
    }

    speed_real = (ZEROSPEED * 100)/bldc_cm->speed; // period converted to speed
    uint8_t speed_divider = 1;
    if(!(bldc_cm->state & BLDC_MOTOR_STATE_DC_OPERATION)){
    	speed_divider = 25;
    }
    if(((err_position * bldc_cm->speed / speed_divider )< (stopTime_real) ) && !stop_started && (!ButtonStatus) && //speed  je perioda (1/v), za manj racunanja
    		(!(bldc_cm->ctrl & BLDC_CTRL_HOMING ))  ) // Dont stop when homing
    { 
      stop_started = 1;
      speed_onStart = speed_real;
      err_position_start = err_position;
      pid_out = 0;
    }
    if(stop_started){ //MOTOR STOPING

      if(stopTime == 0 && was_full_speed_move){    //Measure stop time on 1st stop
        if(stopTime_cnt == 0){ // Measure freewheel speed on start of stoping time measurement
        	bldc_cm->speed_freewheel = bldc_cm->speed;
        }
        motor_accelerate(0);
        bldc_cm->target = bldc_cm->position;
        stopTime_cnt ++;
        if(bldc_cm->speed == ZEROSPEED){
          stopTime = stopTime_cnt;
          bldc_cm->ramp = stopTime /10;
          //bldc_cm->speed_freewheel = speed_freewheel;
        }
        return;
      }
      was_full_speed_move = 0;
      stopTime_cnt = 0;

      //HARD ACTIVE BRAKING
      if(err_position < 1 || brakeStarted ||   (err_position <= 2 && (bldc_cm->speed < bldc_cm->speed_freewheel*6 || get_Hall_Enable(bldc_cm->index, HALL_ENABLED_SINGLE))) ){
    	if(bldc_cm->state & BLDC_MOTOR_STATE_DC_OPERATION){
    		motor_brake(PWM_MAX_NEG_DC+PWM_MAX_NEG_ACTIVE); //Hard braking
    	}else{
    		motor_brake(PWM_MAX_NEG); //BLDC Hard braking
    	}
        brakeStarted = 1;
        //destination_A = position_A;
        stop_timeout = 400; //40ms hard braking
        return;
      }

      if(bldc_cm->speed == ZEROSPEED){ // if motor stopped before destination is reached
        stop_timeout = 400;
        motor_accelerate(0); //turn-off PWM
        return;
      }

      //PID
      speed_set = speed_onStart *  (err_position + err_position_start/4) / (err_position_start * 5/4); // Ramp aimed 25% longer, to avoid too low comutation speed
      speed_err = speed_set - speed_real;

      if(speed_err < 0){
    	pid_P = speed_err / Pgain_neg;
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
      SEGGER_RTT_printf(0, "e%d, p%d s%d\r\n", err_position, pid_out, bldc_cm->speed);
      if(pid_out > 0){ //ACCELERATE
          if(pid_out > MOTOR_PWM_PERIOD){ //Limits
            pid_out = MOTOR_PWM_PERIOD;
          }
          motor_accelerate(pid_out); 
      }
      else{ //BRAKE
          if(pid_out < -2*pid_neg_max){  //Limits
             pid_out = -2*pid_neg_max;
          }
          motor_brake(-1*pid_out);
      }
    }
    else{   //Motor start
      pid_out += MOTOR_START_RAMP;
      if(pid_out > MOTOR_PWM_PERIOD){    //Max Limit
        pid_out = MOTOR_PWM_PERIOD;
        motorStarting = 0;
      }else if (pid_out < MOTOR_PWM_MIN){//Min Limit
        pid_out = MOTOR_PWM_MIN;
        motorStarting = 1;//prevent debounce filtering on startup
      }
      motor_accelerate(pid_out);
    }
  }else{ // ! bldc_cm->status & BLDC_STATUS_ACTIVE
    pid_out = 0;
    motorStarting = 0;
  }
}




#define getHallStateA1(motorIndex) (motorIndex ? HAL_GPIO_ReadPin(HALL_A1_GPIO_Port, HALL_A1_Pin ) : HAL_GPIO_ReadPin(HALL_B1_GPIO_Port, HALL_B1_Pin ))
#define getHallStateA2(motorIndex) (motorIndex ? HAL_GPIO_ReadPin(HALL_A2_GPIO_Port, HALL_A2_Pin ) : HAL_GPIO_ReadPin(HALL_B2_GPIO_Port, HALL_B2_Pin ))

#define getHallStateB1(motorIndex) (motorIndex ? HAL_GPIO_ReadPin(HALL_A3_GPIO_Port, HALL_A3_Pin ) : HAL_GPIO_ReadPin(HALL_B3_GPIO_Port, HALL_B3_Pin ))
#define getHallStateB2(motorIndex) (motorIndex ? HAL_GPIO_ReadPin(END_SW_A_HI_GPIO_Port, END_SW_A_HI_Pin ) : HAL_GPIO_ReadPin(END_SW_B_HI_GPIO_Port, END_SW_B_HI_Pin ))

GPIO_PinState getHallState1(unsigned char index){
	switch(index)
	{
	case 0:
		return HAL_GPIO_ReadPin(HALL_A1_GPIO_Port, HALL_A1_Pin);
	case 1:
		return HAL_GPIO_ReadPin(HALL_A3_GPIO_Port, HALL_A3_Pin);
	case 2:
		return HAL_GPIO_ReadPin(HALL_B1_GPIO_Port, HALL_B1_Pin);
	case 3:
		return HAL_GPIO_ReadPin(HALL_B3_GPIO_Port, HALL_B3_Pin);
	default: return 0;
	}
}

GPIO_PinState getHallState2(unsigned char index){
	switch(index)
	{
	case 0:
		return HAL_GPIO_ReadPin(HALL_A2_GPIO_Port, HALL_A2_Pin);
	case 1:
		return HAL_GPIO_ReadPin(END_SW_A_HI_GPIO_Port, END_SW_A_HI_Pin);
	case 2:
		return HAL_GPIO_ReadPin(HALL_B2_GPIO_Port, HALL_B2_Pin);
	case 3:
		return HAL_GPIO_ReadPin(END_SW_B_HI_GPIO_Port, END_SW_B_HI_Pin);
	default: return 0;
	}
}


void DC_QuadEncoder(void){
	uint8_t motor = bldc_cm->index;
	//----------A-MOTOR Fast Hall Read
	  if (!(bldc_cm->state & BLDC_MOTOR_STATE_DC_B_MOTOR) || (bldc_cm->state & BLDC_MOTOR_STATE_DC_AB_MOTOR) ){
	    if(get_Hall_Enable(motor, HALL_ENABLED_A)) { //Read motor speed 4x per turn
	      //----- Hall A1 -----
	      if (getHallState1(bldc_cm->index) == GPIO_PIN_SET) {  				// *** hall 1 = 1 ***
	        if(hallState_prev_A1[motor] == 0){
	          if(++debounce_cntA[motor] > HALL_DEBOUNCE){ //FILTER: ignore too short pulses, or deounce counter
	        	bldc_cm->speed = hallCntA_S[motor];     //South pole magnet counter
	        	moving_counter[motor] = 400;
	            HallCntA[motor]++;
	            hallCntA_S[motor] = 0;
	            A_enc_new[motor] |= (1<<0);
	            debounce_cntA[motor] = 0;
	            hallState_prev_A1[motor] = 1;
	          }
	        }
	        else{ //hallState_prev_A1 == 1
	          debounce_cntA[motor]--;
	        }
	      } else {                         				// *** hall 1 = 0 ***
	        if(hallState_prev_A1[motor] == 1){
	          if(++debounce_cntA[motor] > HALL_DEBOUNCE){ //ignore too short pulses (debouncing)
	        	bldc_cm->speed = hallCntA_N[motor];     //North pole magnet counter
	        	moving_counter[motor] = 400;
	            hallCntA_N[motor] = 0;
	            A_enc_new[motor] &= ~(1<<0);
	            debounce_cntA[motor] = 0;
	            hallState_prev_A1[motor] = 0;
	          }
	        }
	        else{ //hallState_prev_A1 == 0
	          debounce_cntA[motor]--;
	        }
	      }

	    if(hallCntA_S[motor] < ZEROSPEED / 10)
	      hallCntA_S[motor]++;
	    else
	      bldc_cm->speed = ZEROSPEED;
	    if(hallCntA_N[motor] < ZEROSPEED / 10)
	      hallCntA_N[motor]++;
	    }

	    if(get_Hall_Enable(motor, HALL_ENABLED_B)) {
	      //----- Hall A2 -----
	      if (getHallState2(motor) == GPIO_PIN_SET) { 				// *** hall 1 = 1 ***
	        if(hallState_prev_A2[motor] == 0){
	          if(++debounce_cntA[motor] > HALL_DEBOUNCE){ //ignore too short pulses (debouncing)
	        	bldc_cm->speed = hallCntB_S[motor];     //South pole magnet counter
	        	moving_counter[motor] = 400;
	            hallCntB_S[motor] = 0;
	            HallCntB[motor]++;
	            A_enc_new[motor] |= (1<<1);
	            debounce_cntA[motor] = 0;
	            hallState_prev_A2[motor] = 1;
	          }
	        }
	        else{ //hallState_prev_A2 == 1
	          debounce_cntA[motor]--;
	        }
	      } else {                         				// *** hall 1 = 0 ***
	        if(hallState_prev_A2[motor] == 1){
	          if(++debounce_cntA[motor] > HALL_DEBOUNCE){ //ignore too short pulses (debouncing)
	        	bldc_cm->speed = hallCntB_N[motor];     //North pole magnet counter
	        	moving_counter[motor] = 400;
	            hallCntB_N[motor] = 0;
	            A_enc_new[motor] &= ~(1<<1);
	            debounce_cntA[motor] = 0;
	            hallState_prev_A2[motor] = 0;
	          }
	        }
	        else{ //hallState_prev_A2 == 0
	          debounce_cntA[motor]--;
	        }
	      }
	    if(hallCntB_S[motor] < ZEROSPEED / 10)
	      hallCntB_S[motor]++;
	    else
	      bldc_cm->speed = ZEROSPEED;
	    if(hallCntB_N[motor] < ZEROSPEED / 10)
	      hallCntB_N[motor]++;
	    }
	  }


	/********************************************
	    pulse count on A axis ---
	*********************************************/
	  if(get_Hall_Enable(bldc_cm->index, HALL_ENABLED_BOTH)) {
	    switch (A_enc_old[motor]) {                                          //swap bits  0...0000 00xx
	      case 0:
	        A_enc_temp[motor]=0;
	        break;
	      case 1:
	        A_enc_temp[motor]=2;
	        break;
	      case 2:
	        A_enc_temp[motor]=1;
	        break;
	      case 3:
	        A_enc_temp[motor]=3;
	        break;
	    }
	  //--------------------
	    A_enc_old[motor]=A_enc_new[motor];        //new->old
	    A_enc_temp[motor]^=A_enc_new[motor];      //swapped_old EXOR new
	  //    BKP_WriteBackupRegister(BKP_DR6,H_enc_old);	            //backup - da reset virtualno ne steje impulza

	    switch (A_enc_temp[motor]){                                          // direction
	      case 0: break;                                              // 00 = no change
	      case 1: {
	        if(!(bldc_GetInvertHall(motor)))
	        	bldc_cm->position++;                                           // 01 = up
	          else
	        	bldc_cm->position--;
	          break;
	      }
	      case 2: {                                                   // 10 = down
	    	  if(!(bldc_GetInvertHall(motor)))
	        	bldc_cm->position--;                                           // 01 = up
	        else
	        	bldc_cm->position++;
	        break;
	      }
	      case 3:
	        break;                                                    // 11 = no change
	    }
	  }

	  if(get_Hall_Enable(motor, HALL_ENABLED_SINGLE)) {	//Count position when One Hall is used
	    if(A_enc_new[motor] != A_enc_old[motor]) {
	      if(bldc_cm->status & BLDC_STATUS_MOVING_OUT)
	    	  bldc_cm->position += 2;
	      else if (bldc_cm->status & BLDC_STATUS_MOVING_IN || bldc_cm->ctrl & BLDC_CTRL_HOMING)
	    	  bldc_cm->position -= 2;
	    }
	    A_enc_old[motor] = A_enc_new[motor];
	  }

	  /**********************************************************
	  	Hall loosing pulses detection
	  **********************************************************/
	  if(get_Hall_Enable(motor, HALL_ENABLED_BOTH)) {
	    if(HallCntA[motor] >= HallCntB[motor]) {
	      if((HallCntA[motor] - HallCntB[motor]) >= HALL_LOST_DETECT)
	        bldc_cm->status |= BLDC_STATUS_HALL_LOST_IMPULSES;
	    }
	    else
	      if((HallCntB[motor] - HallCntA[motor]) >= HALL_LOST_DETECT)
	    	  bldc_cm->status |= BLDC_STATUS_HALL_LOST_IMPULSES;
	  }

}


unsigned char get_Hall_Enable(uint8_t motor, uint8_t hall_A_B){
	if(hall_A_B == HALL_ENABLED_A){
		return hall_enable & (0x01 << motor);
	}
	if(hall_A_B == HALL_ENABLED_B){
		return hall_enable & (0x02 << motor);
	}
	if(hall_A_B == HALL_ENABLED_BOTH && ((hall_enable & (0x03 << motor))== 0x03 << motor)){
		return 1;
	}
	if(hall_A_B == HALL_ENABLED_SINGLE && ( ((hall_enable & (0x03 << motor))== 0x01 << motor) || ((hall_enable & (0x03 << motor))== 0x02 << motor)) ){
		return 1;
	}
	return 0;
}

























