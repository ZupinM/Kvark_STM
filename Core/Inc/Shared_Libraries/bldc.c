
#define BLDC_PA_POS (1<<5)
#define BLDC_PA_NEG (1<<1)
#define BLDC_PB_POS (1<<6)
#define BLDC_PB_NEG (1<<2)
#define BLDC_PC_POS (1<<7)
#define BLDC_PC_NEG (1<<3)

#define BLDC_CW_S1 1
#define BLDC_CW_S2 5
#define BLDC_CW_S3 4
#define BLDC_CW_S4 6
#define BLDC_CW_S5 2
#define BLDC_CW_S6 3

#define BLDC_CCW_S1 1
#define BLDC_CCW_S2 3
#define BLDC_CCW_S3 2
#define BLDC_CCW_S4 6
#define BLDC_CCW_S5 4
#define BLDC_CCW_S6 5


#define BLDC_CW_P1  (BLDC_PC_NEG | BLDC_PB_POS)
#define BLDC_CW_P2  (BLDC_PC_NEG | BLDC_PA_POS)
#define BLDC_CW_P3  (BLDC_PB_NEG | BLDC_PA_POS)
#define BLDC_CW_P4  (BLDC_PB_NEG | BLDC_PC_POS)
#define BLDC_CW_P5  (BLDC_PA_NEG | BLDC_PC_POS)
#define BLDC_CW_P6  (BLDC_PA_NEG | BLDC_PB_POS)


#define BLDC_CCW_P1 (BLDC_PA_NEG | BLDC_PB_POS)
#define BLDC_CCW_P2 (BLDC_PA_NEG | BLDC_PC_POS)
#define BLDC_CCW_P3 (BLDC_PB_NEG | BLDC_PC_POS)
#define BLDC_CCW_P4 (BLDC_PB_NEG | BLDC_PA_POS)
#define BLDC_CCW_P5 (BLDC_PC_NEG | BLDC_PA_POS)
#define BLDC_CCW_P6 (BLDC_PC_NEG | BLDC_PB_POS)

//Next CW number
const unsigned char bldc_cw_next[7][3] ={
                          {0,0,0},
                          {BLDC_CW_S2, BLDC_CW_P2, BLDC_CW_S3},
                          {BLDC_CW_S6, BLDC_CW_P6, BLDC_CW_S1},
                          {BLDC_CW_S1, BLDC_CW_P1, BLDC_CW_S2},
                          {BLDC_CW_S4, BLDC_CW_P4, BLDC_CW_S5},
                          {BLDC_CW_S3, BLDC_CW_P3, BLDC_CW_S4},
                          {BLDC_CW_S5, BLDC_CW_P5, BLDC_CW_S6}};

const unsigned char bldc_ccw_next[7][3] ={
                          {0,0,0},
                          {BLDC_CCW_S2, BLDC_CCW_P2, BLDC_CCW_S3},
                          {BLDC_CCW_S4, BLDC_CCW_P4, BLDC_CCW_S5},
                          {BLDC_CCW_S3, BLDC_CCW_P3, BLDC_CCW_S4},
                          {BLDC_CCW_S6, BLDC_CCW_P6, BLDC_CCW_S1},
                          {BLDC_CCW_S1, BLDC_CCW_P1, BLDC_CCW_S2},
                          {BLDC_CCW_S5, BLDC_CCW_P5, BLDC_CCW_S6}};

const unsigned char dc_cw_next[8] = {0, 0, 0, 0, 5, 7, 4, 6};
const unsigned char dc_ccw_next[8] = {0, 0, 0, 0, 6, 4, 7, 5};

//const 

#include "bldc.h"
#include "../main.h"
#include "../flash.h"
#include "../tim.h"

#include "focus.h"


#include "../gpio.h"

#include <math.h>
#include <stdlib.h>

#include "suntracer.h"

float        bldc_pwm;
int          bldc_runtime;
int          bldc_idletime;
int          bldc_status;
int          bldc_pause;


unsigned int  bldc_Voltage;
float         bldc_Uavg;
unsigned int  raw_Current;
float         bldc_Iavg;
float         zeroCurrent_voltage_0;
float         zeroCurrent_voltage_1;

float UVccHALL_0, UVccHALL_1;
float UVccHALL_0_avg, UVccHALL_1_avg;
float temperature;


volatile unsigned int           bldc_Speed;     //RPM
volatile unsigned int           number_of_poles = 2;
volatile unsigned int           speed_sample_counter;
volatile unsigned int           bldc_Speed_raw; //ticks from comutation to comutation

extern unsigned int tracker_status;

extern unsigned int store_in_flash;
volatile unsigned char phase_active;
volatile int mosfet_protection_cnt;

extern uint32_t adc3_VAL;
extern uint32_t adc4_VAL;
uint32_t adc3_SUM = 0;
uint32_t adc4_SUM = 0;
uint32_t adc_CNT = 0;

extern uint32_t cflags;

extern uint32_t SystemCoreClock;

unsigned int moving_counter[4] = {0, 0, 0, 0};
unsigned char any_motor_moving = 0;


unsigned char ES_0_normallyOpenLo = 0;
unsigned char ES_0_normallyOpenHi = 0;
unsigned char ES_1_normallyOpenLo = 0;
unsigned char ES_1_normallyOpenHi = 0;

unsigned char hall_fault = 0;
unsigned char hall_detect = 0;
unsigned int disconnected_motor[BLDC_MOTOR_COUNT];
unsigned char commutation_counter = 0;


float MOTOR_START_VOLTAGE;
float UNDERVOLTAGE_LEVEL;
float MOTOR_CUTOF_LEVEL; 


bldc_misc  bldc_cfg;
bldc_motor bldc_motors[BLDC_MOTOR_COUNT];            //motors
bldc_motor *bldc_cm = &bldc_motors[0];
bldc_motor blank_motor;
uint8_t motor_count = 1;

#define ADC_CONVERT_TICS 120

void ActivateDrivers(int dir);
void Flag_check();
void bldc_Comutate(unsigned char motor);

unsigned char ButtonStates() {
  unsigned char val = 0;

  if(HAL_GPIO_ReadPin(BT1_GPIO_Port, BT1_Pin) == GPIO_PIN_RESET)
    val |= 1<<0;
  if(HAL_GPIO_ReadPin(BT2_GPIO_Port, BT2_Pin) == GPIO_PIN_RESET)
    val |= 1<<1;
  #ifdef BT3_Pin
  if(HAL_GPIO_ReadPin(BT3_GPIO_Port, BT3_Pin) == GPIO_PIN_RESET)
    val |= 1<<2;
  #endif
  return (val & 0x07);
}

int bldc_ReadHall(unsigned char motor){

  int res = 0;  
  if (motor == 0) {
	  if(HAL_GPIO_ReadPin(HALL_A1_GPIO_Port, HALL_A1_Pin ) == GPIO_PIN_SET)
        res |= 1<<0;
	  if(HAL_GPIO_ReadPin(HALL_A2_GPIO_Port, HALL_A2_Pin ) == GPIO_PIN_SET)
        res |= 1<<1;
	  if(HAL_GPIO_ReadPin(HALL_A3_GPIO_Port, HALL_A3_Pin ) == GPIO_PIN_SET)
        res |= 1<<2;
  }
#if DEVICE == KVARK
  else if ( motor == 1){
	  if(HAL_GPIO_ReadPin(HALL_B1_GPIO_Port, HALL_B1_Pin ) == GPIO_PIN_SET)
        res |= 1<<0;
	  if(HAL_GPIO_ReadPin(HALL_B2_GPIO_Port, HALL_B2_Pin ) == GPIO_PIN_SET)
        res |= 1<<1;
	  if(HAL_GPIO_ReadPin(HALL_B3_GPIO_Port, HALL_B3_Pin ) == GPIO_PIN_SET)
        res |= 1<<2;
  }
#endif
  return ~res & 0x7;

}


void bldc_init_motors(int LoadDefaults)
{
  if(LoadDefaults) {
	motor_count = 2; //Default: 2 BLDC motors
    for(int i = 0; i < BLDC_MOTOR_COUNT; i++) {
      //init parameters
      bldc_motors[i].state           = 0;
      bldc_motors[i].index           = i;
      bldc_motors[i].ctrl            = 0;
      bldc_motors[i].position        = 0;
      bldc_motors[i].target          = 0;
      bldc_motors[i].status          = 0;
      bldc_motors[i].home_remaining  = 0;

      bldc_motors[i].gear_ratio      = 800;
      bldc_motors[i].I_limit         = 2;
      bldc_motors[i].I_Inrush_ratio  = 3; 
      bldc_motors[i].I_Inrush_time   = 500;
      bldc_motors[i].home_offset     = 0;
      bldc_motors[i].min_position    = 0;
      bldc_motors[i].max_position    = 800;
      bldc_motors[i].modbus_timeout_position = 400;
      bldc_motors[i].Idetection      = 0.07;
      bldc_motors[i].end_switchDetect = 10;

      bldc_motors[i].speed_freewheel = FREEWHEEL_SPEED_DEFAULT_BLDC;
      bldc_motors[i].speed = ZEROSPEED;
      bldc_motors[i].speed_old = ZEROSPEED;
      bldc_motors[i].ramp = 800;

      //init hall state
      bldc_motors[i].hall_state = bldc_ReadHall((unsigned char)i);

    }
  } else {
    for(int i=0 ;i < BLDC_MOTOR_COUNT; i++) {  
      bldc_motors[i].speed = ZEROSPEED;
      bldc_motors[i].speed_old = ZEROSPEED;
      bldc_motors[i].index           = i;
      bldc_motors[i].ctrl            = BLDC_CTRL_IDLE;
      bldc_motors[i].status          &= 0xf0 | BLDC_STATUS_ENDSWITCH_ERROR | BLDC_STATUS_ERR_MOVEOUT; 

      //range sanity check
      if(!isnormal(bldc_motors[i].max_position))
        bldc_motors[i].max_position = 0;
      if(!isnormal(bldc_motors[i].min_position))
        bldc_motors[i].min_position = 0;
      if(bldc_motors[i].max_position > 5000)
        bldc_motors[i].max_position = 0;
      if(bldc_motors[i].min_position < DEFAULT_MIN_RANGE)
        bldc_motors[i].min_position = 0;
      if(bldc_motors[i].min_position > bldc_motors[i].max_position) {
        bldc_motors[i].min_position = bldc_motors[i].max_position = 0;
      }

      //current sanity check
      if(!isnormal(bldc_motors[i].I_limit))
        bldc_motors[i].I_limit = 0;
      if(bldc_motors[i].I_limit<0 || bldc_motors[i].I_limit>10)
        bldc_motors[i].I_limit = 0;

      //home offset sanity check
      if(!isnormal(bldc_motors[i].home_offset))
        bldc_motors[i].home_offset = 0;
      if(bldc_motors[i].home_offset > bldc_motors[i].max_position)
        bldc_motors[i].home_offset = 0;
      if(bldc_motors[i].home_offset < bldc_motors[i].min_position)
        bldc_motors[i].home_offset = 0;

      //end switch sanity check
      if(!isnormal(bldc_motors[i].end_switchDetect))
        bldc_motors[i].end_switchDetect = 10;
      if(bldc_motors[i].end_switchDetect > bldc_motors[i].max_position)
        bldc_motors[i].end_switchDetect = 10;
      if(bldc_motors[i].end_switchDetect < 0)
        bldc_motors[i].end_switchDetect = 10;

      if(!isnormal(bldc_motors[i].Idetection))
        bldc_motors[i].Idetection = 0.07;
      if(!isnormal(bldc_motors[i].I_Inrush_ratio))
        bldc_motors[i].I_Inrush_ratio = 3;
      if(!isnormal(bldc_motors[i].I_Inrush_time))
        bldc_motors[i].I_Inrush_ratio = 500;

      
      if(number_of_poles < 1 || number_of_poles > 100)
        number_of_poles = 1;

      if (!isnormal(max_line_resistance))
        max_line_resistance = 0.56;

      bldc_motors[i].target = bldc_motors[i].position;

    }

    bldc_status &= ~(BLDC_LOCKED | BLDC_MOTOR_CUTOFF);
  }
}

void bldc_init_cfg(int LoadDefaults) {
  if (!LoadDefaults)
    return;
  bldc_cfg.IConvertRatio = 28;
  bldc_cfg.UConvertRatio = 28;
  bldc_cfg.BConvertRatio = 28;
  bldc_cfg.TConvertRatio = 28;
  bldc_cfg.HConvertRatio = 28;
  bldc_cfg.H1ConvertRatio = 28;
  bldc_cfg.homing_timeout = 600;
}




void bldc_init(int LoadDefaults){
  bldc_init_motors(LoadDefaults);
  bldc_init_cfg(LoadDefaults);

  bldc_pwm = 0;

//****initialize ports*******

/*
  NVIC_EnableIRQ(PIN_INT2_IRQn);
  NVIC_EnableIRQ(PIN_INT3_IRQn);
  NVIC_EnableIRQ(PIN_INT4_IRQn);
#ifdef MOTOR_B_HI1_PORT
  NVIC_EnableIRQ(PIN_INT5_IRQn);
  NVIC_EnableIRQ(PIN_INT6_IRQn);
  NVIC_EnableIRQ(PIN_INT7_IRQn);
#endif*/


//*****initialize timers********


  for(int i = 0; i < 4000000; i++);//wait for voltage samples

  //setup voltages
  if(GetAnalogValues(SUPPLY) >= 16) {//24V system
    MOTOR_START_VOLTAGE = MOTOR_START_VOLTAGE24;
    UNDERVOLTAGE_LEVEL  = UNDERVOLTAGE_LEVEL24;
    MOTOR_CUTOF_LEVEL   = MOTOR_CUTOF_LEVEL24;
  } else {          //12V system
    MOTOR_START_VOLTAGE= MOTOR_START_VOLTAGE12;
    UNDERVOLTAGE_LEVEL = UNDERVOLTAGE_LEVEL12;
    MOTOR_CUTOF_LEVEL  = MOTOR_CUTOF_LEVEL12;
  }

}

//******* PUBLIC CONTROL FUNCTIONS ******
void bldc_Lock(int state) {
  if(state)
    bldc_status |= BLDC_LOCKED;
  else
    bldc_status &= ~BLDC_LOCKED;
}

void Enable_ChargePump(uint8_t state){
  if(state == 1){
	  HAL_TIM_PWM_MspInit(&hChargePumpTIM);
  }else{
	  HAL_TIM_PWM_MspDeInit(&hChargePumpTIM);
  }
}

void bldc_Stop(int CancelManual){ //stop all motors

	bldc_SetDrivers(0,0);
	bldc_SetDrivers(0,1);

  hall_fault = 0;


  for(int i = 0; i < BLDC_MOTOR_COUNT; i++){

    uint8_t selected_motor = bldc_cm->index;

    if (CancelManual)
      bldc_cm = &bldc_motors[i];

    bldc_motors[i].status &= ~BLDC_STATUS_WIND_MODE;
    bldc_cm->target = bldc_cm->position;
     
    ActivateDrivers(0);

    bldc_cm->target = bldc_cm->position;
    bldc_motors[i].ctrl = BLDC_CTRL_STOP;
    HallCntA[i] = HallCntB[i] = 0;

    bldc_cm = &bldc_motors[selected_motor];	 //loads previuosly selected motor to finish comutation

  }
}


void bldc_ReleaseDrivers() {
  ActivateDrivers(0);
}

void bldc_ClearStatus() {
  bldc_status &= ~(BLDC_MOTOR_CUTOFF | BLDC_VOLTAGE_TO_LOW);
  for(int i = 0; i < BLDC_MOTOR_COUNT; i++){
    bldc_motors[i].status &= ~BLDC_STATUS_CLEARMASK;
    //bldc_motors[i].status = 0;
    bldc_motors[i].i_err_cnt = 0;
    bldc_EnableMotor(i,1);
    //bldc_Stop(1);
  }
}

int bldc_setPosition(unsigned char motor, float newpos, int windmode) { //go to position
  if(motor >= BLDC_MOTOR_COUNT)
    return -1;
  //bldc_cm = &bldc_motors[motor];
  bldc_motor *mptr = &bldc_motors[motor];

  if(mptr->status & BLDC_STATUS_ERR)
    return -2;                                                //motor in error state
  if(mptr->ctrl & (BLDC_CTRL_HOMING) && (!windmode) )
    return -3;                                                //cancel 

  if(bldc_status & BLDC_UNDERVOLTAGE) {
    bldc_status |= BLDC_VOLTAGE_TO_LOW;
    return -4;
  }

  if(mptr->status & BLDC_STATUS_MANUAL)
    return -5;                                                //controlled manually

  newpos += mptr->home_offset;

  if(newpos < mptr->min_position)
    newpos = mptr->min_position;                              //limit min position
  if(newpos > mptr->max_position)
    newpos = mptr->max_position;                              //limit max position
  mptr->target = newpos * mptr->gear_ratio;
  mptr->ctrl = BLDC_CTRL_TRACKING;

  //wind mode current limit *1.3
  if(windmode)
    bldc_cm->status |=  BLDC_STATUS_WIND_MODE;
  else
    bldc_cm->status &= ~BLDC_STATUS_WIND_MODE;

  Flag_check();
  return  0;
}

int bldc_setPositionImp(unsigned char motor, int newposImp, int windmode){ //go to position
  if(motor >= BLDC_MOTOR_COUNT) 
    return -1;
  bldc_motor *mptr = &bldc_motors[motor];

  if(mptr->status & BLDC_STATUS_ERR) 
    return -2;                    //motor in error state
  if(mptr->ctrl & (BLDC_CTRL_HOMING) && (!windmode)) 
    return -3; //cancel 

  if(bldc_status & BLDC_UNDERVOLTAGE) {
    bldc_status |= BLDC_VOLTAGE_TO_LOW;
    return -4;
  }

  if(mptr->status & BLDC_STATUS_MANUAL) 
    return -5;                 //controlled manually

  newposImp += mptr->home_offset / mptr->gear_ratio;

  if(newposImp < (mptr->min_position * mptr->gear_ratio)) 
    newposImp = mptr->min_position * mptr->gear_ratio; //limit min position
  if(newposImp > (mptr->max_position * mptr->gear_ratio)) 
    newposImp = mptr->max_position * mptr->gear_ratio; //limit max position
  mptr->target = newposImp;
  mptr->ctrl = BLDC_CTRL_TRACKING;

  //wind mode current limit *1.3
  if(windmode) 
    bldc_cm->status |=  BLDC_STATUS_WIND_MODE;
  else         
    bldc_cm->status &= ~BLDC_STATUS_WIND_MODE;

  Flag_check();
  return  0;
}

int bldc_Home(unsigned char motor) {  //execute homing
  if(motor>=BLDC_MOTOR_COUNT)
    return -1;

  bldc_cm = &bldc_motors[motor];
  bldc_motor *mptr = &bldc_motors[motor];

  if(mptr->status& BLDC_STATUS_ERR)
    return -2;                        //motor in error state
  if(bldc_status&BLDC_UNDERVOLTAGE) {
    bldc_status |= BLDC_VOLTAGE_TO_LOW;
    return-4;
  }
  mptr->homing_time = 0;
  mptr->ctrl = BLDC_CTRL_HOMING;
  bldc_cm->status &= ~BLDC_STATUS_WIND_MODE;
  Flag_check();
  return 0;
}

void bldc_EnableMotors(unsigned int state){
  unsigned int newstate=state>>16;

  for(int i = 0; i < BLDC_MOTOR_COUNT; i++)
    if(state & (1 << i)) {         //axis 
      if(newstate&(1<<i))
        bldc_motors[i].state |= BLDC_MOTOR_STATE_ENABLED;
      else
        bldc_motors[i].state &= ~BLDC_MOTOR_STATE_ENABLED;
    }   					
}

void bldc_EnableMotor(unsigned char motor, unsigned char state){
	if(state == 1)
		bldc_motors[motor].state |= BLDC_MOTOR_STATE_ENABLED;
	if(state == 0)
		bldc_motors[motor].state &= ~BLDC_MOTOR_STATE_ENABLED;
}

unsigned int bldc_GetEnabledMotors(){
  unsigned int state = 0;

  for(int i = 0; i < BLDC_MOTOR_COUNT; i++){
    state |= 1 << (16 + i);
    if(bldc_motors[i].state&BLDC_MOTOR_STATE_ENABLED)
      state |= 1 << i; 
  }
  return state;
}

int bldc_Enabled(unsigned char motor) {
  if(motor >= BLDC_MOTOR_COUNT)
    return 0 ;
  bldc_motor *mptr = &bldc_motors[motor];  

  return mptr->state&BLDC_MOTOR_STATE_ENABLED ? 1: 0;
}

void bldc_SetInvert(unsigned char motor, unsigned int state){
  if(motor >= BLDC_MOTOR_COUNT)
    return;
  bldc_motor *mptr = &bldc_motors[motor];  

  bldc_manual(1);  // mzp
  bldc_Stop(1);
  bldc_runout(RUNOUT_ACTIVATE);

  if(state){
    mptr->state |= BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST;
    mptr->state |= BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST_STATE;
//    if(motor) cflags |= 1<<SwapRotation_A;
//    else cflags |= 1<<SwapRotation_B;
  }
  else{
    mptr->state |= BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST;
    mptr->state &= ~BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST_STATE;
//    if(motor) cflags &= ~(1<<SwapRotation_A);
//    else      cflags &= ~(1<<SwapRotation_B);
  }  
}

unsigned int bldc_GetInvert(unsigned char motor) {
  if(motor >= BLDC_MOTOR_COUNT)
    return 0 ;
   bldc_motor *mptr = &bldc_motors[motor];  

  return mptr->state&BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST_STATE ? 1: 0;
}

void bldc_SetInvertHall(unsigned char motor, unsigned int state) {
  if(motor >= BLDC_MOTOR_COUNT)
    return;
  bldc_motor *mptr = &bldc_motors[motor];  

  bldc_Stop(0);
  bldc_runout(RUNOUT_ACTIVATE);


  if(state){
    mptr->state |= BLDC_MOTOR_STATE_INVERT_HALL_REQUEST;
    mptr->state |= BLDC_MOTOR_STATE_INVERT_HALL_REQUEST_STATE;
  }else{
    mptr->state |= BLDC_MOTOR_STATE_INVERT_HALL_REQUEST;
    mptr->state &= ~BLDC_MOTOR_STATE_INVERT_HALL_REQUEST_STATE;

  }
}

unsigned int bldc_GetInvertHall(unsigned char motor) {
  if(motor >= BLDC_MOTOR_COUNT)
    return 0 ;
  bldc_motor *mptr = &bldc_motors[motor];  

  return   mptr->state&BLDC_MOTOR_STATE_INVERT_HALL_REQUEST_STATE ? 1: 0;
}

int bldc_position_to_pulses(unsigned char motor, float pos) {
  if(motor >= BLDC_MOTOR_COUNT)
    return 0;
  bldc_motor *mptr = &bldc_motors[motor];
  return  pos * mptr->gear_ratio;
}

float gpio_U(){
  return GetAnalogValues(SUPPLY);
}

int m_A_idle(){
  return ((abs(bldc_motors[0].target - bldc_motors[0].position) < 100 && (!(bldc_motors[0].state & BLDC_STATUS_ACTIVE))))?1:0;	
}

int m_B_idle(){
  return ((abs(bldc_motors[1].target - bldc_motors[1].position) < 100 && (!(bldc_motors[1].state & BLDC_STATUS_ACTIVE))))?1:0;	
}

int m_idle(){
  if(m_A_idle() && m_B_idle()) return 1;
  return 0;
}

int m_referencing(){
 if((bldc_motors[0].state & BLDC_STATUS_HOMING)||(bldc_motors[1].state & BLDC_STATUS_HOMING)) return 1;
 return 0;
}

int   motorA_moving()
{
  	return (bldc_motors[0].state & BLDC_STATUS_ACTIVE && bldc_cm->index == 0)?1:0;
}

int   motorB_moving()
{
 	return (bldc_motors[1].state & BLDC_STATUS_ACTIVE && bldc_cm->index == 1)?1:0;
} 

int bldc_getStatus(unsigned char motor) {
  if(motor>BLDC_MOTOR_COUNT)
    return 0;
  return bldc_motors[motor].status;
}

int bldc_Status() {
  return  bldc_status;
}

void bldc_initStatus(unsigned int status) {
  bldc_status=status;
}

int bldc_Active(unsigned char motor) {
  if(motor>BLDC_MOTOR_COUNT)
    return 0;
  return bldc_motors[motor].status & BLDC_STATUS_ACTIVE;
}

void bldc_manual(int enable) {
  for(int i = 0; i < BLDC_MOTOR_COUNT; i++)   
    if(enable)
      bldc_motors[i].status |=  BLDC_STATUS_MANUAL;
    else
      bldc_motors[i].status &= ~BLDC_STATUS_MANUAL;
}


uint8_t ignore_setpoint = 0;

void bldc_runout(int state){

  if(state == RUNOUT_ACTIVATE){   //Activate free runout of motor
    ignore_setpoint = 1;   
  }

  if(ignore_setpoint){                         //motor runing out (RUNOUT_FREEWHEEL)
    bldc_cm->target = bldc_cm->position;
    if(!(bldc_cm->status & BLDC_STATUS_MOVING)){
      ignore_setpoint = 0;                     //deactivate motor runout, once stopped
      bldc_cm->ctrl   = BLDC_CTRL_IDLE;        //deactivate homing
      ActivateDrivers(0);
    }
  }   
   
}

float bldc_position(unsigned char motor) {
  if(motor>=BLDC_MOTOR_COUNT)
    return -0xfff;
  bldc_motor *mptr = &bldc_motors[motor];

  return (float)mptr->position / (float)mptr->gear_ratio;
}

int bldc_positionImp(unsigned char motor){
  if(motor>=BLDC_MOTOR_COUNT) 
    return -0xfff;;
  bldc_motor *mptr = &bldc_motors[motor];
 
  return mptr->position;
}

float bldc_target(unsigned char motor) {
  if(motor>=BLDC_MOTOR_COUNT)
    return -0xfff;
  bldc_motor *mptr = &bldc_motors[motor];

  return (float)mptr->target / (float)mptr->gear_ratio;
}

int bldc_targetImp(unsigned char motor){
  if(motor>=BLDC_MOTOR_COUNT) 
    return -0xfff;
  bldc_motor *mptr = &bldc_motors[motor];

  return mptr->target;
}

float bldc_remaining(unsigned char motor) {
  if(motor>=BLDC_MOTOR_COUNT)
    return -0xfff;
  bldc_motor *mptr = &bldc_motors[motor];

  return (float)mptr->home_remaining / (float)mptr->gear_ratio;
}

int bldc_remainingImp(unsigned char motor) {
  if(motor>=BLDC_MOTOR_COUNT) 
    return -0xfff;
  bldc_motor *mptr = &bldc_motors[motor];

  return mptr->home_remaining;
}

bldc_motor *bldc_Motor(unsigned char motor) {
  if(motor>=BLDC_MOTOR_COUNT){
    memset(&blank_motor, 0, sizeof(blank_motor));
    return &blank_motor;
  }
    //return (void*)(0); //Dereferencing NULL pointer is undefined behaviour!!!
  return &bldc_motors[motor];
}

bldc_misc * bldc_config() {
  return &bldc_cfg;
}

int old_motor_mode[BLDC_MOTOR_COUNT];

void Flag_check() {
  int val;
  for(int i = 0; i < BLDC_MOTOR_COUNT; i++) {
    bldc_motors[i].status &= ~(BLDC_STATUS_MOVING_OUT | BLDC_STATUS_MOVING_IN | BLDC_STATUS_HOMING | BLDC_STATUS_ENDSWITCH_LOW_ACTIVE | BLDC_STATUS_ENDSWITCH_HIGH_ACTIVE);
  
    bldc_motors[i].status |=  bldc_HomeSwitchActive(i,0) ? BLDC_STATUS_ENDSWITCH_LOW_ACTIVE: 0;
    bldc_motors[i].status |=  bldc_HomeSwitchActive(i,1) ? BLDC_STATUS_ENDSWITCH_HIGH_ACTIVE: 0;

    val = (bldc_motors[i].target - bldc_motors[i].position);

    /*if(val >= (-bldc_motors[i].pid.deadband -bounce_stop) && val <= (bldc_motors[i].pid.deadband + bounce_stop))
      bldc_motors[i].status &= ~(BLDC_STATUS_MOVING_OUT | BLDC_STATUS_MOVING_IN);
    else*/ if(bldc_motors[i].status & BLDC_STATUS_MOVING) {
      if(val>0) {

        bldc_motors[i].status |= BLDC_STATUS_MOVING_OUT;
        bldc_motors[i].status &= ~BLDC_STATUS_ERR_MOVEOUT;
      } else if(val < 0) {

        bldc_motors[i].status |= BLDC_STATUS_MOVING_IN;
        bldc_motors[i].status &= ~BLDC_STATUS_ERR_MOVEOUT;
      }
      if(bldc_motors[i].ctrl == BLDC_CTRL_HOMING)
        bldc_motors[i].status|= BLDC_STATUS_HOMING;
    }
#if DEVICE != PICO
    if((bldc_motors[i].state & BLDC_MOTOR_STATE_DC_OPERATION) != old_motor_mode[i]){
    	set_external_INT(i, !(bldc_motors[i].state & BLDC_MOTOR_STATE_DC_OPERATION));
    }
    old_motor_mode[i] = bldc_motors[i].state & BLDC_MOTOR_STATE_DC_OPERATION;
#endif
  }

  if(!(bldc_motors[0].status & BLDC_STATUS_MOVING_IN) && !(bldc_motors[0].status & BLDC_STATUS_MOVING_OUT) &&
    !(bldc_motors[1].status & BLDC_STATUS_MOVING_IN) && !(bldc_motors[1].status & BLDC_STATUS_MOVING_OUT))
    bldc_Speed = 0;

}

void bldc_update_pwm(unsigned short value) {
  /* Set the pulse value for channel 1, 2, 3 */
    //if(cnt_print++ > 9){
    		//uint32_t tmp1 = bldc_cm->state & BLDC_MOTOR_STATE_BRAKING;
  		  //SEGGER_RTT_printf(0, "c%d e%d o%d\r\n", tmp1, (bldc_cm->target-bldc_cm->position), value);
  		  //cnt_print = 0;
   // }
  __HAL_TIM_SetCompare (&htim1, TIM_CHANNEL_1, value);
  __HAL_TIM_SetCompare (&htim1, TIM_CHANNEL_2, value);
  __HAL_TIM_SetCompare (&htim1, TIM_CHANNEL_3, value);
}


int bldc_HomeSwitchActive(unsigned char motor , unsigned char switch_h_l) {
#if DEVICE != PICO
  if(bldc_cm->state & BLDC_MOTOR_STATE_DC_OPERATION){	// DC Motors --One Low Switch per Port
	  if(motor == 0 || motor == 2){

		if(!ES_0_normallyOpenLo && !switch_h_l)
		  return(!HAL_GPIO_ReadPin(END_SW_A_HI_GPIO_Port, END_SW_A_HI_Pin));
		else if(ES_0_normallyOpenLo && !switch_h_l)
			return(HAL_GPIO_ReadPin(END_SW_A_HI_GPIO_Port, END_SW_A_HI_Pin));
	  }

	  if(motor == 1 || motor == 3){
		if(!ES_1_normallyOpenLo && !switch_h_l)
			return(!HAL_GPIO_ReadPin(END_SW_B_HI_GPIO_Port, END_SW_B_HI_Pin));
		else if(ES_1_normallyOpenLo && !switch_h_l)
			return(HAL_GPIO_ReadPin(END_SW_B_HI_GPIO_Port, END_SW_B_HI_Pin));
	  }
	  return 0;
  }
#endif


  if (motor == 0){										//BLDC Motors -- Low and High switch for every motor
#if DEVICE != PICO
    if(!ES_0_normallyOpenLo && !switch_h_l)
      return(HAL_GPIO_ReadPin(END_SW_A_LO_GPIO_Port, END_SW_A_LO_Pin));
    else if(ES_0_normallyOpenLo && !switch_h_l)
    	return(!HAL_GPIO_ReadPin(END_SW_A_LO_GPIO_Port, END_SW_A_LO_Pin));
#endif
    if(!ES_0_normallyOpenHi && switch_h_l)
    	return(!HAL_GPIO_ReadPin(END_SW_A_HI_GPIO_Port, END_SW_A_HI_Pin));
    else if(ES_0_normallyOpenHi && switch_h_l)
    	return(HAL_GPIO_ReadPin(END_SW_A_HI_GPIO_Port, END_SW_A_HI_Pin));
  }
#if DEVICE != PICO
  else if (motor == 1){
    if(!ES_1_normallyOpenLo && !switch_h_l)
    	return(HAL_GPIO_ReadPin(END_SW_B_LO_GPIO_Port, END_SW_B_LO_Pin));
    else if(ES_1_normallyOpenLo && !switch_h_l)
    	return(!HAL_GPIO_ReadPin(END_SW_B_LO_GPIO_Port, END_SW_B_LO_Pin));
    else if(!ES_1_normallyOpenHi && switch_h_l)
    	return(!HAL_GPIO_ReadPin(END_SW_B_HI_GPIO_Port, END_SW_B_HI_Pin));
    else if(ES_1_normallyOpenHi && switch_h_l)
    	return(HAL_GPIO_ReadPin(END_SW_B_HI_GPIO_Port, END_SW_B_HI_Pin));
  }
#endif
  return 0;
}

void setGPIO_Function(GPIO_TypeDef  *GPIOx, uint16_t GPIO_Pin, uint8_t mode){
	uint32_t temp = GPIOx->MODER;
	temp &= ~(GPIO_Pin*GPIO_Pin*0x3); // 00-Input mode
	temp |= GPIO_Pin*GPIO_Pin * mode;
	GPIOx->MODER = temp;
}

void bldc_SetDrivers(unsigned char NewState, unsigned char motor){
  //disable pwm drivers
  phase_active = 0;
  mosfet_protection_cnt = 0;

  // WDT reset 
  //LPC_WWDT->FEED = 0xAA;
  //LPC_WWDT->FEED = 0x55;


  if(motor==1){
#if DEVICE != PICO

	    //disable low side
	    if(!(NewState & BLDC_PA_NEG))
	    	HAL_GPIO_WritePin(MOTOR_B_1L_GPIO_Port, MOTOR_B_1L_Pin, GPIO_PIN_RESET);
	    if(!(NewState & BLDC_PB_NEG))
	    	HAL_GPIO_WritePin(MOTOR_B_2L_GPIO_Port, MOTOR_B_2L_Pin, GPIO_PIN_RESET);
	    if(!(NewState & BLDC_PC_NEG))
	    	HAL_GPIO_WritePin(MOTOR_B_3L_GPIO_Port, MOTOR_B_3L_Pin, GPIO_PIN_RESET);

	    //activate pwm high side driver
	    switch(NewState & 0xf0){
	      case BLDC_PA_POS:
	    	setGPIO_Function(MOTOR_B_1H_GPIO_Port, MOTOR_B_1H_Pin, MODE_ALTERNATE);
	    	setGPIO_Function(MOTOR_B_2H_GPIO_Port, MOTOR_B_2H_Pin, MODE_OUTPUT);
	    	setGPIO_Function(MOTOR_B_3H_GPIO_Port, MOTOR_B_3H_Pin, MODE_OUTPUT);
	        break;
	      case BLDC_PB_POS:
	      	setGPIO_Function(MOTOR_B_1H_GPIO_Port, MOTOR_B_1H_Pin, MODE_OUTPUT);
	      	setGPIO_Function(MOTOR_B_2H_GPIO_Port, MOTOR_B_2H_Pin, MODE_ALTERNATE);
	      	setGPIO_Function(MOTOR_B_3H_GPIO_Port, MOTOR_B_3H_Pin, MODE_OUTPUT);
	        break;
	      case BLDC_PC_POS:
	      	setGPIO_Function(MOTOR_B_1H_GPIO_Port, MOTOR_B_1H_Pin, MODE_OUTPUT);
	      	setGPIO_Function(MOTOR_B_2H_GPIO_Port, MOTOR_B_2H_Pin, MODE_OUTPUT);
	      	setGPIO_Function(MOTOR_B_3H_GPIO_Port, MOTOR_B_3H_Pin, MODE_ALTERNATE);
	        break;
	    }

	    if(!(bldc_cm->state & BLDC_MOTOR_STATE_BRAKING)){
			//activate low side driver
			switch(NewState & 0x0f){
			  case BLDC_PA_NEG:
				  HAL_GPIO_WritePin(MOTOR_B_1L_GPIO_Port, MOTOR_B_1L_Pin, GPIO_PIN_SET); phase_active++; break;// A L L1
			  case BLDC_PB_NEG:
				  HAL_GPIO_WritePin(MOTOR_B_2L_GPIO_Port, MOTOR_B_2L_Pin, GPIO_PIN_SET); phase_active++; break;// A L L2
			  case BLDC_PC_NEG:
				  HAL_GPIO_WritePin(MOTOR_B_3L_GPIO_Port, MOTOR_B_3L_Pin, GPIO_PIN_SET); phase_active++; break;// A L L3
			}
	    }
#endif
   }

  else if (motor == 0){

    //disable low side
    if(!(NewState & BLDC_PA_NEG))
    	HAL_GPIO_WritePin(MOTOR_A_1L_GPIO_Port, MOTOR_A_1L_Pin, GPIO_PIN_RESET);
    if(!(NewState & BLDC_PB_NEG))
    	HAL_GPIO_WritePin(MOTOR_A_2L_GPIO_Port, MOTOR_A_2L_Pin, GPIO_PIN_RESET);
    if(!(NewState & BLDC_PC_NEG))
    	HAL_GPIO_WritePin(MOTOR_A_3L_GPIO_Port, MOTOR_A_3L_Pin, GPIO_PIN_RESET);

    //activate pwm high side driver
    switch(NewState & 0xf0){
      case BLDC_PA_POS:
    	setGPIO_Function(MOTOR_A_1H_GPIO_Port, MOTOR_A_1H_Pin, MODE_ALTERNATE);
    	setGPIO_Function(MOTOR_A_2H_GPIO_Port, MOTOR_A_2H_Pin, MODE_OUTPUT);
    	setGPIO_Function(MOTOR_A_3H_GPIO_Port, MOTOR_A_3H_Pin, MODE_OUTPUT);
        break;
      case BLDC_PB_POS:
      	setGPIO_Function(MOTOR_A_1H_GPIO_Port, MOTOR_A_1H_Pin, MODE_OUTPUT);
      	setGPIO_Function(MOTOR_A_2H_GPIO_Port, MOTOR_A_2H_Pin, MODE_ALTERNATE);
      	setGPIO_Function(MOTOR_A_3H_GPIO_Port, MOTOR_A_3H_Pin, MODE_OUTPUT);
        break;
      case BLDC_PC_POS:
      	setGPIO_Function(MOTOR_A_1H_GPIO_Port, MOTOR_A_1H_Pin, MODE_OUTPUT);
      	setGPIO_Function(MOTOR_A_2H_GPIO_Port, MOTOR_A_2H_Pin, MODE_OUTPUT);
      	setGPIO_Function(MOTOR_A_3H_GPIO_Port, MOTOR_A_3H_Pin, MODE_ALTERNATE);
        break;
    }

    if(!(bldc_cm->state & BLDC_MOTOR_STATE_BRAKING)){
    //activate low side driver
		switch(NewState & 0x0f){
		  case BLDC_PA_NEG:
			  HAL_GPIO_WritePin(MOTOR_A_1L_GPIO_Port, MOTOR_A_1L_Pin, GPIO_PIN_SET); phase_active++; break;// A L L1
		  case BLDC_PB_NEG:
			  HAL_GPIO_WritePin(MOTOR_A_2L_GPIO_Port, MOTOR_A_2L_Pin, GPIO_PIN_SET); phase_active++; break;// A L L2
		  case BLDC_PC_NEG:
			  HAL_GPIO_WritePin(MOTOR_A_3L_GPIO_Port, MOTOR_A_3L_Pin, GPIO_PIN_SET); phase_active++; break;// A L L3
		}
    }
  }

}


void ActivateDrivers(int dir) {
  if (bldc_cm->status & BLDC_STATUS_ACTIVE) {
    bldc_idletime = 0;
    bldc_runtime++;
    if(dir != 0)
      return;
    bldc_update_pwm(0);
    bldc_SetDrivers(0,bldc_cm->index);  //stop
    Enable_ChargePump(0);
    bldc_cm->status &= ~(BLDC_STATUS_ACTIVE | BLDC_STATUS_MANUAL | BLDC_STATUS_WIND_MODE);

  } else {        // only on 1.st entry --- next one is hall comutated
    if(dir == 0)   
      return;
    
    Enable_ChargePump(1);
    bldc_runtime = 0;
    bldc_pwm = 0;
    bldc_cm->status |= BLDC_STATUS_ACTIVE;
    
    if(!(bldc_cm->state & BLDC_MOTOR_STATE_DC_OPERATION))
      bldc_Comutate(bldc_cm->index);
   // else
      //dc_Comutate(bldc_cm->index, bldc_ReadHall(bldc_cm->index));
  }
}

unsigned int motor_err(uint8_t motor)
{
    return 1; //bldc_motors[motor].target - bldc_motors[motor].position;
}

int marginPulses;
int startPulses;
int halfPos;
int breaking = 0;


unsigned int  bldc_recovery_overvoltage=0;
unsigned int  bldc_recovery_undervoltage=0;
unsigned int  bldc_recovery_timeout=0;
unsigned int  bldc_undervoltage_timer=0;
unsigned int  bldc_overvoltageDetectDelay=0;

void voltage_detection() {
  float voltage = GetAnalogValues(SUPPLY);
  UVccHALL_0 = GetAnalogValues(HALL);

  bldc_recovery_timeout++;
  bldc_recovery_undervoltage++;
  bldc_recovery_overvoltage++;

  //  	//420=15V, 450=16V, 480=17V, 510=18V  770=27.5 812=29V
  //Overvoltage detection
  if(voltage >= 32.0) {
    if(bldc_overvoltageDetectDelay++ >= 1000) {
      bldc_recovery_overvoltage = 0;
      bldc_overvoltageDetectDelay = 1000;
      bldc_status |= BLDC_OVERVOLTAGE;
    }
  } else if((voltage < 30.0) && (bldc_recovery_overvoltage >= 10000)) {
    bldc_status &= ~BLDC_OVERVOLTAGE;
    bldc_overvoltageDetectDelay = 0;
  }

  //Undervoltage detection
  if (voltage < UNDERVOLTAGE_LEVEL) { //undervoltage	
    bldc_undervoltage_timer++;
    bldc_recovery_undervoltage=0;
    bldc_status |= BLDC_UNDERVOLTAGE;
  } else if ((voltage >= (UNDERVOLTAGE_LEVEL)) && (bldc_recovery_undervoltage >= 5000)) {	
    bldc_status &= ~BLDC_UNDERVOLTAGE;
  }	

  if(voltage < MOTOR_CUTOF_LEVEL){
    if(bldc_cm->status & BLDC_STATUS_ACTIVE) bldc_status |= BLDC_MOTOR_CUTOFF;
    bldc_Stop(1);
  }
	
  //turn motors off
  if(voltage <= SHUTDOWN_VOLTAGE) {
    bldc_status |= BLDC_SHUTTING_DOWN;
    bldc_recovery_timeout = 0;

    /*if ((bldc_status&(1<<flash_erase_done))&&(!(bldc_status&(1<<flash_write_done)))) {
            if(undervoltage_timer<=200)tracker_exstatus &=~EFS_UNDERVOLTAGE;
            flash_write(SYS_VARS_ADDR);
            bldc_status|=(1<<flash_write_done);
    }*/

  } else if( voltage>MOTOR_CUTOF_LEVEL && bldc_status&BLDC_SHUTTING_DOWN) { //15.0v 10s
    bldc_recovery_timeout++;
    if(bldc_recovery_timeout > 10000) {
        bldc_status &= ~BLDC_SHUTTING_DOWN;	
    }
  }
}

uint32_t max_I_A, max_I_B;

uint8_t HallState_prev1[BLDC_MOTOR_COUNT];
uint8_t HallState_prev2[BLDC_MOTOR_COUNT];


//MOTOR core control function
void bldc_process() {

  bldc_runout(RUNOUT_FREEWHEEL);

  if (bldc_GetInvertHall(0))
             cflags |= 1<<swap_halls_A;
  else cflags &= ~(1<<swap_halls_A);

  if (bldc_GetInvertHall(1))
             cflags |= 1<<swap_halls_B;
  else cflags &= ~(1<<swap_halls_B);	


  if (bldc_GetInvert(0))
       cflags |= 1<<SwapRotation_A;
  else cflags &= ~(1<<SwapRotation_A);

  if (bldc_GetInvert(1)) 
       cflags |= 1<<SwapRotation_B;
  else cflags &= ~(1<<SwapRotation_B);   


  any_motor_moving = 0;
  for(int i=0 ; i<BLDC_MOTOR_COUNT ; i++){

    if(moving_counter[i]){                          // BLDC_STATUS_MOVING
      bldc_motors[i].status |= BLDC_STATUS_MOVING;
      moving_counter[i] --;
      any_motor_moving = 1;
    }
    else{
      bldc_motors[i].status &= ~BLDC_STATUS_MOVING; 
    } 

    //check if BLDC motor is disconnected
    if(!(bldc_motors[i].state & BLDC_MOTOR_STATE_DC_OPERATION) ){
		if(bldc_ReadHall(i) == 7 && bldc_motors[i].status & BLDC_STATUS_ACTIVE){
		  disconnected_motor[i]++;
		  if (disconnected_motor[i] > 50){
			bldc_motors[i].status |= BLDC_STATUS_HALL_FAULT;
			//bldc_motors[0].status |= BLDC_STATUS_CABLEERROR;
			bldc_EnableMotor(i,0);
		  }
		}else if(disconnected_motor[i]){
		  disconnected_motor[i]--;
		  if(disconnected_motor[i] == 0){
			bldc_motors[i].status &= ~BLDC_STATUS_HALL_FAULT;
		  }
		}
    }

	if(bldc_motors[i].state & BLDC_MOTOR_STATE_DC_OPERATION  && bldc_motors[i].status & BLDC_STATUS_ACTIVE){	//DC Hall check
		  if((getHallState1(i) != HallState_prev1[i] || !(hall_enable & (0x01 << 8 * i)) ) && ( getHallState2(i) != HallState_prev2[i] || !(hall_enable & (0x02 << 8 * i)))){
			  HallState_prev1[i] = getHallState1(i);
			  HallState_prev2[i] = getHallState2(i);
			  disconnected_motor[i] = 0;
			  bldc_motors[i].status &= ~BLDC_STATUS_HALL_FAULT;
		  }
		  else if(++disconnected_motor[i] > 2000){
			bldc_motors[i].status |= BLDC_STATUS_HALL_FAULT;
			bldc_EnableMotor(i,0);
			disconnected_motor[i] = 0;
		  }
	}
  }

//switch motors when current motor is finished
#if BLDC_MOTOR_COUNT > 1
  if( (target_error(bldc_cm->index) < BLDC_DEADBAND) && (!any_motor_moving)){
  	for(int m=0 ; m<=BLDC_MOTOR_COUNT ; m++){
  		if(target_error(m) > BLDC_DEADBAND && (bldc_cm->index != m)){
  			bldc_cm = &bldc_motors[m];
  			bldc_cm->ctrl = BLDC_CTRL_TRACKING;
  		}
  	}
  }
#endif

  voltage_detection();
  Flag_check();

 // bldc_motor *bldc_cm = &bldc_motors[1];

  //***control disabled error state*** 
 if(bldc_status & BLDC_LOCKED || !(bldc_cm->state & BLDC_MOTOR_STATE_ENABLED) || bldc_cm->status & BLDC_STATUS_ERR) {
    ActivateDrivers(0);
    bldc_pause = 0;
    bldc_cm->ctrl = BLDC_CTRL_IDLE;
    bldc_cm->target = bldc_cm->position;
    return;
  }

  //****over current detection****
  float ILimit;
  if (bldc_runtime < bldc_cm->I_Inrush_time)
    ILimit = bldc_cm->I_limit *  bldc_cm->I_Inrush_ratio;
  else if(bldc_cm->status & BLDC_STATUS_WIND_MODE)
    ILimit = bldc_cm->I_limit *  1.3;
  else
    ILimit = bldc_cm->I_limit;
  
  if( GetAnalogValues(MotorSelectI(bldc_cm->index)) >= ILimit) {
    SetEventParameters(bldc_cm->index);
  
    ActivateDrivers(0); //stop motor now
  
    bldc_cm->i_err_cnt++;
    if(bldc_cm->i_err_cnt>=1) bldc_cm->status |= BLDC_STATUS_ERR | BLDC_STATUS_OVERCURRENT;
    else bldc_pause = 3 * 1000; //5s timeout

    return;
  }

//****cable error detection****
    if(bldc_cm->status & BLDC_STATUS_ACTIVE && bldc_cm->status & BLDC_STATUS_STALL && abs(bldc_pwm)>=0xa00){
      SetEventParameters(bldc_cm->index);
      ActivateDrivers(0);
      bldc_cm->status |= BLDC_STATUS_ERR | BLDC_STATUS_CABLEERROR;
      return;
    }

  //****end switch error detection****
  if(bldc_HomeSwitchActive(bldc_cm->index,0) && bldc_cm->ctrl == BLDC_CTRL_TRACKING && bldc_cm->position > bldc_position_to_pulses(bldc_cm->index, bldc_cm->end_switchDetect - 0.1)){
    SetEventParameters(bldc_cm->index);
    //ActivateDrivers(0);
    bldc_cm->status|= BLDC_STATUS_ENDSWITCH_ERROR;
    //return;
  }
  if(motor_count < 3){
	  if(bldc_HomeSwitchActive(bldc_cm->index,1) && bldc_cm->ctrl == BLDC_CTRL_TRACKING && bldc_cm->position <  bldc_position_to_pulses(bldc_cm->index, (bldc_cm->max_position -bldc_cm->end_switchDetect - 0.1)) ){
		SetEventParameters(bldc_cm->index);
		//ActivateDrivers(0);
		bldc_cm->status|= BLDC_STATUS_ENDSWITCH_ERROR;
		//return;
	  }
  }
  //****end switch detection****

  if(bldc_HomeSwitchActive(bldc_cm->index,0) && bldc_cm->ctrl & BLDC_CTRL_HOMING) {
    ActivateEvent(EVENT_HOME_A_FINISHED);
    bldc_cm->ctrl = BLDC_CTRL_IDLE;
    bldc_cm->home_remaining = bldc_cm->position;
    bldc_cm->position = bldc_cm->target = 0;
     bldc_manual(1);
    bldc_Stop(1);
    bldc_runout(RUNOUT_ACTIVATE);
    return;
  }

  if(bldc_HomeSwitchActive(bldc_cm->index,0) && bldc_cm->target < bldc_cm->position){
    bldc_cm->target = bldc_cm->position;
    bldc_cm->ctrl = BLDC_CTRL_STOP;
    ActivateDrivers(0);
    return;
  }
  if(motor_count < 3){
	  if(bldc_HomeSwitchActive(bldc_cm->index,1) && bldc_cm->target > bldc_cm->position){
		bldc_cm->target = bldc_cm->position;
		bldc_cm->ctrl = BLDC_CTRL_STOP;
		ActivateDrivers(0);
		return;
	  }
  }

  //****Invert direction of rotation or hall****
  if(!(bldc_motors[0].status & BLDC_STATUS_MOVING) && (bldc_motors[0].state & BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST)){ //After motor A stoped moving
    bldc_motors[0].state &= ~BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST;
    if(bldc_motors[0].state & BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST_STATE)
      bldc_motors[0].state |= BLDC_MOTOR_STATE_INVERT_DIRECTION;
    else
      bldc_motors[0].state &= ~BLDC_MOTOR_STATE_INVERT_DIRECTION;
  }
  if(!(bldc_motors[0].status & BLDC_STATUS_MOVING) && (bldc_motors[0].state & BLDC_MOTOR_STATE_INVERT_HALL_REQUEST)){ //After motor A stoped moving
    bldc_motors[0].state &= ~BLDC_MOTOR_STATE_INVERT_HALL_REQUEST;
    if(bldc_motors[0].state & BLDC_MOTOR_STATE_INVERT_HALL_REQUEST_STATE)
      bldc_motors[0].state |= BLDC_MOTOR_STATE_INVERT_DIRECTION;   
    else
      bldc_motors[0].state &= ~BLDC_MOTOR_STATE_INVERT_DIRECTION; 
  }

  if(!(bldc_motors[1].status & BLDC_STATUS_MOVING) && (bldc_motors[1].state & BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST)){ //After motor B stoped moving
    bldc_motors[1].state &= ~BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST;
    if(bldc_motors[1].state & BLDC_MOTOR_STATE_INVERT_DIRECTION_REQUEST_STATE)
      bldc_motors[1].state |= BLDC_MOTOR_STATE_INVERT_DIRECTION;
    else
      bldc_motors[1].state &= ~BLDC_MOTOR_STATE_INVERT_DIRECTION;
  }

  if(!(bldc_motors[1].status & BLDC_STATUS_MOVING) && (bldc_motors[1].state & BLDC_MOTOR_STATE_INVERT_HALL_REQUEST)){ //After motor B stoped moving
    bldc_motors[1].state &= ~BLDC_MOTOR_STATE_INVERT_HALL_REQUEST;
    if(bldc_motors[1].state & BLDC_MOTOR_STATE_INVERT_HALL_REQUEST_STATE)
      bldc_motors[1].state |= BLDC_MOTOR_STATE_INVERT_DIRECTION;   
    else
      bldc_motors[1].state &= ~BLDC_MOTOR_STATE_INVERT_DIRECTION; 
  }
  //*************************************************************


  // Homing timeout
  if(bldc_cm->ctrl == BLDC_CTRL_HOMING && bldc_cm->homing_time >= (bldc_cfg.homing_timeout * 1000)) {
    bldc_cm->status |= BLDC_STATUS_ERR | BLDC_STATUS_HOME_TIMEOUT;
  }

  

  //***Process control function*****
  if(bldc_cm->ctrl & BLDC_CTRL_STOP) { //STOP REQUEST
    bldc_cm->target = bldc_cm->position;                    //force ramp down
  }else if(bldc_cm->ctrl == BLDC_CTRL_HOMING) {
    bldc_cm->target = bldc_cm->position - bldc_position_to_pulses(0, 10);      //set destination negative
    bldc_cm->homing_time++;
  }

  // calculate next step
  //motor_ramp();


  int err_p = bldc_cm->target - bldc_cm->position;

  if((err_p > 0) && !(any_motor_moving && bldc_cm->status&BLDC_STATUS_CCW)) {
    bldc_cm->status &= ~BLDC_STATUS_CCW;
    ActivateDrivers(1);
    //bldc_update_pwm(bldc_pwm);
  }else if((err_p < 0) && !(any_motor_moving && !(bldc_cm->status&BLDC_STATUS_CCW))) {
    bldc_cm->status |= BLDC_STATUS_CCW;
    ActivateDrivers(1);
    //bldc_update_pwm(-bldc_pwm);
  } else {  //deactivate drivers
    bldc_idletime++;
    if(bldc_idletime>=1000) {
      bldc_idletime = 1000;
      bldc_cm->ctrl = BLDC_CTRL_IDLE;  
      ActivateDrivers(0);
    }
  }

}


int dbg_state;
//commutation 
void bldc_Comutate(unsigned char motor){

        //debugigng output toggle
    //LPC_GPIO_PORT->B[1][24] ^= 1;

    bldc_motors[motor].status  &=  ~BLDC_STATUS_STALL;
/*
    int raw_timer = LPC_SCT1->COUNT;

    //take sample of comutation speed
    if(!LPC_SCT1->EVFLAG & 1<<0){ //read speed only if timer not overflowed
      bldc_Speed_raw += LPC_SCT1->COUNT;
      if(++speed_sample_counter >= 6 * number_of_poles){
        speed_sample_counter = 0;
        bldc_Speed =  (60ULL * SystemCoreClock)  / bldc_Speed_raw;	    
        bldc_Speed_raw = 0;
      }
    }

    LPC_SCT1->EVFLAG |= 1 << 0;   // reset overflow flag
    LPC_SCT1->CTRL |= (1 << 2); // halt speed measurment timer
    LPC_SCT1->CTRL |= (1 << 3); // clear count*/

    unsigned char state = bldc_ReadHall(motor); 

    if(state > 0 && state < 7)
      hall_detect++;
    else 
      hall_detect = 0;

    if(commutation_counter < 6)
         commutation_counter++;
    else commutation_counter = 0, hall_detect = 0; 

    if((hall_detect < 6) && (commutation_counter == 6) && !(bldc_motors[motor].state & BLDC_MOTOR_STATE_DC_OPERATION)){ // Detect missing hall pulses
      hall_fault++;
      if (hall_fault > 5){
        bldc_motors[bldc_motors[motor].index].status |= BLDC_STATUS_HALL_FAULT;
        bldc_EnableMotor(bldc_motors[motor].index,0);      
        hall_fault = 0;
        }
    }

    if(bldc_motors[motor].state & BLDC_MOTOR_STATE_INVERT_HALL){ //Swap direct. of encoder count) //inverter 
        if      (bldc_ccw_next[bldc_motors[motor].hall_state][0] == state)  
          bldc_motors[motor].position++;
        else  if(bldc_cw_next [bldc_motors[motor].hall_state][0] == state)  
          bldc_motors[motor].position--;
    }else{                                       //Normal
        if      (bldc_ccw_next[bldc_motors[motor].hall_state][0] == state)  
          bldc_motors[motor].position--;
        else  if(bldc_cw_next [bldc_motors[motor].hall_state][0] == state)  
          bldc_motors[motor].position++;
    }

    if(bldc_motors[motor].status & BLDC_STATUS_ACTIVE /*&& (!(bldc_motors[OTHER_MOTOR(motor)].status & BLDC_STATUS_MOVING)  ||  BLDC_MOTOR_COUNT == 1)*/){
        moving_counter[motor] = 100;
        if(!(bldc_motors[motor].state & BLDC_MOTOR_STATE_INVERT_DIRECTION) != !(bldc_motors[motor].state & BLDC_MOTOR_STATE_BRAKING) ){//Inverted operation (XOR braking)
        	//SEGGER_RTT_printf(0, "brake");
            if(bldc_motors[motor].status & BLDC_STATUS_CCW ){
              bldc_SetDrivers(bldc_cw_next [state][1], motor);
            }else{
              bldc_SetDrivers(bldc_ccw_next[state][1], motor); 
            }
        }else{                                       //NORMAL operation         
        	//SEGGER_RTT_printf(0, "a");
            if(bldc_motors[motor].status & BLDC_STATUS_CCW){
              bldc_SetDrivers(bldc_ccw_next[state][1], motor);
            }else {
              bldc_SetDrivers(bldc_cw_next [state][1], motor); 
            }
        }
    }else bldc_Speed = 0;

    bldc_motors[motor].hall_state = state;//save state

}


uint32_t speed_filter[4];
uint8_t speed_f_cnt[4];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	uint8_t filter_n_samples = 4;
	uint8_t motor = 0;

	uint16_t speed_t = __HAL_TIM_GET_COUNTER(&htim16);

	if (!(bldc_motors[0].state & BLDC_MOTOR_STATE_DC_OPERATION))
	{
		if(GPIO_Pin == HALL_A1_Pin || GPIO_Pin == HALL_A2_Pin || GPIO_Pin == HALL_A3_Pin ){ //BLDC Motor 0
			bldc_Comutate(0);
			motor = 0;
			filter_n_samples = 6;
		}
	}

#if DEVICE == KVARK
	if (!(bldc_motors[1].state & BLDC_MOTOR_STATE_DC_OPERATION))
	{
		if(GPIO_Pin == HALL_B1_Pin || GPIO_Pin == HALL_B2_Pin || GPIO_Pin == HALL_B3_Pin ){ //BLDC Motor 1
			bldc_Comutate(1);
			motor = 1;
			filter_n_samples = 6;
		}
	}
#endif
	speed_filter[motor] += speed_t - bldc_motors[motor].cnt_old;
	if (speed_t  < bldc_motors[motor].cnt_old ){ //Timer overflow
		speed_filter[motor] += htim16.Init.Period;
	}
	if(++speed_f_cnt[motor] >= filter_n_samples){
		bldc_motors[motor].speed = speed_filter[motor] / filter_n_samples;
		speed_f_cnt[motor] = 0;
		speed_filter[motor] = 0;
	}
	if(bldc_motors[motor].speed < 200 && bldc_motors[motor].state & BLDC_MOTOR_STATE_DC_OPERATION){	//first sample is junk
		bldc_motors[motor].speed = bldc_motors[motor].speed_freewheel;
	}
	bldc_motors[motor].cnt_old = speed_t;

}

void set_external_INT(uint8_t motor, uint8_t state){
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  if(state == BLDC_SET_INT){
		  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		  bldc_motors[motor].speed_freewheel = FREEWHEEL_SPEED_DEFAULT_BLDC; // Initialize default Freewheeling speed
	  }
	  else{
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  bldc_motors[motor].speed_freewheel = FREEWHEEL_SPEED_DEFAULT_DC;
	  }
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  if(motor == MOTOR_0){
		  GPIO_InitStruct.Pin = HALL_A1_Pin;
		  HAL_GPIO_DeInit(HALL_A1_GPIO_Port, HALL_A1_Pin); 		//Disables EINT genration
		  HAL_GPIO_Init(HALL_A1_GPIO_Port, &GPIO_InitStruct);

		  GPIO_InitStruct.Pin = HALL_A2_Pin;
		  HAL_GPIO_DeInit(HALL_A2_GPIO_Port, HALL_A2_Pin);
		  HAL_GPIO_Init(HALL_A2_GPIO_Port, &GPIO_InitStruct);
#if DEVICE == KVARK
		  GPIO_InitStruct.Pin = HALL_A3_Pin;
		  HAL_GPIO_DeInit(HALL_A3_GPIO_Port, HALL_A3_Pin);
		  HAL_GPIO_Init(HALL_A3_GPIO_Port, &GPIO_InitStruct);
	  }
	  else if(motor == MOTOR_1){
		  GPIO_InitStruct.Pin = HALL_B2_Pin;
		  HAL_GPIO_DeInit(HALL_B2_GPIO_Port, HALL_B2_Pin);
		  HAL_GPIO_Init(HALL_B2_GPIO_Port, &GPIO_InitStruct);

		  GPIO_InitStruct.Pin = HALL_B3_Pin;
		  HAL_GPIO_DeInit(HALL_B3_GPIO_Port, HALL_B3_Pin);
		  HAL_GPIO_Init(HALL_B3_GPIO_Port, &GPIO_InitStruct);
#endif
#if DEVICE != PICO
		  GPIO_InitStruct.Pin = HALL_B1_Pin;
		  HAL_GPIO_DeInit(HALL_B1_GPIO_Port, HALL_B1_Pin);
		  HAL_GPIO_Init(HALL_B1_GPIO_Port, &GPIO_InitStruct);
#endif
	  }
}

