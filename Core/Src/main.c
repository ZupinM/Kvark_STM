/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Shared_Libraries/bldc.h"
#include "Shared_Libraries/display_values.h"
#include "Shared_Libraries/suntracer.h"
#include "Shared_Libraries/write_values.h"
#include "rtc.h"
#include "Shared_Libraries/sun.h"
#include "Shared_Libraries/micro.h"
#include "Shared_Libraries/SX1278.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "Shared_Libraries/config.h"
#include "flash.h"
#include "Shared_Libraries/modbus.h"
#include "RTT/SEGGER_RTT.h"
#include "position.h"

//#define PRODUCTION_RELEASE

#define DAY_MS 	 86400000
#define HOUR_MS (DAY_MS/24)
#define MIN_MS	(HOUR_MS/60)


#define ENERGY_WINDOW_SIZE 16
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//astronimical calc
double declination;              //deklinacija
double hour_angle;               //urni kot
double AE_azimuth;               //azimut po AZI/ELE izracunih
double AE_elevation;             //elevacija po AZI/ELE izracunih
double PM_azimuth;               //azimut po POLAR-MOUNT izracunih
double PM_elevation;             //elevacija po POLAR-MOUNT izracunih
double HE_azimuth;               //azimut po HELIOSTAT enacbah
double HE_elevation;             //elevacija po HELIOSTAT enacbah
double HE_AE_azimuth;               //azimut po HELIOSTAT enacbah
double HE_AE_elevation;             //elevacija po HELIOSTAT enacbah
double angle_A;                  //kot prve (vcasih:horizontalne) osi za prikaz na PCju
double angle_B;                  //kot druge (vcasih:vertikalne) osi za prikaz na PCju
double show_angle_A;             //kot prve (vcasih:horizontalne) osi za prikaz na PCju
double show_angle_B;             //kot druge (vcasih:vertikalne) osi za prikaz na PCju
//u32 day_of_year;
double eot_offset;    //"equation of time" offset in minutes
double latitude_2;               //samo pozitivne vrednosti (zaradi Ju�ne poloble)

uint32_t goref_Nday_cnt_B;
uint32_t goref_Nday_cnt_A;
uint8_t upgrExe_gl;

signed int count_S=0;

/* parametri */
unsigned int events;
float err_currentA;
float err_positionA;
float err_voltageA;
float err_currentB;
float err_positionB;
float err_voltageB;

extern float bldc_pwm;

unsigned int tracker_status;			//status kondicije, v kateri je tracker (napake, halli...)
unsigned int tracker_status2;			//Status for motors 1 & 3 when motor_count > 3
unsigned int tracker_exstatus;
unsigned int tracker_exstatus2;
uint8_t slave_addr;			//slave address on RS485

unsigned int SN[4];	   			//vsebujejo serijske stevilke

unsigned int modbus_timeout;                    //po tem casu gre v rest_position [ure]
unsigned int modbus_cnt;					//stevec
unsigned int modbus_cnt1;         //stevec
unsigned int modbus_cnt2;         //stevec
unsigned int modbus_timeout_delay;              //unsigned int modbus_timout_delay * MODBUS_ID  [s]

unsigned int crc_errors;

float           LineResistance;
float 	        max_line_resistance;

extern int WindWaitCnt;
extern int SnowWaitCnt;
extern int WindCnt;

unsigned char ButtonStatus;

 extern unsigned char pcb_version1;                //TIV           27   27C1=0x1B 0x43 0x01 .... 0x1B=27 0x43='C' 0x01=1 .... vpisi 0x1B4301 oziroma 1786625
 extern char pcb_version2;                         //verzija TIVa  C
 extern unsigned char pcb_version3;                //TIV polaganje 1
 extern unsigned int FocusMiddleA,FocusMiddleB;

uint32_t systick_count = 0;

/* system */
unsigned int  button_timeout;
unsigned int  OverUnderVoltageFlagClearTimer;

//unsigned char move_direction_B;

//unsigned int time_out;					//na silo ustavljen motor preden stop_motor()
unsigned int backup_timeout;			//zakasnjen vpis v flash - backup
unsigned int green_led;					//utripanje LED
unsigned int tick_1s;					//stevec za generiranj 1s intervala
unsigned int counter10s;                                //stevec za generiranj 10s intervala
unsigned int modbus_indicator;			//stevec dolzine utripa ob rs485 sprejetju stringa
extern volatile int slaveCommandTimeout;

extern volatile int systick_ms;

float parameters [N_parameters];
float parameters_fixed [N_parameters_fixed];

/* flags registers */
unsigned int flags = 0;

unsigned char compensated = 0;

/* rs485 */
extern volatile uint8_t UARTTxEmpty0;
extern volatile uint8_t UARTTxEmpty1;

extern bldc_motor bldc_motors[BLDC_MOTOR_COUNT];
extern bldc_motor *bldc_cm;

unsigned int reset_status;
volatile unsigned int start_count = 0;
uint8_t init_main_state = 1;

extern volatile unsigned int number_of_poles;

unsigned int motor_operation;
unsigned int hall_enable = (0x03 | 0x03 << 8 | 0x03 << 16 | 0x03 << 24);

//fixed loaction in flash for version and checksum
__attribute__ ((section (".appver")))
//fsta
const Version swVersion={.sw_version=5003,
                         .blank1=0xffff,
                         .blank2=0xffff,
                         .blank3=0xffff,
                         .blank4=0xffff,
                         .blank5=0xffff,
                         .blank6=0xffff,
                         .blank7=0xffff,
                         .blank8=0xffff
                         };

uint8_t usbcnt = 0;
unsigned char status_homing = 0;
uint8_t usb_drive;
int motor_manual_timer;

time_t_ time;
time_t_ GMTtime;

time_t_ lastSyncTime;

uint32_t seconds24;
uint32_t last_seconds24;

unsigned int Focus_avg_timeout;


extern MODE_TYPE mode;
 MODE_TYPE mode_normal;
extern unsigned int no_run_time;

int micro_mode;

extern unsigned char phase_active;
extern int mosfet_protection_cnt;
unsigned int rxOffline_counter = 0;
unsigned int sigma_just_connected = 0;

#define AUTO_BIND_TIMEOUT 20000
#define AUTO_BIND_TIMEOUT2 1000

SEGGER_RTT_PRINTF_DESC BufferDesc;
char acBuffer[SEGGER_RTT_PRINTF_BUFFER_SIZE];
  unsigned NumDigits = 4;
  unsigned FormatFlags = 0;
  unsigned FieldWidth = 0;

uint8_t transceiver = NONE;
uint8_t transceiver_saved;
uint8_t tx_settings_flag;
uint8_t tx_setting_route;
uint8_t set_settings_flag;
uint8_t tx_buffered_flag;
uint32_t bindCheck_timeout = AUTO_BIND_TIMEOUT2;
uint8_t timeout_counter;
uint8_t bindMode_byChannel = 0;
uint8_t route_settings_delay = 0;

uint8_t tx_settings_flag;
uint8_t tx_setting_route;
uint8_t set_settings_flag;
uint8_t tx_buffered_flag;
uint32_t timeout_master_check = AUTO_BIND_TIMEOUT;
uint8_t tx_packet_buffer[BUFSIZE];
uint32_t tx_packet_length;
uint8_t SPI_RxFifo_cmplt;
uint8_t SPI_TxFifo_cmplt;

uint16_t online_timeouts[165];
uint8_t available_positioners[20];
int baudrate = 19200;
int baudrate_timeout = 1;

extern volatile uint8_t UARTBuffer0[BUFSIZE];
volatile uint8_t rs485_forward_enabled;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void ButtonProcess ();
void StatusUpdate();
void Measure_Line_Resistance();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char * strUpper(char *str){
char *tmp=str;
while (*str!=0){ *str= toupper(*str);str++;}
return tmp;
}

char * strLower(char *str){
char *tmp=str;
while (*str!=0){ *str= tolower(*str);str++;}
return tmp;
}

int GetMode(){
 return micro_mode;
}


void SetMode(int mode){
  if(mode != micro_mode){
   micro_mode = mode;
   if(mode==MICRO_MODE_SLAVE){
     AxisSetStates((1<<18)|(1<<2));
     //disable some features
     bflags &=~(1<<time_enable);
     cflags &=CFLAGS_SLAVE_MODE_MASK;
     bldc_Stop(1);
     store_in_flash=100;
     //BKP_WriteBackupRegister(BKP_DR5,bflags); //write in RTC RAM
   }else{
     AxisSetStates((0<<18)|(1<<2));
     bldc_Stop(1);
     store_in_flash=100;
   }
  }
}

//axis 3 used as micro mode
void AxisSetStates(unsigned int state){
  unsigned int *axis_enable=(unsigned int *)&modbus_m_enabled;
  unsigned int oldstate=*(&modbus_m_enabled);
  unsigned int newstate=state>>16;

  int i;
  for(i=0;i<16;i++){
    if(state&(1<<i)){//axis
      if(newstate&(1<<i))*axis_enable |= (1<<i);
      else             *axis_enable &=~(1<<i);
    }
  }
  if( oldstate != *axis_enable ){
	  if(!backup_timeout) store_in_flash=100;
  }
}

int AxisEnabled(unsigned int index){
  unsigned int *axis_enable=(unsigned int *)&modbus_m_enabled;
  if(index>=3) return 0;
  if(*axis_enable&(1<<index)) return 1;
  return 0;
}

volatile int systick_ms;
void SysTick_Handler(void)
{
	  systick_count++;
	  WindCnt++;
	  systick_ms=1;

	  if(sigma_just_connected < 100000)
	    sigma_just_connected++;

	  if(phase_active)
	    mosfet_protection_cnt++;
	  if(mosfet_protection_cnt > 500){
	    bldc_Stop(1);
	    bldc_cm->status |= BLDC_STATUS_TO_DST_ERR;
	  }
	  slaveCommandTimeout++;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

#ifdef PRODUCTION_RELEASE 					//Code Read Protection
  FLASH_OBProgramInitTypeDef CRP_settings;
  HAL_FLASH_Unlock();
  HAL_FLASH_OB_Unlock();
  HAL_FLASHEx_OBGetConfig(&CRP_settings);
  if(CRP_settings.RDPLevel == OB_RDP_LEVEL_0){
	  CRP_settings.RDPLevel = OB_RDP_LEVEL_1;
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	  HAL_FLASHEx_OBProgram(&CRP_settings);
	  HAL_FLASH_OB_Launch();
  }
#endif

  int LoadDefaults = 0;
  int startup = 50;

  //HAL_Delay(500);

  //sys_data_write();
  //Watchdog_Init();
  read_SN();

  micro_mode = MICRO_MODE_STANDALONE;
  micro_mode = AxisEnabled(2);


  //****Load settings from FLASH******
  flash_read(FLASH_ADDR_MAIN);
  StartADC_DMA_Sequence();
  if(LoRa_id < 100 || LoRa_id > 200){
    LoRa_id = 116;
    LoadDefaults = 1;
  }
  if(slave_addr == 0 || slave_addr > 100){
    slave_addr = 16;
    LoadDefaults = 1;
  }

  //initialize flags/Variables
  if(reset_status != RESET_MANUAL)
    tracker_status |=  SF_POWER_FAILURE;   //set power failure flag indicating CPU reset

  reset_status = RESET_UNKNOWN;

  tracker_status &=  ~(ERR_HALL_B | ERR_TOOLONG_REF_B | ERR_CABLE_B | (0x7f << 11));
  tracker_exstatus &=  ~(EFS_UNDERVOLTAGE | EFS_OVERVOLTAGE | EFS_LINE_RES_MEASURING); //Clear flag

  if (modbus_timeout > 4320000)
    modbus_timeout = 1800;
  if (modbus_timeout_delay > 4320000)
    modbus_timeout_delay = 0;

  //***Initialize CORE BLDC motor functions****
  bldc_init(LoadDefaults);

  //button stuck detection
  if (ButtonStates()){
    tracker_exstatus |= EFS_BUTTON_STUCK;
  }
  //wait until valid voltage
  int USB_power_delay = 0;
  while(GetAnalogValues(SUPPLY) < SHUTDOWN_VOLTAGE) {
    //LPC_WWDT->FEED = 0xAA;
    //LPC_WWDT->FEED = 0x55;
    if(USB_power_delay ++ > 100000){
      break;
    }
  }

  //        channel,  tx power, SF=spread factor, bandwidth,   msg length(only needed when SF=6)  ,  tx/rx
  if(module.spFactor > 6 && module.spFactor < 11)
    transceiver = LoRa_config(module.channel, module.power, module.spFactor, module.LoRa_BW, LoRa_MAX_PACKET, RxMode);
  else
    transceiver = LoRa_config(42, LoRa_POWER_20DBM, LoRa_SF_7, LoRa_BW_250KHZ, LoRa_MAX_PACKET, RxMode); //default settings
  if(transceiver == LORA){
    //LPC_GPIO_PORT->DIR[0] |= (1<<5) | (1<<6); //LED output
  }
  else{
    //NVIC_DisableIRQ(PIN_INT0_IRQn);
    transceiver = NONE;
    uartMode = UART_MODE_RS485;
    LoRa_id = 116;  // default
  }

#if (DEVICE != PICO)
  //focus_init();
  HallVoltage();
  init_weather_sensor();
#endif

  if(tracker_status&SF_TRACKING_ENABLED)
    mode = MODE_MICRO;

  //*****control loop*******

  //unsigned int countRsRx = 0;
  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  flags &= ~(1 << reset_it);  // do not reset for begin


  //setGPIO_Function(MOTOR_A_1H_GPIO_Port, MOTOR_A_1H_Pin, MODE_ALTERNATE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  init_main_state = 0;
  while (1)
  {
	  if(bldc_Status() & BLDC_SHUTTING_DOWN) {
	       bldc_ReleaseDrivers();
	     }

	     if (systick_ms != 1)
	       continue;
	     systick_ms=0;       // execute every 1ms

	     //SEGGER_RTT_printf(0, "main period:%d\r\n", systick_count -systick_count_prev);
	     //systick_count_prev = systick_count;

	     // reset before upgrade
	     if(delay_reset > 0) {
	       delay_reset--;
	       if(delay_reset <= 0) {
	         flags |= (1 << reset_it);
	       }
	     }

	     // reset command
	     if ((flags & (1 << reset_it)) == (1 << reset_it)) {
	       flash_write(FLASH_ADDR_MAIN);
	       while(1);
	     }

	     //save flash on power-shutdown
	     if(GetAnalogValues(SUPPLY_UNFILTERED) < SHUTDOWN_VOLTAGE && USB_power_delay < 100000) { //No check when powered over USB
	       flash_write(FLASH_ADDR_MAIN);
	       while(1);
	     }

	     set_LED(0,COUNTDOWN,0); //LED timeouts processing

	     if(++usbcnt >= 20){ //Zapis v USB na 20ms
	       USB_display();
	       if(usb_drive)
	         if (!(bldc_getStatus(0) & BLDC_STATUS_MANUAL || bldc_getStatus(1) & BLDC_STATUS_MANUAL))
	           if(motor_manual_timer++ >= 13) {
	             if(!status_homing) bldc_manual(1);
	             motor_manual_timer = 0;
	           }
	       USB_write();
	       usbcnt = 0;
	     }


	   //store parameters in flash - check once per second
	     if(!bldc_Active(0) && !bldc_Active(1)){
	       if (store_in_flash==1) flash_write(FLASH_ADDR_MAIN);
	       //if (store_in_flash==101) upgrExe_gl = 1;
	       if (store_in_flash!=0) store_in_flash--;
	       else store_in_flash = 1000;

	     }

	     getSolarDateTime(&time);
	     getGMTDateTime(&GMTtime);

/*	     //RTC correction on midnight
	     if(time.hours == 0 && time.minutes == 0 && time.seconds == 0 && !compensated)
	       rtc_apply_correction(), compensated = 1;
	     if(time.hours == 1)   compensated = 0;
*/
	     sun_Update();

	     if(startup){
	       startup--;
	       if(startup==0)
	         Recalc_sun();
	     }

	     if(mode==MODE_MICRO || mode==MODE_OK)
	       check_time();

	     weather_sensor();

	     focus_process();

	     //if(!(bldc_cm->state & BLDC_MOTOR_STATE_DC_OPERATION))
	       bldc_process();
	     //else
	       //dc_process(); // brush mode motor

	     led_handling();

	     ButtonProcess();
	     HallVoltage();

	     bindByChannel_check();
	     onlineDevices_check();
	     auto_BaudRate();
	     bindMode_check();

	     //bldc_I(0);

	     // MODBUS RS485
	     if(modbus_newRequest() &&  ((
	    	huart485->gState != HAL_UART_STATE_BUSY && huart485->RxState != HAL_UART_STATE_BUSY_RX && //UART is IDLE
	       (uartMode == UART_MODE_RS485 || uartMode == UART_MODE_XBEE))))
	     {
	       if((UARTBuffer0[0] == LoRa_id || UARTBuffer0[0] == slave_addr || UARTBuffer0[0] == 0 || ((UARTBuffer0[0] == slave_addr+1) && (motor_count > 2)))
	           && uartMode == UART_MODE_RS485 ){
	         modbus_cmd();              // 485 received from Sigma, Answer KVARK data, 485->Sigma
	         modbus_ReqProcessed();      // re-enable reception
	         rxOffline_counter = 10000;
	       }
	       else if (UARTBuffer0[0] != slave_addr && uartMode == UART_MODE_RS485 && transceiver == XBEE) {
	         modbus_cmd3();
	         modbus_ReqProcessed();
	       }
	       else if (transceiver == LORA){
	         modbus_cmd2();              // 485 received from Nano, forward through LoRa to SIgma
	         modbus_ReqProcessed();      // re-enable reception
	       }
	       else
	         modbus_ReqProcessed();      // re-enable reception
	     }
	     else if(modbus_newRequest() &&
	        uartMode == UART_MODE_XBEE )
			//&& UART_CheckIdleState(&huart1) )
	     {
	       modbus_cmd ();
	       modbus_ReqProcessed();

	       if(modbus_discard()) {
	         UART1ClearStatus();
	         modbus_ReqProcessed();
	       }
	     }

	     if (transceiver == LORA) {

	 		if(SPI_TxFifo_cmplt == 1){
		 		 uint8_t tmp = 0x8b;
		 		 LoRa_SPIWrite(LR_RegOpMode, &tmp, 1); //Tx Mode
		 		 SPI_TxFifo_cmplt = 0;
	 		}
			if(SPI_RxFifo_cmplt == 1){
				lora_int_stat = 0;
				checkRouting = 1;
				uint8_t tmp = 0x8D;
				LoRa_SPIWrite(LR_RegOpMode, &tmp, 1); //Rx Mode
				rxOffline_counter = 10000;
				SPI_RxFifo_cmplt = 0;
			}
			get_LoRa_Status_DIO();
	       if(lora_int_stat == TRANSMISSION_FINISHED)
	         tx_finished();
	       if(lora_int_stat == PACKET_RECEIVED)
	         rx_finished();

	       if(checkRouting && !rs485_forward_enabled)
	         check_routing();
	       if(module.packetReady && (module.rxBuffer[0] == LoRa_id || module.rxBuffer[0] == slave_addr || module.rxBuffer[0] == 0 || ((module.rxBuffer[0] == slave_addr+1) && (motor_count > 2)))){
	         modbus_cmd ();         //LoRa receive, KVARK response
	         modbus_ReqProcessed(); //re-enable reception
	       }
	       else if(module.packetReady){
	         //modbus_cmd1();              // LoRa received, forward through 485->nano
	         timeout_master_check = AUTO_BIND_TIMEOUT;
	         modbus_ReqProcessed1();      // re-enable reception
	       }
	       module.packetReady = 0;

	       if(transmission == 0){

	         if(tx_buffered_flag){
	           LoRa_TxPacket((uint8_t *)tx_packet_buffer, tx_packet_length, 8000);
	           tx_buffered_flag = 0;
	           if(set_settings_flag)   //exit sending settings, -> set settings
	             tx_settings_flag = 0;
	         }

	       }
	     }

	     if (rxOffline_counter) {
	       rxOffline_counter --;

	     } else {  ////////////////////Sigma disconected, timeout  -> set mode micro tracking or manual
			   sigma_just_connected = 0;
			   if (mode == MODE_SLAVE){
				 mode = MODE_OK;
				 bldc_EnableMotor(0,1);
				 bldc_EnableMotor(1,1); //Enable disabled states in sigma
			   }
			   else if(mode == MODE_SLAVE_TRACKING){
				 mode = MODE_MICRO;
				 bflags|=(1<<time_enable);
				 no_run_time=0;
				 Recalc_sun();
			   }
	      }

	       if (mode != MODE_WIND && mode != MODE_SNOW)
	         mode_normal = mode;
	       if (flags&(1<<WindModeOn))
	          mode=MODE_WIND;  //Wind ima prednost pred Snow
	       else if (flags&(1<<SnowModeOn))
	          mode=MODE_SNOW;
	        else mode = mode_normal;

	       modbus_ReqProcessed();
	       reEnable_485_DMA_RX();

	     //Measure_Line_Resistance();

	     StatusUpdate();
	     // auto clear status flags
	     AutoClearFlag();

	     // WDT reset
	      if(!phase_active){

	     }

	     // counter for delay looking any status, to 15min
	     if(start_count < CLEAR_TIME)
	       start_count++;
	     if(start_count == CLEAR_TIME) {         // clear status
	       ClearStatus(0);
	       ClearStatus(2);
	       start_count = CLEAR_TIME + 1;         // stop counter
	     }

	     if(counter10s>10000){
	       counter10s = 0;
	       GetAnalogValues(BATTERY);
	     }
	     counter10s++;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void bindMode_check(){

  if (!LoRa_bindMode_slave && ButtonStatus & (1<<2)){
    LoRa_Bind_Mode(BIND_SLAVE);
    ButtonStatus = 0;
  }
  if(LoRa_bindMode_slave){
    if(LoRa_channel_received || ButtonStatus & (1<<2)){
      //LoRa_channel_received = 0;
      LoRa_bindMode_master = 0;
      LoRa_Bind_Mode(BIND_EXIT);
      ButtonStatus = 0;
    }
  }
}

void auto_BaudRate(){

  //int baudrate_delay = 10000;

  if(transceiver != NONE) {
    if(baudrate == 115200){
    	UART_ChangeBaudRate(19200);
    }
    return;
  }

  if(baudrate_timeout)                  //10s retry on both baudrates
    if(baudrate_timeout++ > 10000){
      if(baudrate == 115200){
        baudrate = 19200;
      }
      else{
        baudrate = 115200;
      }
      //UART_ChangeBaudRate(baudrate);
      baudrate_timeout = 1;
    }

}

void onlineDevices_check(){

  if(timeout_counter > 100){
    for(int i=1 ; i<164 ; i++){
      if(online_timeouts[i] > 0){
        online_timeouts[i] --;
      }else{
          int n = (i-1) / 8;
          available_positioners[n] &= ~(1 << (i - n*8 - 1));
      }
    }
    timeout_counter = 0;
    if(transmission && !LoRa_bindMode_master && !bindMode_byChannel)
       transmission ++;
    if(transmission > 5){                 // Transmission hang escape
       transmission = 0;
       //SEGGER_RTT_printf(0, "transmission hang");
       LoRa_EntryRx(LoRa_MAX_PACKET, 2000);
    }
  }
  timeout_counter++;

}

void bindByChannel_check(){

  if(transceiver != LORA)
    return;

  if(timeout_master_check){
    if(!LoRa_bindMode_slave)
      timeout_master_check--;
  }else{
    if(!bindMode_byChannel){
      LoRa_Bind_Mode(BIND_BY_CHANNEL);
      //debug_printf("bind by channel \n");
      checkRouting = 0;
      bindMode_byChannel = 1;
    }
    if(bindCheck_timeout){
      bindCheck_timeout --;
    }else{                      //no channel received,
      LoRa_bindMode_slave = 0;
      LoRa_bindMode_master = 1; //To let slave set orriginal settings
      LoRa_Bind_Mode(BIND_EXIT);
      //debug_printf("bind disable \n");
      bindMode_byChannel = 0;
      bindCheck_timeout = AUTO_BIND_TIMEOUT2;
      timeout_master_check = AUTO_BIND_TIMEOUT;
    }
  }

}


void AutoClearFlag()
{
  unsigned int stat = bldc_Status();

  if ((stat & BLDC_OVERVOLTAGE) || (stat & BLDC_UNDERVOLTAGE))
    OverUnderVoltageFlagClearTimer = 0;
  if (OverUnderVoltageFlagClearTimer++ == (MIN_MS * 5))
    tracker_exstatus &= ~(EFS_OVERVOLTAGE | EFS_UNDERVOLTAGE);
}

/***********************************************************
  LED HANDLING & 1 second TICK
************************************************************/
void led_handling() {
  unsigned int stat = bldc_Status();

  if ((tracker_status&SF_NO_MODBUS) || (modbus_indicator > 0)) {    // oranzna sveti, ce MODbus komunikacije ni - najbolj prednostna indikacija
    set_LED(RED, LED_ON, 0);
    set_LED(GREEN, LED_ON, 0);
    tick_1s = 1000;     // zaradi lepsega prehoda iz oranzne nazaj na zeleno (tick_1s)
  } else {
    set_LED(RED, LED_OFF, 0);
    if (green_led == 0)
      set_LED(GREEN, LED_OFF, 0);

    if (tracker_status & 0x0FF || stat& BLDC_OVERVOLTAGE || stat&BLDC_UNDERVOLTAGE || (tracker_exstatus &(EFS_ERROR_STATESA | EFS_ERROR_STATESB | EFS_ERROR_STATES))) {
      set_LED(RED, 1, 0);
    }
  }

  if (modbus_indicator)
    modbus_indicator--;

   //1s TIME TICK
  if (green_led) {      // zeleno utripanje = delovanje ok
    set_LED(GREEN, 1, 0);
    green_led--;
  } else
    set_LED(GREEN, 0, 0);

  // zeleno normalno utripanje
  if (tick_1s == 0) {
    tick_1s = 1000;	//reload 1000 * 1ms = 1s   ... izbrana A os
    green_led = 30;
  }
  else
    tick_1s--;
}

/***********************************************************
  TRACKER STATUS
************************************************************/
unsigned int tracker_exstatus_prev = 0;
void StatusUpdate() {

  /* indikacija izpada napajanja ( flag se postavi po resetu izven main() ) - ostane do "clear" ukaza */
  int bldcs = bldc_Status();
  bldc_motor *ma = bldc_Motor(0);
  bldc_motor *mb = bldc_Motor(1);

  unsigned int *tracker_status_p = &tracker_status;
  unsigned int *tracker_exstatus_p = &tracker_exstatus;

  uint8_t motor_count_l = 2;
  if(motor_operation & 0x0002){ // MA DC motor operation
	bldc_motors[0].state |= BLDC_MOTOR_STATE_DC_OPERATION;
	bldc_motors[2].state |= BLDC_MOTOR_STATE_DC_OPERATION;
	motor_count_l++;
  }else{
	  bldc_motors[0].state &= ~BLDC_MOTOR_STATE_DC_OPERATION;
	  bldc_motors[2].state &= ~BLDC_MOTOR_STATE_DC_OPERATION;
  }
  if(motor_operation & 0x10000){ // MB DC motor operation
	  bldc_motors[1].state |= BLDC_MOTOR_STATE_DC_OPERATION;
	  bldc_motors[3].state |= BLDC_MOTOR_STATE_DC_OPERATION;
	motor_count_l++;
  }else{
	  bldc_motors[1].state &= ~BLDC_MOTOR_STATE_DC_OPERATION;
	  bldc_motors[3].state &= ~BLDC_MOTOR_STATE_DC_OPERATION;
  }
  motor_count = motor_count_l;

  if(motor_count > 2){
	  mb = bldc_Motor(2);
  }
  for (int m = 0 ; m < motor_count ; m += 2){
	  if(m >= 2){
		  tracker_status_p = &tracker_status2;
		  tracker_exstatus_p = &tracker_exstatus2;
		  ma = bldc_Motor(1);
		  mb = bldc_Motor(3);
	  }
	  *tracker_status_p &= ~(0xff| SF_MOVING_OUT_A | SF_MOVING_IN_A | SF_MOVING_REF_CLR_A | SF_ENDSW_A_LO_PRESSED | SF_ENDSW_A_HI_PRESSED | SF_MOVING_OUT_B | SF_MOVING_IN_B | SF_MOVING_REF_CLR_B | SF_ENDSW_B_LO_PRESSED | SF_ENDSW_B_HI_PRESSED);

	  *tracker_exstatus_p &= ~(EFS_VOLTAGE_TO_LOW | EFS_MOTOR_CUTOFF);

	  *tracker_status_p |= (ma->status >> 7) & 0xf;    //error flags
	  if (ma->status & BLDC_STATUS_ENDSWITCH_LOW_ACTIVE)
		*tracker_status_p |= SF_ENDSW_A_LO_PRESSED;
	  if (ma->status & BLDC_STATUS_ENDSWITCH_HIGH_ACTIVE)
		*tracker_status_p |= SF_ENDSW_A_HI_PRESSED;

	  if (ma->status & BLDC_STATUS_ERR_MOVEOUT)
		*tracker_exstatus_p |= ESF_MOVE_OUT_ERR_A;
	  else
		*tracker_exstatus_p &=~ ESF_MOVE_OUT_ERR_A;

	  if(GetAnalogValues(BATTERY) < 2.4)
		*tracker_exstatus_p |= EFS_BATTERY_LOW;
	  else
		*tracker_exstatus_p &= ~EFS_BATTERY_LOW;

#if DEVICE != PICO
	  *tracker_status_p |= (mb->status >> 3) & 0xf0; //error flags
	  //if(mb->status & BLDC_STATUS_HALL_FAULT)
	   // tracker_status |= ERR_HALL_B;
	  if(mb->status& BLDC_STATUS_ENDSWITCH_LOW_ACTIVE)
		*tracker_status_p |=SF_ENDSW_B_LO_PRESSED;
	  if(mb->status& BLDC_STATUS_ENDSWITCH_HIGH_ACTIVE)
		*tracker_status_p |=SF_ENDSW_B_HI_PRESSED;

	  if(mb->status& BLDC_STATUS_ERR_MOVEOUT)
		*tracker_exstatus_p |= ESF_MOVE_OUT_ERR_B;
	  else
		*tracker_exstatus_p &=~ ESF_MOVE_OUT_ERR_B;
#endif
	  // INDIKACIJE
	  // extended status
	  if (ma->status & BLDC_STATUS_ENDSWITCH_ERROR)
		*tracker_exstatus_p |= EFS_END_SWA_FAIL;
	  if (bldcs & BLDC_OVERVOLTAGE)
		*tracker_exstatus_p |= EFS_OVERVOLTAGE;
	  if (bldcs & BLDC_UNDERVOLTAGE)
		*tracker_exstatus_p |= EFS_UNDERVOLTAGE;
	  if (bldcs & BLDC_MOTOR_CUTOFF)
		*tracker_exstatus_p |= EFS_MOTOR_CUTOFF;
	  if (bldcs & BLDC_VOLTAGE_TO_LOW)
		*tracker_exstatus_p |= EFS_VOLTAGE_TO_LOW;
	  if (flags & buttonstuck)
		*tracker_exstatus_p |= EFS_BUTTON_STUCK;
	  else
		*tracker_exstatus_p &= ~EFS_BUTTON_STUCK;

	  if (bldcs & BLDC_LOCKED)
		*tracker_exstatus_p |= EFS_LOCKED;
	  else
		*tracker_exstatus_p &=~ EFS_LOCKED;

	  if ((flags & Modbus_timeout) && (!(tracker_status & SF_NO_MODBUS))) {
		*tracker_status_p |= SF_NO_MODBUS;
	  }

	  /* napaka MOTORJA A */
	  if ((ma->status & BLDC_STATUS_OVERCURRENT)&&(!(*tracker_status_p & ERR_OVERCURRENT_MOTOR_A))) {
		  *tracker_status_p |= ERR_OVERCURRENT_MOTOR_A;
	    backup_timeout=BACKUP_TO;
	  }
	  if ((!(ma->status & BLDC_STATUS_OVERCURRENT))&&(*tracker_status_p & ERR_OVERCURRENT_MOTOR_A)) {
		  *tracker_status_p &=~ ERR_OVERCURRENT_MOTOR_A;
	    backup_timeout=BACKUP_TO;
	  }

	  /* napaka MOTORJA B */
	  if ((mb->status & BLDC_STATUS_OVERCURRENT)&&(!(*tracker_status_p & ERR_OVERCURRENT_MOTOR_B))) {
		  *tracker_status_p |= ERR_OVERCURRENT_MOTOR_B;
	    backup_timeout=BACKUP_TO;
	  }
	  if ((!(mb->status & BLDC_STATUS_OVERCURRENT))&&(*tracker_status_p & ERR_OVERCURRENT_MOTOR_B)) {
		  *tracker_status_p &= ~ERR_OVERCURRENT_MOTOR_B;
	    backup_timeout=BACKUP_TO;
	  }
	  /* napaka HALL-a A - sproti se osvezuje */
	  if ((ma->status& BLDC_STATUS_HALL_FAULT)&&(!(*tracker_status_p &ERR_HALL_A))) {
		  *tracker_status_p |=ERR_HALL_A;
	    backup_timeout=BACKUP_TO;
	  }
	  if ((!(ma->status& BLDC_STATUS_HALL_FAULT))&&(*tracker_status_p &ERR_HALL_A)) {
		  *tracker_status_p &=~ERR_HALL_A;
	    backup_timeout=BACKUP_TO;
	  }

	  /* HALL-A Loosing impulses - sproti se osvezuje */
	  if ((ma->status& BLDC_STATUS_HALL_LOST_IMPULSES)&&(!(*tracker_status_p &SF_HALL_WIRING_A))) {
		  *tracker_status_p |=SF_HALL_WIRING_A;
	    backup_timeout=BACKUP_TO;
	  }
	  if ((!(ma->status& BLDC_STATUS_HALL_LOST_IMPULSES))&&(*tracker_status_p &SF_HALL_WIRING_A)) {
		  *tracker_status_p &=~SF_HALL_WIRING_A;
	    backup_timeout=BACKUP_TO;
	  }

	  /* napaka HALL-a B - sproti se osvezuje */
	  if ((mb->status& BLDC_STATUS_HALL_FAULT)&&(!(*tracker_status_p&ERR_HALL_B))) {
		  *tracker_status_p |=ERR_HALL_B;
	    backup_timeout=BACKUP_TO;
	  }
	  if ((!(mb->status& BLDC_STATUS_HALL_FAULT))&&(*tracker_status_p &ERR_HALL_B)) {
		  *tracker_status_p &=~ERR_HALL_B;
	    backup_timeout=BACKUP_TO;
	  }

	  /* HALL-B Loosing impulses - sproti se osvezuje */
	  if ((mb->status& BLDC_STATUS_HALL_LOST_IMPULSES)&&(!(*tracker_status_p &SF_HALL_WIRING_B))) {
		  *tracker_status_p |=SF_HALL_WIRING_B;
	    backup_timeout=BACKUP_TO;
	  }
	  if ((!(mb->status& BLDC_STATUS_HALL_LOST_IMPULSES))&&(*tracker_status_p &SF_HALL_WIRING_B)) {
		  *tracker_status_p &=~SF_HALL_WIRING_B;
	    backup_timeout=BACKUP_TO;
	  }

	  /* napaka KABLA A - sproti se osvezuje */
	  if ((ma->status & BLDC_STATUS_CABLEERROR)&&(!(*tracker_status_p &ERR_CABLE_A))) {
		  *tracker_status_p |=ERR_CABLE_A;
	    backup_timeout=BACKUP_TO;
	  }
	  if ((!(ma->status & BLDC_STATUS_CABLEERROR))&&(*tracker_status_p &ERR_CABLE_A)) {
		  *tracker_status_p &=~ERR_CABLE_A;
	    backup_timeout=BACKUP_TO;
	  }
	  /* napaka KABLA B - sproti se osvezuje */
	  if ((mb->status & BLDC_STATUS_CABLEERROR)&&(!(*tracker_status_p &ERR_CABLE_B))) {
		  *tracker_status_p |=ERR_CABLE_B;
	    backup_timeout=BACKUP_TO;
	  }
	  if ((!(mb->status & BLDC_STATUS_CABLEERROR))&&(*tracker_status_p &ERR_CABLE_B)) {
		  *tracker_status_p &=~ERR_CABLE_B;
	    backup_timeout=BACKUP_TO;
	  }

	  if ((ma->status & BLDC_STATUS_TO_DST_ERR) && (!(*tracker_status_p & (1 << ERR_TO_DST_A)))) {
		  *tracker_status_p |= (1 << ERR_TO_DST_A);
	    backup_timeout=BACKUP_TO;
	  }
	  if ((!(ma->status & BLDC_STATUS_TO_DST_ERR)) && (*tracker_status_p & (1 << ERR_TO_DST_A))) {
		  *tracker_status_p &= ~(1 << ERR_TO_DST_A);
	    backup_timeout=BACKUP_TO;
	  }
	  if ((mb->status & BLDC_STATUS_TO_DST_ERR) && (!(*tracker_status_p & (1 << ERR_TO_DST_B)))) {
		  *tracker_status_p |= (1 << ERR_TO_DST_B);
	    backup_timeout=BACKUP_TO;
	  }
	  if ((!(mb->status & BLDC_STATUS_TO_DST_ERR)) && (*tracker_status_p & (1 << ERR_TO_DST_B))) {
		  *tracker_status_p &= ~(1 << ERR_TO_DST_B);
	    backup_timeout=BACKUP_TO;
	  }

	  // napaka referenc too long A
	  if ((ma->status & BLDC_STATUS_HOME_TIMEOUT)&&(!(*tracker_status_p &ERR_TOOLONG_REF_A))) {
		  *tracker_status_p |=ERR_TOOLONG_REF_A;
	    backup_timeout=BACKUP_TO;
	  }
	  if ((!(ma->status & BLDC_STATUS_HOME_TIMEOUT))&&(*tracker_status_p&ERR_TOOLONG_REF_A)) {
		  *tracker_status_p &=~ERR_TOOLONG_REF_A;
	      backup_timeout=BACKUP_TO;
	  }

	  // napaka referenc too long B
	  if ((mb->status & BLDC_STATUS_HOME_TIMEOUT)&&(!(*tracker_status_p &ERR_TOOLONG_REF_B))) {
		  *tracker_status_p |=ERR_TOOLONG_REF_B;
	    backup_timeout=BACKUP_TO;
	  }
	  if ((!(mb->status & BLDC_STATUS_HOME_TIMEOUT))&&(*tracker_status_p &ERR_TOOLONG_REF_B)) {
		  *tracker_status_p &=~ERR_TOOLONG_REF_B;
	    backup_timeout=BACKUP_TO;
	  }


	//  INDIKACIJE

	  /* indikacija pritiska TIPKE v statusu */
	  if (ButtonStatus != 0)
		  *tracker_status_p |=SF_BUTTON_PRESSED;
	        else tracker_status&=~SF_BUTTON_PRESSED;

	   /*indikacija VRTENJA motorja A - se osvezuje sproti */

	  if(ma->status & BLDC_STATUS_MOVING_IN) {
		  *tracker_status_p |= SF_MOVING_IN_A;
		  *tracker_status_p &= ~SF_MOVING_IN_B;
		  *tracker_status_p &= ~SF_MOVING_OUT_B;
	  }
	  else {
	    tracker_status &= ~SF_MOVING_IN_A;
	  }
	  if(ma->status & BLDC_STATUS_MOVING_OUT) {
		  *tracker_status_p |= SF_MOVING_OUT_A;
		  *tracker_status_p &= ~SF_MOVING_IN_B;
		  *tracker_status_p &= ~SF_MOVING_OUT_B;
	  }
	  else {
		  *tracker_status_p &= ~SF_MOVING_OUT_A;
	  }

	  /* indikacija VRTENJA motorja B - se osvezuje sproti */
	  if(mb->status & BLDC_STATUS_MOVING_IN) {
		  *tracker_status_p |= SF_MOVING_IN_B;
		  *tracker_status_p &= ~SF_MOVING_IN_A;
		  *tracker_status_p &= ~SF_MOVING_OUT_A;
	  }
	  else {
		  *tracker_status_p &= ~SF_MOVING_IN_B;
	  }

	  if(mb->status & BLDC_STATUS_MOVING_OUT) {
		  *tracker_status_p |= SF_MOVING_OUT_B;
		  *tracker_status_p &= ~SF_MOVING_IN_A;
		  *tracker_status_p &= ~SF_MOVING_OUT_A;
	  }
	  else {
		  *tracker_status_p &= ~SF_MOVING_OUT_B;
	  }

	  /* indikacija REFERENCE A - se osvezuje sproti */
	  if (ma->status & BLDC_STATUS_HOMING)
		  *tracker_status_p |=SF_MOVING_REF_CLR_A;
	  else *tracker_status_p &=~SF_MOVING_REF_CLR_A;

	  /* indikacija REFERENCE B - se osvezuje sproti */
	  if  (mb->status & BLDC_STATUS_HOMING)
		  *tracker_status_p |=SF_MOVING_REF_CLR_B;
	  else *tracker_status_p &=~SF_MOVING_REF_CLR_B;
  }

  /*if(tracker_exstatus_prev != tracker_exstatus && (tracker_exstatus & EFS_UNDERVOLTAGE) == EFS_UNDERVOLTAGE)
    flash_write(FLASH_ADDR_MAIN);

  tracker_exstatus_prev = tracker_exstatus;*/ //TODO
}

void ClearStatus(uint8_t motor_id) {

  unsigned int *tracker_status_p;
  unsigned int *tracker_exstatus_p;
  if(motor_id == 0){
	  tracker_status_p = &tracker_status;
	  tracker_exstatus_p = &tracker_exstatus;
  }
  else{
	  tracker_status_p = &tracker_status2;
	  tracker_exstatus_p = &tracker_exstatus2;
  }
  crc_errors       = 0;
  *tracker_status_p &= ~(0xff | SF_HALL_WIRING_A | SF_HALL_WIRING_B | SYS_PARAM_FLASH_ERR | SF_POWER_FAILURE | SF_MOVING_OUT_A | SF_MOVING_IN_A | SF_MOVING_REF_CLR_A | SF_ENDSW_A_LO_PRESSED | SF_ENDSW_A_HI_PRESSED | SF_MOVING_OUT_B | SF_MOVING_IN_B | SF_MOVING_REF_CLR_B | SF_ENDSW_B_LO_PRESSED | SF_ENDSW_B_HI_PRESSED | SF_NO_MODBUS);			//brisi zastavice
  *tracker_exstatus_p &= ~EFS_ERROR_CLEAR;
  bldc_ClearStatus();
  //void ClearStatus (void) {
  //
  //    tracker_status  = 0;			//brisi zastavice
  //    H_direrror      = 0;
  //    V_direrror      = 0;
   //   motor_clear_status();
      focus_restart();
      backup_timeout=BACKUP_TO;		//4 sekunde zatem backup v flash
      flags&=~(1<<Modbus_timeout);
  //}
}

/***********************************************************
 BUTTONS
************************************************************/
unsigned char ButtonCnt[4];
unsigned char ButtonStatus;

void ButtonDebounce() {
  int value = ButtonStates();

  for (int i = 0; i < 3; i++) {
    if (ButtonStatus & (1 << i)) {  //button in active state
      if (!(value & (1 << i)))
        ButtonCnt[i]--;
      else if (ButtonCnt[i] < 20)
        ButtonCnt[i]++;

      if(ButtonCnt[i] == 0)
        ButtonStatus &= ~(1<<i);    //deactivate
    } else {
      if (value & (1<<i))
        ButtonCnt[i]++;
      else if (ButtonCnt[i])
        ButtonCnt[i]--;

      if(ButtonCnt[i] >= 20)
        ButtonStatus |= (1<<i);     //activate
    }
  }
}

int ButtonsInvalid(){
  int cnt = 0;

  for (int i = 0; i<3; i++){
    if (ButtonStatus & (1 << i))
      cnt++;
  }
  if (cnt > 1)
    return  1;

  return  0;
}

unsigned char drive_select = 0;
uint16_t select_motor_delay = 0;

void ButtonProcess () {
  ButtonDebounce();

  //****button stuck detection*****
  if (ButtonStatus && !(flags&buttonstuck)) {   // at least one button pressed
    if (button_timeout++ > (600*1000)) {        // 10 min timeout
      flags |= buttonstuck;
      button_timeout = 5000;
    }
  }

//  if (ButtonsInvalid()) {           // two or more buttons pressed deactivate control
//    if (bldc_getStatus(0) & BLDC_STATUS_MANUAL)
//      bldc_Stop(1);
//    if (bldc_getStatus(1) & BLDC_STATUS_MANUAL)
//      bldc_Stop(1);
//    return;
//  }

  if (ButtonStatus == 0) {          // no button pressed
    //deactivate control
    if (bldc_getStatus(0) & BLDC_STATUS_MANUAL && bldc_getStatus(0) & BLDC_STATUS_ACTIVE)
      bldc_Stop(1);
    if (bldc_getStatus(1) & BLDC_STATUS_MANUAL && bldc_getStatus(1) & BLDC_STATUS_ACTIVE)
      bldc_Stop(1);

    //button stuck release
    if (flags & buttonstuck && button_timeout) {
      button_timeout--;
      if (button_timeout == 0) {
        flags &= ~buttonstuck;
      }
    }
    return;
  }

  if (flags & buttonstuck) {
    if (bldc_getStatus(0) & BLDC_STATUS_MANUAL)
      bldc_Stop(1);
    if (bldc_getStatus(1) & BLDC_STATUS_MANUAL)
      bldc_Stop(1);
    return;//button stuck manual control disabled
  }

  if(select_motor_delay > 400 && select_motor_delay < 1200){  //delay between motor selection
    select_motor_delay ++;
    return;
  }

  if (ButtonStatus == 0x03){
    if (drive_select == 1) drive_select = 0;
    else drive_select = 1;
    select_motor_delay++;
    bldc_manual(1);
    bldc_Stop(1);

    //while(ButtonStates() == 0x03);
    for(int i = 0 ; i < 10000 ; i++)
    select_motor_delay = 500;
    return;
  }


  select_motor_delay++;

  if (select_motor_delay > 300){


    if (ButtonStatus & (1 << 1)) {

      //if((bldc_cm->status & BLDC_STATUS_ACTIVE) && bldc_cm->destination == bldc_Motor(drive_select))            // move OUT
      bldc_manual(0);
      bldc_setPosition(drive_select, bldc_Motor(drive_select)->max_position, 0);
      bldc_manual(1);

    } else if (ButtonStatus & (1<<0)) {       // move IN
      bldc_manual(0);
      bldc_setPosition(drive_select, bldc_Motor(drive_select)->min_position, 0);
      bldc_manual(1);

    } else if (ButtonStatus & (1<<2)) {       // Clear status
      ClearStatus(0);
      ClearStatus(1);

    } else if (ButtonStatus & (1<<3)) {       // Home
      bldc_Home(0);
      bldc_Home(1);
    }
    select_motor_delay = 0;
  }

                    //if(GetMode()) SetMode(MICRO_MODE_STANDALONE);//cancel slave mode
                  //bflags|=(1<<time_enable);           //omogoci avtomatsko gibanje
                  //BKP_WriteBackupRegister(BKP_DR5,bflags);  //write in RTC RAM
                  //Recalc_sun();
}

/***********************************************************
  REST POSITION
************************************************************/



int MotorAisMoving()
{
  return tracker_status & (SF_MOVING_OUT_A | SF_MOVING_IN_A);
}

int MotorBisMoving()
{
  return tracker_status & (SF_MOVING_OUT_B | SF_MOVING_IN_B);
}

/***********************************************************
  event commands
************************************************************/
void ActivateEvent(unsigned int ev)
{
  tracker_exstatus |= EFS_EVENTS_ACTIVE;
  events |= ev;
}

void SetEventParameters(unsigned char motor)
{
  if (motor == 0) {
    err_currentA  = GetAnalogValues(MotorSelectI(motor));
    err_positionA = bldc_position(motor);
    err_voltageA  = GetAnalogValues(SUPPLY);
  }
  else if (motor == 1) {
    err_currentB  = GetAnalogValues(MotorSelectI(motor));
    err_positionB = bldc_position(motor);
    err_voltageB  = GetAnalogValues(SUPPLY);
  }
}

/***********************************************************
  PowerLine resistance commands
************************************************************/
float RMessure_StartU;
float RMessure_AvgU;
unsigned int RMesTimer;

void Measure_Line_Resistance_Start()
{
  bldc_motor* m = bldc_Motor(0);
  bldc_motor* mb = bldc_Motor(1);

  if (flags & (1 << lineR_measure))
    return;
  if (bldc_Status() & BLDC_LOCKED || m->status&BLDC_STATUS_ERR || (!(m->state&BLDC_MOTOR_STATE_ENABLED))) {
    ActivateEvent(EVENT_LINE_R_CANCELED);
    return;
  }
  if(bldc_Status() & BLDC_LOCKED || mb->status&BLDC_STATUS_ERR || (!(mb->state&BLDC_MOTOR_STATE_ENABLED))) {
    ActivateEvent(EVENT_LINE_R_CANCELED);
    return;
  }

  bldc_Stop(1);
  flags|= (1 << lineR_measure);
  RMessure_AvgU=RMessure_StartU = GetAnalogValues(SUPPLY);
  RMesTimer = 0;

  if ((m->min_position + bldc_position(0) ) >= (m->max_position - bldc_position(0)))
    bldc_setPosition(0, m->min_position, 0);
  else
    bldc_setPosition(0, m->max_position, 0);

  if ((mb->min_position + bldc_position(1) ) >= (mb->max_position - bldc_position(1)))
    bldc_setPosition(1, mb->min_position, 0);
  else
    bldc_setPosition(1, mb->max_position,0);

  StatusUpdate();
}

void RMeasure_Stop()
{
  if (flags & (1 << lineR_measure)) {
    flags &= ~(1 << lineR_measure);
    ActivateEvent(EVENT_LINE_R_CANCELED);
    bldc_Stop(1);
  }
}

void Measure_Line_Resistance() {
  if (!(flags & (1 << lineR_measure)))
    return;

  RMesTimer++;

  if (RMesTimer < 10) {
     RMessure_StartU = (RMessure_StartU + GetAnalogValues(SUPPLY)) / 2;
     flags |= (1 << lineR_init);
  }
  RMessure_AvgU = (RMessure_AvgU+GetAnalogValues(SUPPLY)) / 2;
  if (MotorAisMoving() || MotorBisMoving()) {
    if (RMesTimer >= 3000) {
      LineResistance = (RMessure_StartU - GetAnalogValues(SUPPLY)) / GetAnalogValues(MotorSelectI(bldc_cm->index));
      if (LineResistance > max_line_resistance) {
	tracker_exstatus |= EFS_LINE_RESISTANCE_HIGH;
      }
      flags &= ~(1 << lineR_measure);
      bldc_Stop(1);

      ActivateEvent(EVENT_LINE_R_FINISHED);
    }
  } else
    RMeasure_Stop();    //cancel
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
