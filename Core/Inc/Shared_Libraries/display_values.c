/**************************************************************************
 *		"Pico" solar positioner software
 *
 *		filename: modbus.c
 *		pcb: tiv29B
 *
 *		Copyright(C) 2011, Sat Control d.o.o.
 *		All rights reserved.
**************************************************************************/

#include "display_values.h"
#include "suntracer.h"
#include "asciibox.h"
#include "write_values.h"
#include "../rtc.h"
#include "sun.h"
#include "usbd_cdc_if.h"

#include "bldc.h"
#include "../main.h"
#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


////////////////////////////

uint32_t screen_mux_A;		//high speed multiplex counter for sending divided strings to PC menu
uint32_t screen_mux_B;               //medium speed multiplex counter for sending divided strings to PC menu
uint32_t screen_mux_C;               //slow speed multiplex counter for sending divided strings to PC menu
unsigned int USBlink_ok;        //preverjanje ce komunikacija (refresh) dela - povecevanje stevila
unsigned int RS485link_ok;
extern unsigned int crc_calc;
extern uint8_t slave_addr;
//unsigned char CharTemp[10];

/////////////
 unsigned char pcb_version1;                //TIV           27   27C1=0x1B 0x43 0x01 .... 0x1B=27 0x43='C' 0x01=1 .... vpisi 0x1B4301 oziroma 1786625
 char pcb_version2;                         //verzija TIVa  C
 unsigned char pcb_version3;                //TIV polaganje 1
/////////////
 extern unsigned int flags;
 extern unsigned int tracker_status;
 unsigned int H_direrror;                 //impulz kot napacna smer - motnje motorcka
 unsigned int V_direrror;
 unsigned int WindWaitCnt;
 unsigned int SnowWaitCnt;
 extern double show_angle_A;
 extern double show_angle_B;
 float Imotor_A;
 float Imotor_B;
 extern float parameters [N_parameters];
 extern  float parameters_fixed [N_parameters_fixed];
 uint32_t ID_number_1;
 uint32_t ID_number_2;
 uint32_t ID_number_3;
 float Usolar;
 MODE_TYPE mode;
 float WindSpeed;
 uint32_t dichecksum;
 uint32_t buyflags;
 uint8_t buffer_out[256];
 extern uint8_t g_rxBuff[256];
// uint32_t green_led;
 unsigned int BldVer;
 float Isolar;

 unsigned int FocusMiddleA,FocusMiddleB;
 float offset_A,offset_B;
 uint32_t adc3_VAL;
 uint32_t adc4_VAL;

extern time_t_ time;
extern time_t_ GMTtime;

extern time_t_ lastSyncTime;

//USART3
// unsigned char USART_TxBuffer[USART_TXBUFFSIZE];
// unsigned char USART_RxBuffer[USART_RXBUFFSIZE];
 volatile unsigned int USART_RxCounter;
 volatile unsigned int USART_TxCounter;
 unsigned int USART_NrTxBytes;         //koliko jih je za oddajo
 unsigned int USART_RxEndTimer;		//stevec, ki doloci konec stringa


 uint32_t CAN_send_strings;        //zahteva za oddajo CAN stringov menija za "web monitor"
 uint32_t adv_CAN_send_strings;    //zahteva za oddajo CAN stringov za "advanced system editor"
// CanRxMsg RxMessage;
 uint32_t debug_multiplex_can;	

 extern float parameters [N_parameters];
extern float parameters_fixed [N_parameters_fixed];
 uint32_t cflags;
 unsigned int H_direrror;                 //impulz kot napacna smer - motnje motorcka
 unsigned int V_direrror;

 unsigned int rtc_counter;
 unsigned int day_of_year;
 uint16_t months_length[13];
 unsigned int store_in_flash;
 extern unsigned int flags;
 unsigned int bflags;
 unsigned int no_run_time;
 unsigned int ref_toolong_cnt_A;
 unsigned int ref_toolong_cnt_B;
 unsigned char move_direction;
 unsigned int error_motor_A;		
 unsigned int error_hall_A;	  	
 unsigned int error_cable_A;		
 unsigned int error_motor_B;		
 unsigned int error_hall_B;	  	
 unsigned int error_cable_B;		
 extern unsigned int backup_timeout;
 volatile int delay_reset = 0;
 extern uint32_t green_led;
 unsigned int FocusMiddleA,FocusMiddleB;
 uint32_t adc3_VAL;
 uint32_t adc4_VAL;


//static uint8_t g_rxBuff[256];
unsigned char Str[256];
uint32_t prompt = 0, rdCnt = 0;

extern double AE_azimuth;		
extern double AE_elevation;		
extern double PM_azimuth;		
extern double PM_elevation;

extern const Version swVersion;

extern unsigned char ES_0_normallyOpenLo;
extern unsigned char ES_0_normallyOpenHi;
extern unsigned char ES_1_normallyOpenLo;
extern unsigned char ES_1_normallyOpenHi;

extern volatile unsigned int bldc_Speed;
extern volatile unsigned int number_of_poles;
extern int baudrate;
extern unsigned int motor_operation;

extern unsigned int SN[4];
extern uint8_t LoRa_id;


void USART_To_USB_Send_Data(uint8_t* ascii_string, uint32_t count_in){

       /* if (vcom_connected() != 0) {
                vcom_write(&Str[0], count_in);
        }    */
	//USBD_CDC_SetTxBuffer(pdev, &Str[0], count_in);
	if(count_in > 255)
		return;
	CDC_Transmit_FS(&Str[0], count_in);
	//USBD_CDC_TransmitPacket(pdev);

	//USBD_CDC_GetDeviceQualifierDescriptor(length);

}


/***********************************************************
    USB DISPLAY VALUES

VIRTUAL_COM_PORT_DATA_SIZE = 64
************************************************************/
void USB_display(void) {

  int buf2pc_cnt = 0;

  /********************* mux A ********************************/
  if (++screen_mux_A==1) {

      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2lu",chours,time.hours);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2lu",cminutes,time.minutes);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2lu",cseconds,time.seconds);

#ifndef DISABLE_FOCUS          
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4lu",cFocusSensorOutputA,adc4_VAL);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4lu",cFocusSensorOutputB,adc3_VAL);
#endif
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
  }

  if (screen_mux_A==2){
      show_angle_A = geometry_selection_show (1, (unsigned int)geometry_mode_A);
		buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cshow_angle_A,show_angle_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4f",cposition_A, bldc_position(0));
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4f",cdestination_A, bldc_target(0));
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f",cImotor_A, GetAnalogValues(CURRENT_A));
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4f",cremain_A, bldc_remaining(0));
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
  }

  if (screen_mux_A==3) {
#ifndef DISABLE_MOTOR_B
          show_angle_B = geometry_selection_show (2, (unsigned int)geometry_mode_B);
          buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cshow_angle_B,show_angle_B);
          buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4f",cposition_B, bldc_position(1));
          buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4f",cdestination_B, bldc_target(1));
          buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f",cImotor_B, GetAnalogValues(CURRENT_A));
          buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4f",cremain_B, bldc_remaining(1));
          USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
#else 
     screen_mux_A=4;
#endif           
  }
  
  if (screen_mux_A==4) {
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2lu" ,cGMT_sec ,GMTtime.seconds);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2lu" ,cGMT_min ,GMTtime.minutes);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2lu" ,cGMT_hour,GMTtime.hours);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2lu" ,cGMT_day ,GMTtime.date);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);  
  }
  
  if (screen_mux_A==5) {
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2lu",cGMT_mon ,GMTtime.month);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%lu"  ,cGMT_year,GMTtime.year);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%cRun:%s,Wi=%d,Si=%d",cService,(cflags&(1<<RunBothMotors))?"synch":"normal",WindWaitCnt/50,SnowWaitCnt/50);
#ifndef DISABLE_FOCUS             
      if(cflags&(1<<FocusSensInUse)){
           if(1)buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],",F:seek");
          else           buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],",F:avg");
      }
#endif         
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);  
  }


  if (screen_mux_A==6) {
    /********************* mux B ********************************/
    if(++screen_mux_B==1){
     
#ifndef DISABLE_FOCUS 
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4d",cFocusMiddleA,FocusMiddleA);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4d",cFocusMiddleB,FocusMiddleB);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cFocusOffsetA,offset_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cFocusOffsetB,offset_B);
#endif
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }
//#define VERSION  42
    if(screen_mux_B==2){
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2lu",cdate,time.date);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2lu",cmonth,time.month);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.3f (B%.3f)", cversion, ((float)swVersion.sw_version) / 1000.0, ((float)BOOT_VERSION) / 1000.0);
      
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f",cUsolar,GetAnalogValues(SUPPLY));
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%08X-%08X-%08X-%08X", cID_number, SN[0], SN[1], SN[2], SN[3]);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }

    if(screen_mux_B==3){
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt], "$%c%.4d", cStatus, tracker_status);
      buf2pc_cnt+=sprintf((char*)&Str[buf2pc_cnt],"$%c",cMode);
      switch(mode){
                  case MODE_SLAVE:    buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"sigma standby");
                  break;  
                  case MODE_SLAVE_TRACKING:    buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"sigma tracking");
                  break;
                  case MODE_OK:       buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"tracker ok");
                  break;
                  case MODE_2FAR:     buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"the sun is too far");
                  break;
                  case MODE_DISABLE:  buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"tracking disabled");
                  break;
                  case MODE_WIND:     buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"wind mode");
                  break;
                  case MODE_SNOW:     buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"snow mode");
                  break;
                  case MODE_MICRO:    buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"Micro tracking");
      }
      
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }

    if(screen_mux_B==4){

     
#ifndef DISABLE_MOTOR_B   
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cgoref_Nday_B,goref_Nday_B);
#endif            
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cWidePanelA,WidePanelA);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cSpacePanelA,SpacePanelA);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cgeometry_mode_A,geometry_mode_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",crtc_correction,rtc_correction);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cgoref_Nday_A,goref_Nday_A);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }

    if(screen_mux_B==5){
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cA1_A,A1_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cA2_A,A2_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cA3_A,A3_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cA4_A,A4_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cA5_A,A5_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cA6_A,A6_A);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }

    if(screen_mux_B==6){ 
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cB1_A,B1_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cB2_A,B2_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cgear_ratio_A,gear_ratio_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cmax_range_A,max_range_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",ccoordinate_mode_A,coordinate_mode_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4f",clatitude,latitude);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }
    if(screen_mux_B==7){
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cusolar_factor,usolar_factor);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cimotor_factor_A,imotor_factor_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4lu",ccflags,cflags);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4d", cnormally_close_es,
        ES_0_normallyOpenLo + (ES_1_normallyOpenLo << 1) + (ES_0_normallyOpenHi << 2) + (ES_1_normallyOpenHi << 3));
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4f",clongitude,longitude);
#ifndef DISABLE_MOTOR_B 
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cimotor_factor_B,imotor_factor_B);
#endif
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }

    if(screen_mux_B==8){
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0d",cslave_id,slave_addr);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%d%c%d",cpcb_version,pcb_version1,pcb_version2,pcb_version3);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%d",clink_ok,USBlink_ok);
#ifndef DISABLE_HELIOSTAT 
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",ctarget_H,target_H);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",ctarget_V,target_V);
#endif
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
      if (USBlink_ok++>99)USBlink_ok=0;   //reset 0...99
    }

    if(screen_mux_B==9){
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.4lu",cbuyflags,buyflags);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cinterval,interval);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",crun_delay,run_delay);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cgroup,group);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",crun_delay_home,run_delay_home);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }

    if(screen_mux_B==10){
#ifndef DISABLE_MOTOR_B 
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cA1_B,A1_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cA2_B,A2_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cA3_B,A3_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cA4_B,A4_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cA5_B,A5_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cA6_B,A6_B);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
#else
      screen_mux_B=11;
#endif

    }

    if(screen_mux_B==11){
#ifndef DISABLE_MOTOR_B 
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cB1_B,B1_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cB2_B,B2_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cgear_ratio_B,gear_ratio_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cmax_range_B,max_range_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",ccoordinate_mode_B,coordinate_mode_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cgeometry_mode_B,geometry_mode_B);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
#else
     screen_mux_B=12;
#endif
    }

    
    if(screen_mux_B==12){
      
      if ((int)home_position_A==1000)  buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%coff",chome_position_A); 
      else                             buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",chome_position_A,home_position_A);
       
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cmin_range_A,min_range_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f",cmax_Imotor_A,max_Imotor_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f", czero_offsetA, zero_offsetA);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cSpacePanelB,SpacePanelB);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cWidePanelB,WidePanelB);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }
    
    
    if(screen_mux_B==13){
#ifndef DISABLE_MOTOR_B             
      if ((int)home_position_B==1000)  buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%coff",chome_position_B); 
      else                             buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",chome_position_B,home_position_B);
      
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cmin_range_B,min_range_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f",cmax_Imotor_B,max_Imotor_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f", czero_offsetB, zero_offsetB);
#endif
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cDayMode_time,DayMode_time);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cNightMode_time,NightMode_time);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }
    
    
    if(screen_mux_B==14){
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cInrush_ratioA,Mot_inrush_ratio_A);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cInrush_timeA,Mot_inrush_time_A);
#ifndef DISABLE_MOTOR_B              
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cInrush_ratioB,Mot_inrush_ratio_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cInrush_timeB,Mot_inrush_time_B);  
#endif
      
#ifndef DISABLE_FOCUS 
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cfocus_max_offset  ,focus_max_offset);
#endif
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }

    
    
    if(screen_mux_B==15){
#ifndef DISABLE_SENSORS
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cWindSpeed,WindSpeed);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f",cWindSpeedThreshold,WindSpeedThreshold);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cWindWaitOn,WindWaitOn);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cWindWaitOff,WindWaitOff);
      if ((int)SnowDestinationA==1000) buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%coff",cSnowDestinationA); 
      else                             buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cSnowDestinationA,SnowDestinationA);

      if ((int)WindDestinationA==1000)  buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%coff",cWindDestinationA);
      else                              buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cWindDestinationA,WindDestinationA);
      
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
#else 
      screen_mux_B=16;     
#endif
    }

    
    
    if(screen_mux_B==16){
#ifndef DISABLE_MOTOR_B  
#ifndef DISABLE_SENSORS
      if ((int)SnowDestinationB==1000) buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%coff",cSnowDestinationB);
      else                          buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cSnowDestinationB,SnowDestinationB); 
      
      if ((int)WindDestinationB==1000)  buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%coff",cWindDestinationB);
      else buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cWindDestinationB,WindDestinationB);
#endif
#endif 


      //Hall  voltages

      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%d",cVoltage_select,voltage_select);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f",cUVccHALL_0,GetAnalogValues(HALL));
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }
    
    
    
    if(screen_mux_B==17){
#ifndef DISABLE_SENSORS
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.3f",cWindFactor,WindFactor);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",cWindSensorType,WindSensorType);
      //S
       buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f", cOverTempShift, OverTempShift);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0f",  cTime_out_of_focus, Time_out_of_focus);
       ///S
#endif

      //batery voltage
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f",cBatteryVoltage,GetAnalogValues(BATTERY));

      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }
    
    if(screen_mux_B==18){
       buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f", cdeviation_A  , deviation_A);
       buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f", cinclination_A, inclination_A);
       buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f", cpanel_space_A, panel_space_A);
       buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f", cpanel_width_A, panel_width_A);
       buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f", cpanel_thick_A, panel_thick_A);
      
       USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }
    
    if(screen_mux_B==19){
#ifndef DISABLE_MOTOR_B  
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f", cdeviation_B  , deviation_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f", cinclination_B, inclination_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f", cpanel_space_B, panel_space_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f", cpanel_width_B, panel_width_B);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f", cpanel_thick_B, panel_thick_B);
       
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
#else 
     screen_mux_B=20;
#endif
    } 
    
    if(screen_mux_B==20){
      
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2lu:%.2lu:%.2lu (Local:%.2lu:%.2lu:%.2lu)",csunrise,sun_sunrise()->hours, sun_sunrise()->minutes, sun_sunrise()->seconds,
                                                                                                         sun_local_sunrise()->hours, sun_local_sunrise()->minutes, sun_local_sunrise()->seconds);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2lu:%.2lu:%.2lu (Local:%.2lu:%.2lu:%.2lu)",csunset ,sun_sunset()->hours , sun_sunset()->minutes,  sun_sunset()->seconds,
                                                                                                         sun_local_sunset()->hours,  sun_local_sunset()->minutes,  sun_local_sunset()->seconds);       
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
   
    }
    
    if(screen_mux_B==21){
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%d",clflags,(int)lflags);
#ifndef DISABLE_HELIOSTAT
      if( HTarget2!=1000)buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f",cHTarget2,HTarget2);
      else               buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%cOff" ,cHTarget2);
      
      if( VTarget2!=1000)buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f",cVTarget2,VTarget2);
      else               buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%cOff" ,cVTarget2);
      
#endif
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);

    }
    
    
      if(screen_mux_B==22){
#ifndef DISABLE_HELIOSTAT       
      if( HTarget3!=1000)buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f",cHTarget3,HTarget3);
      else               buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%cOff" ,cHTarget3);
      
      if( VTarget3!=1000)buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2f",cVTarget3,VTarget3);
      else               buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%cOff" ,cVTarget3);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
#else 
      screen_mux_B=23;
#endif
    }
    
    if(screen_mux_B==23){
#ifndef DISABLE_HELIOSTAT
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2d:%.2d",cHeliostatP1_start, (int)HeliostatP1_start/3600, ((int)HeliostatP1_start%3600)/60);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2d:%.2d",cHeliostatP1_end,   (int)HeliostatP1_end/3600  , ((int)HeliostatP1_end%3600)/60);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2d:%.2d",cHeliostatP2_start, (int)HeliostatP2_start/3600, ((int)HeliostatP2_start%3600)/60);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2d:%.2d",cHeliostatP2_end,   (int)HeliostatP2_end/3600  , ((int)HeliostatP2_end%3600)/60);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2d:%.2d",cHeliostatP3_start, (int)HeliostatP3_start/3600, ((int)HeliostatP3_start%3600)/60);    
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
#else
      screen_mux_B=24;
#endif
    }
    
    if(screen_mux_B==24){
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.2d:%.2d",cHeliostatP3_end,   (int)HeliostatP3_end/3600  , ((int) HeliostatP3_end%3600)/60);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%d",cSoftRTCCorrection, (int)SoftRTCCorrection);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%lu",cyear,time.year);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.1f", ctime_zone, time_zone);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt], "$%c%.lf", ctimeDst, time_dst);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt], "$%c%.2lu/%.2lu/%.4lu %.2lu:%.2lu:%.2lu", clastSync_time, lastSyncTime.date, lastSyncTime.month, lastSyncTime.year, lastSyncTime.hours, lastSyncTime.minutes, lastSyncTime.seconds);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }

    if(screen_mux_B==25){
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt], "$%c%d", cmotor_speedA, (int)motor_speedA);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt], "$%c%d", cmotor_speedB, (int)motor_speedB);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt], "$%c%d", covervoltageOcc, (int)overvoltageOcc);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt], "$%c%.2f%c(PM), %.2f%c(AE)", chour_angle, PM_azimuth, 248, AE_azimuth, 248);		
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt], "$%c%.2f%c(PM), %.2f%c(AE)", celevation, PM_elevation, 248, AE_elevation, 248);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }      
    
    if(screen_mux_B==26){
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%d", cbldc_Speed, bldc_Speed);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%d", cnumber_of_poles, number_of_poles);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.3f", cpid_pA, bldc_Motor(0)->ramp);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.3f", cpid_pB, bldc_Motor(1)->ramp);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);
    }

    if(screen_mux_B==27) {
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%.0d", cLoRa_id, LoRa_id);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%d", cchannel, module.channel);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%d", cpower, module.power);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%d", cspread_factor, module.spFactor);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%d", cbandwidth, module.LoRa_BW);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%d", crssi, LoRa_get_rssi() - 164);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%d", cbaudrate, baudrate);
      buf2pc_cnt += sprintf((char *)&Str[buf2pc_cnt],"$%c%d", cmotor_operation, motor_operation);
      USART_To_USB_Send_Data(&Str[0],buf2pc_cnt);

      screen_mux_B = 0;
    }
    screen_mux_A=0;
  }

}    
    






