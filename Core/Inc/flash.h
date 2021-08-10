#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#ifdef APPLICATION
  #include "main.h"
#else
  //#include "bootloader LPC1549/main.h"
  //#define BOOTLOADER

#endif
#include "Shared_Libraries/bldc.h"
#include "Shared_Libraries/SX1278.h"
#include "rtc.h"
#include "Shared_Libraries/config.h"
#include "Shared_Libraries/suntracer.h"
#include "gpio.h"


#define FLASH_USER_SIZE 200     //size in floats
#define FLASH_USER_PAGE_1 254 //burned?
//#define FLASH_USER_PAGE_1 252
#define FLASH_ROW_SIZE 32  		// 32 Double Words
#define BANK1_WRITE_START_ADDR  ((uint32_t)0x08000000) //Main flash memory ; Currently used memory is aliased at 0x0000000
#define BANK1_WRITE_END_ADDR    BANK1_WRITE_START_ADDR + FLASH_BANK_SIZE //0x 0808 0000
#define FLASH_USER_START_ADDR  (BANK1_WRITE_START_ADDR + FLASH_USER_PAGE_1*FLASH_PAGE_SIZE) //0x 0807 f000
#define FLASH_USER_SIZE_BYTES ( FLASH_USER_SIZE * sizeof(float) )

#define FLASH_ADDR_MAIN (BANK1_WRITE_END_ADDR - (FLASH_PAGE_SIZE*2)) //burned?
#define FLASH_ADDR_BACKUP (BANK1_WRITE_END_ADDR - (FLASH_PAGE_SIZE*1))
//#define FLASH_ADDR_MAIN (BANK1_WRITE_END_ADDR - (FLASH_PAGE_SIZE*4))
//#define FLASH_ADDR_BACKUP (BANK1_WRITE_END_ADDR - (FLASH_PAGE_SIZE*3))


#define BOOT_VERSION_ADDR         0x0807ef00
#define BOOT_HW_REV_ADDR          0x0807ef04
#define BOOT_DEVTYPE_ADDR         0x0807ef08
#define BOOT_APP_MINVERSION_ADDR  0x0807ef0C

#define BOOT_VERSION		*((unsigned int *)BOOT_VERSION_ADDR)
#define BOOT_HW_REV		*((unsigned int *)BOOT_HW_REV_ADDR)
#define BOOT_DEVTYPE		*((unsigned int *)BOOT_DEVTYPE_ADDR)
#define BOOT_APP_MINVERSION	*((unsigned int *)BOOT_APP_MINVERSION_ADDR)


typedef void (*IAP)(unsigned int [],unsigned int[]);

#define flash_backup_ui ((unsigned int *)(&flash_backup[0]))
extern uint32_t SystemCoreClock;
extern unsigned int events;
extern unsigned int tracker_exstatus;
extern unsigned int tracker_status;
extern uint32_t cflags;
extern unsigned int bflags;
extern uint32_t buyflags;
extern unsigned int FocusMiddleA,FocusMiddleB;
extern time_t_ lastSyncTime;
extern uint8_t slave_addr;
extern unsigned int FlashWriteCounter;
extern unsigned int reset_status;
extern unsigned int SN[4];	
extern float flash_backup[FLASH_USER_SIZE];

extern float err_currentA;
extern float err_positionA;
extern float err_voltageA;
extern float err_currentB;
extern float err_positionB;
extern float err_voltageB;

extern uint8_t voltage_select_0;
extern uint8_t voltage_select_1;

extern uint8_t LoRa_id;          //LoRa slave ID

extern uint8_t upgrExe_gl;

extern float max_line_resistance;

extern unsigned int modbus_timeout;			// timeout, ko ni MODBUS komunikacije [sekunde]
extern unsigned int modbus_timeout_delay;

extern unsigned char pcb_version1;                //TIV           27   27C1=0x1B 0x43 0x01 .... 0x1B=27 0x43='C' 0x01=1 .... vpisi 0x1B4301 oziroma 1786625
extern char pcb_version2;                         //verzija TIVa  C
extern unsigned char pcb_version3;                //TIV polaganje 1

extern unsigned char ES_0_normallyOpenLo;
extern unsigned char ES_0_normallyOpenHi;
extern unsigned char ES_1_normallyOpenLo;
extern unsigned char ES_1_normallyOpenHi;

extern volatile unsigned int number_of_poles;
extern unsigned int motor_operation;

void flash_read (unsigned int read_address);
void update_flash_backup();
int flash_write(unsigned int write_address);
void read_SN();
void sys_data_write(void);
  


#endif
