#include "flash.h"
#include "Shared_Libraries/modbus.h"

#define ACK_OK      0x00
#define ACK_ERROR   0xFF
IAP iap_entry;


/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

uint32_t Address = 0, PAGEError = 0;




extern bldc_motor bldc_motors[BLDC_MOTOR_COUNT];

/* IAP - flash write (parameters backup) */
unsigned int command[5]; 	//IAP - spremenljivki
unsigned int result[5];
float        flash_backup[FLASH_USER_SIZE]; 	//polje spremenljivk, ki se zapisejo v flash
unsigned int FlashWriteCounter;
unsigned int crc_flash;
int *lflags_p = (int*)&lflags;




void read_SN(){

  // read unique ID via IAP
  SN[0] = HAL_GetUIDw0();
  SN[1] = HAL_GetUIDw1();
  SN[2] = HAL_GetUIDw2();
  SN[3] = 0xaaaabbbb;
}

/***********************************************************
  FLASH READ/WRITE

************************************************************/
typedef struct {
  unsigned int LOADER_VER;
  unsigned int HW_REV;
  unsigned int DEV_TYPE;
  unsigned int DEV_MIN_VERSION;
  unsigned int FLASH_APP_SIZE;
  unsigned int FLASH_APP_START_ADDRESS;
  unsigned int reseved[58];
} system_defs_t;

uint32_t sysdefs[200];
uint32_t flashError;

void sys_data_write(void){


	sysdefs[0] = 5000;
	sysdefs[1] = 1;
	sysdefs[2] = 5;
	sysdefs[3] = 5000;
	sysdefs[4] = 0x78000;
	sysdefs[5] = 0x8000;

	HAL_FLASH_Unlock();

	  /* Clear OPTVERR bit set on virgin samples */
	    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

	  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	  EraseInitStruct.Banks       = FLASH_BANK_1;
	  EraseInitStruct.Page        = 252;
	  EraseInitStruct.NbPages     = 3;

	  if( HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK){
		  flashError = HAL_FLASH_GetError();
	  }

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	for (int i=0 ; i<4 ; i++){
	    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, BOOT_VERSION_ADDR + i*8, (uint32_t)sysdefs[i]) != HAL_OK)
	    {
	    	flashError = HAL_FLASH_GetError();
	    }
	}
}


void flash_read (unsigned int read_address) {

 float *flash_backup = (float *)read_address;
 crc_flash = modbus_crc((uint8_t*)flash_backup, FLASH_USER_SIZE_BYTES-2, CRC_NORMAL);

   if( crc_flash != 0){
    if(read_address == FLASH_ADDR_MAIN){
    	flash_read (FLASH_ADDR_BACKUP);
    }
    tracker_status |= SYS_PARAM_FLASH_ERR;
  }

#ifdef BOOTLOADER
  slave_addr = (int)flash_backup[0];
  module.channel = (int)flash_backup[64];
  module.power = (int)flash_backup[65];
  module.spFactor = (int)flash_backup[66]; 
  module.LoRa_BW = (int)flash_backup[67];
  transceiver = (int)flash_backup[68];
  bflags = (int)flash_backup[147];

  return;
#endif

  slave_addr = flash_backup[0];       
  FlashWriteCounter = flash_backup_ui[1];
  //tracker_status    = flash_backup_ui[2];
  //tracker_exstatus  = flash_backup_ui[3];
  bldc_initStatus( flash_backup_ui[4] );

  events            = flash_backup_ui[6];
  
  err_currentA      = flash_backup[7];
  err_positionA     = flash_backup[8];
  err_voltageA      = flash_backup[9];

  max_line_resistance = flash_backup[10];
  //auto_state     = flash_backup_ui[11];

  modbus_timeout       = flash_backup_ui[12];
  modbus_timeout_delay = flash_backup_ui[13];

  // Motor A driving settings
  bldc_motor *m = bldc_Motor(0);

  bldc_config()->UConvertRatio  = flash_backup[14];         
  bldc_config()->IConvertRatio  = flash_backup[15];
  bldc_config()->homing_timeout = flash_backup_ui[16];
  bldc_config()->BConvertRatio  = flash_backup[58];
  bldc_config()->HConvertRatio  = flash_backup[59];
  bldc_config()->H1ConvertRatio  = flash_backup[60];

  m->state                    = flash_backup[17]; 
  m->status                   = flash_backup[18]; 
  m->position                 = flash_backup[19]; 
  m->min_position             = flash_backup[20];
  m->max_position             = flash_backup[21];
  m->Idetection               = flash_backup[22];
  m->I_limit                  = flash_backup[23];   
  m->I_Inrush_ratio           = flash_backup[24];
  m->I_Inrush_time            = flash_backup[25];
  m->modbus_timeout_position  = flash_backup[26];
  m->gear_ratio               = flash_backup[27];
     
  m->home_remaining           = flash_backup[28];   
  m->home_offset              = flash_backup[29];
  m->end_switchDetect         = flash_backup[30];

  m->ramp               	  = flash_backup[31];
  m->speed_freewheel          = flash_backup[32];

#if BLDC_MOTOR_COUNT > 1
  // Motor B driving settings
  bldc_motor *mb = bldc_Motor(1);

  err_currentB      = flash_backup[35];
  err_positionB     = flash_backup[36];
  err_voltageB      = flash_backup[37];

  mb->state                    = flash_backup[38];  
  mb->status                   = flash_backup[39];  
  mb->position                 = flash_backup[40];  
  mb->min_position             = flash_backup[41];
  mb->max_position             = flash_backup[42];
  mb->Idetection               = flash_backup[43];
  mb->I_limit                  = flash_backup[44];    
  mb->I_Inrush_ratio           = flash_backup[45];
  mb->I_Inrush_time            = flash_backup[46];
  mb->modbus_timeout_position  = flash_backup[47];
  mb->gear_ratio               = flash_backup[48];
       
  mb->home_remaining           = flash_backup[49];    
  mb->home_offset              = flash_backup[50];
  mb->end_switchDetect         = flash_backup[51];
  
  mb->ramp                	   = flash_backup[52];
  mb->speed_freewheel          = flash_backup[53];

#endif
  //longitude                    = flash_backup[57];
  //latitude                     = flash_backup[67];
  //time_zone                    = flash_backup[118];

  voltage_select             = flash_backup[61];

  module.channel               = flash_backup[64];
  module.power                 = flash_backup[65];
  module.spFactor              = flash_backup[66]; 
  module.LoRa_BW               = flash_backup[67];
  transceiver_saved            = flash_backup[68];
  LoRa_id                      = flash_backup[96];

  ES_0_normallyOpenLo = ((unsigned int)flash_backup[149]) & (1 << 0);
  ES_1_normallyOpenLo = (((unsigned int)flash_backup[149]) >> 1) & (1 << 0);
  ES_0_normallyOpenHi = (((unsigned int)flash_backup[149]) >> 2) & (1 << 0);
  ES_1_normallyOpenHi = (((unsigned int)flash_backup[149]) >> 3) & (1 << 0);

  number_of_poles = (unsigned int)flash_backup[150];
  motor_operation = (unsigned int)flash_backup[153];


  if(f_pcb_version == 0)
    f_pcb_version = 0x1E4601; // KVARK

  pcb_version3=(int)f_pcb_version;             //verzija TIV (u8) <- parametri (double)
  pcb_version2=(int)f_pcb_version/0x100;
  pcb_version1=(int)f_pcb_version/0x10000;

  reset_status = flash_backup[56];


  tracker_status    = flash_backup_ui[2];
  tracker_exstatus  = flash_backup_ui[3];

  m->position                 = flash_backup[19];
#if BLDC_MOTOR_COUNT == 2
  //bldc_motor *mb = bldc_Motor(1);
  //mb->position                 = flash_backup[40];
#endif
  cflags=(int)f_cflags;                        //configuration flags (u32) <- parametri (double)
  bflags=(int)f_bflags;
  buyflags=(int)f_buyflags;                    //buying flags (u32) <- parametri (double)

  FocusMiddleA=(int)f_FocusMiddleA;                 //externi light sensor
  FocusMiddleB=(int)f_FocusMiddleB;

  // last sync time
  lastSyncTime.seconds = (int)lastSync_time % 100;
  lastSyncTime.minutes = ((int)lastSync_time / 100) % 100;
  lastSyncTime.hours = (int)lastSync_time / 10000;
  lastSyncTime.date = (int)lastSync_date % 100;
  lastSyncTime.month = ((int)lastSync_date / 100) % 100;
  lastSyncTime.year = (int)lastSync_year;

}



int flash_write(unsigned int write_address) {

  uint8_t write_page;
  if(write_address == FLASH_ADDR_MAIN){
    write_page = FLASH_USER_PAGE_1;
  }
  else{
    write_page = FLASH_USER_PAGE_1 + 1;
  }

#ifdef BOOTLOADER
  flash_backup[0] = (float)slave_addr;
  flash_backup[64] = (float)module.channel;
  flash_backup[65] = (float)module.power;
  flash_backup[66] = (float)module.spFactor; 
  flash_backup[67] = (float)module.LoRa_BW;
  flash_backup[68] = (float)transceiver;
  flash_backup[147] = (float)bflags;
#else
  // upgrade indicator
  int upgrExe = 0;
  if(upgrExe_gl){ //always write flash, when received from usb
    upgrExe = 1;
    upgrExe_gl = 0;
  }
  
  // system settings
  if(flash_backup[0] != (float)slave_addr) {
    flash_backup[0] = slave_addr;
    upgrExe = 1;
  }
  if(flash_backup_ui[2] != tracker_status) {
    flash_backup_ui[2] = tracker_status;
    upgrExe = 1;
  }
  if(flash_backup_ui[3] != tracker_exstatus) {
    flash_backup_ui[3] = tracker_exstatus;
    upgrExe = 1;
  }
  if(flash_backup_ui[4] != bldc_Status()) {
    flash_backup_ui[4] = bldc_Status();
    upgrExe = 1;
  }
  if(flash_backup_ui[6] != events) {
    flash_backup_ui[6] = events;
    upgrExe = 1;
  }
	
  if(flash_backup[7] != err_currentA) {
    flash_backup[7] = err_currentA;
  }
  if(flash_backup[8] != err_positionA) {
    flash_backup[8] = err_positionA;
  }
  if(flash_backup[9] != err_voltageA) {
    flash_backup[9] = err_voltageA;
  }

  if(flash_backup[10] != max_line_resistance) {
    flash_backup[10] = max_line_resistance;
    upgrExe = 1;
  }
//  if(flash_backup_ui[11] != auto_state) {
//    flash_backup_ui[11] = auto_state;
//    upgrExe = 1;
//  }

  if(flash_backup_ui[12] != modbus_timeout) {
    flash_backup_ui[12] = modbus_timeout;
    upgrExe = 1;
  }
  if(flash_backup_ui[13] != modbus_timeout_delay) {
    flash_backup_ui[13] = modbus_timeout_delay;
    upgrExe = 1;
  }

  // Motor A driving settings
  bldc_motor *m = bldc_Motor(0);

  if(flash_backup[14] != bldc_config()->UConvertRatio) {			
    flash_backup[14] = bldc_config()->UConvertRatio;
    upgrExe = 1;
  }			
  if(flash_backup[15] != bldc_config()->IConvertRatio) {
    flash_backup[15] = bldc_config()->IConvertRatio;
    upgrExe = 1;
  }
  if(flash_backup_ui[16] != bldc_config()->homing_timeout) {
    flash_backup_ui[16] = bldc_config()->homing_timeout;
    upgrExe = 1;
  }
  if(flash_backup[58] != bldc_config()->BConvertRatio) {
    flash_backup[58] = bldc_config()->BConvertRatio;
    upgrExe = 1;
  }
  if(flash_backup[59] != bldc_config()->HConvertRatio) {
    flash_backup[59] = bldc_config()->HConvertRatio;
    upgrExe = 1;
  }
  if(flash_backup[60] != bldc_config()->H1ConvertRatio) {
    flash_backup[60] = bldc_config()->H1ConvertRatio;
    upgrExe = 1;
  }

  if(flash_backup[17] != (float)m->state) {
    flash_backup[17] = m->state;
    upgrExe = 1;
  }
  if(flash_backup[18] != (float)m->status) {
    flash_backup[18] = m->status;
    upgrExe = 1;
  }
  if(flash_backup[19] != (float)m->position) {
    flash_backup[19] = m->position;
    upgrExe = 1;
  }
  if(flash_backup[20] != m->min_position) {
    flash_backup[20] = m->min_position;
    upgrExe = 1;
  }
  if(flash_backup[21] != m->max_position) {
    flash_backup[21] = m->max_position;
    upgrExe = 1;
  }
  if(flash_backup[22] != m->Idetection) {
    flash_backup[22] = m->Idetection;
    upgrExe = 1;
  }
  if(flash_backup[23] != m->I_limit) {
    flash_backup[23] = m->I_limit;
    upgrExe = 1;
  }
  if(flash_backup[24] != m->I_Inrush_ratio) {
    flash_backup[24] = m->I_Inrush_ratio;
    upgrExe = 1;
  }
  if(flash_backup[25] != m->I_Inrush_time) {
    flash_backup[25] = m->I_Inrush_time;
    upgrExe = 1;
  }
  if(flash_backup[26] != m->modbus_timeout_position) {
    flash_backup[26] = m->modbus_timeout_position;
    upgrExe = 1;
  }
  if(flash_backup[27] != m->gear_ratio) {
    flash_backup[27] = m->gear_ratio;
    upgrExe = 1;
  }
     
  if(flash_backup[28] != (float)m->home_remaining) {
    flash_backup[28] = m->home_remaining;
    upgrExe = 1;
  }
  if(flash_backup[29] != m->home_offset) {
    flash_backup[29] = m->home_offset;
    upgrExe = 1;
  }
  if(flash_backup[30] != m->end_switchDetect) {
    flash_backup[30] = m->end_switchDetect;
    upgrExe = 1;
  }

  if(flash_backup[31] != m->ramp) {
    flash_backup[31] = m->ramp;
    upgrExe = 1;
  }

  if(flash_backup[32] != m->speed_freewheel) {
    flash_backup[32] = m->speed_freewheel;
    upgrExe = 1;
  }


//#if BLDC_MOTOR_COUNT == 2
  // Motor B driving settings
  bldc_motor *mb = bldc_Motor(1);

  if(flash_backup[35] != err_currentB) {
    flash_backup[35] = err_currentB;
  }
  if(flash_backup[36] != err_positionB) {
    flash_backup[36] = err_positionB;
  }
  if(flash_backup[37] != err_voltageB) {
    flash_backup[37] = err_voltageB;
  }

  if(flash_backup[38] != (float)mb->state) {
    flash_backup[38] = mb->state;
    upgrExe = 1;
  }
  if(flash_backup[39] != (float)mb->status) {
    flash_backup[39] = mb->status;
    upgrExe = 1;
  }
  if(flash_backup[40] != (float)mb->position) {
    flash_backup[40] = mb->position;
    upgrExe = 1;
  }
  if(flash_backup[41] != mb->min_position) {
    flash_backup[41] = mb->min_position;
    upgrExe = 1;
  }
  if(flash_backup[42] != mb->max_position) {
    flash_backup[42] = mb->max_position;
    upgrExe = 1;
  }
  if(flash_backup[43] != mb->Idetection) {
    flash_backup[43] = mb->Idetection;
    upgrExe = 1;
  }
  if(flash_backup[44] != mb->I_limit) {
    flash_backup[44] = mb->I_limit;
    upgrExe = 1;
  }
  if(flash_backup[45] != mb->I_Inrush_ratio) {
    flash_backup[45] = mb->I_Inrush_ratio;
    upgrExe = 1;
  }
  if(flash_backup[46] != mb->I_Inrush_time) {
    flash_backup[46] = mb->I_Inrush_time;
    upgrExe = 1;
  }
  if(flash_backup[47] != mb->modbus_timeout_position) {
    flash_backup[47] = mb->modbus_timeout_position;
    upgrExe = 1;
  }
  if(flash_backup[48] != mb->gear_ratio) {
    flash_backup[48] = mb->gear_ratio;
    upgrExe = 1;
  }
       
  if(flash_backup[49] != (float)mb->home_remaining) {
    flash_backup[49] = mb->home_remaining;
    upgrExe = 1;
  }
  if(flash_backup[50] != mb->home_offset) {
    flash_backup[50] = mb->home_offset;
    upgrExe = 1;
  }
  if(flash_backup[51] != mb->end_switchDetect) {
    flash_backup[51] = mb->end_switchDetect;
    upgrExe = 1;
  }
  
  if(flash_backup[52] != mb->ramp) {
    flash_backup[52] = mb->ramp;
    upgrExe = 1;
  }
  if(flash_backup[53] != mb->speed_freewheel) {
    flash_backup[53] = mb->speed_freewheel;
    upgrExe = 1;
  }
//#endif
  if(flash_backup[56] != (float)reset_status) {
    flash_backup[56] = reset_status;
    upgrExe = 1;
  }

  if(flash_backup[61] != (float)voltage_select) {
    flash_backup[61] = voltage_select;
    upgrExe = 1;
  }

  if(flash_backup[64] != (float)module.channel) {
    flash_backup[64] = module.channel;
    upgrExe = 1;
  }
  if(flash_backup[65] != (float)module.power) {
    flash_backup[65] = module.power;
    upgrExe = 1;
  }
  if(flash_backup[66] != (float)module.spFactor) {
    flash_backup[66] = module.spFactor;
    upgrExe = 1;
  }
  if(flash_backup[67] != (float)module.LoRa_BW) {
    flash_backup[67] = module.LoRa_BW;
    upgrExe = 1;
  }
  if(flash_backup[68] != (float)transceiver) {
    flash_backup[68] = transceiver;
    upgrExe = 1;
  }
  if(flash_backup[96] != (float)LoRa_id) {
    flash_backup[96] = LoRa_id;
    upgrExe = 1;
  }

  if(flash_backup[149] != (float)(ES_0_normallyOpenLo + (ES_1_normallyOpenLo << 1) + (ES_0_normallyOpenHi << 2) + (ES_1_normallyOpenHi << 3))) {
    flash_backup[149] = ES_0_normallyOpenLo + (ES_1_normallyOpenLo << 1) + (ES_0_normallyOpenHi << 2) + (ES_1_normallyOpenHi << 3);
    upgrExe = 1;
  }

  if(flash_backup[150] != (float)number_of_poles) {
    flash_backup[150] = number_of_poles;
    upgrExe = 1;
  }

  if(flash_backup[153] != (float)motor_operation) {
    flash_backup[153] = motor_operation;
    upgrExe = 1;
  }

  if(f_cflags != (float)cflags) {
    f_cflags = cflags;                        //configuration flags (u32) -> parametri (double)
    upgrExe = 1;
  }
  if(f_bflags != (float)bflags) {
    f_bflags = bflags;
    upgrExe = 1;
  }
  if(f_buyflags != buyflags) {
    f_buyflags = buyflags;                    //buying flags (u32) -> parametri (double)
    upgrExe = 1;
  }

  // last sync time
  if(lastSync_year != (float)lastSyncTime.year) {
    lastSync_year = lastSyncTime.year;
    upgrExe = 1;
  }
  if(lastSync_date != (float)(lastSyncTime.month * 100 + lastSyncTime.date)) {
    lastSync_date = lastSyncTime.month * 100 + lastSyncTime.date;
    upgrExe = 1;
  }
  if(lastSync_time != (float)(lastSyncTime.hours * 10000 + lastSyncTime.minutes * 100 + lastSyncTime.seconds)) {
    lastSync_time = lastSyncTime.hours * 10000 + lastSyncTime.minutes * 100 + lastSyncTime.seconds;
    upgrExe = 1;
  }
     
  // no changes
  if(!upgrExe)
    return 0;

#endif

  flash_backup_ui[1] = ++FlashWriteCounter;

  crc_flash = modbus_crc((uint8_t *)flash_backup, FLASH_USER_SIZE_BYTES-4, CRC_NORMAL);
  *(int *)&flash_backup[FLASH_USER_SIZE-1] = crc_flash;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = FLASH_BANK_1;
  EraseInitStruct.Page        = write_page;
  EraseInitStruct.NbPages     = 1;

  if( HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK){
     tracker_status |= SYS_PARAM_FLASH_ERR;  
     flashError = HAL_FLASH_GetError();
  }


  uint32_t Address = write_address;
  uint64_t* flash_backup_dw = (uint64_t*)flash_backup; //Double word

  //Write flash row by row
  while (Address < (write_address + FLASH_USER_SIZE_BYTES) )
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, *(flash_backup_dw++)) == HAL_OK)
    {
      Address = Address + sizeof(uint64_t);
    }
   else
    {
      /* Error occurred while writing data in Flash memory.*/
      tracker_status |= SYS_PARAM_FLASH_ERR;
      flashError = HAL_FLASH_GetError();
      Error_Handler();
    }
  }
return 0;
}
