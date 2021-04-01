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
#include "dac.h"
#include "dma.h"
#include "app_fatfs.h"
#include "i2c.h"
#include "opamp.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "ICM_20948_C.h"
#include <math.h>
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "ms5637.h"
#include "lwgps.h"
#include "lwrb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define USE_SPI
ICM_20948_Status_e my_write_spi(uint8_t reg, uint8_t* data, uint32_t len, void* user) {
  HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET);
  uint8_t wa = (reg & 0x7F);
  HAL_SPI_Transmit(&hspi3, &wa, 1, 100);
  HAL_SPI_Transmit(&hspi3, data, len, 100);
  HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET);
  return ICM_20948_Stat_Ok;
}
ICM_20948_Status_e my_read_spi(uint8_t reg, uint8_t* buff, uint32_t len, void* user) {
  HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET);
  uint8_t ra = (reg | 0x80);
  HAL_SPI_Transmit(&hspi3, &ra, 1, 100);
  HAL_SPI_Receive(&hspi3, buff, len, 100);
  HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET);
  return ICM_20948_Stat_Ok;
}
const ICM_20948_Serif_t mySerif = {my_write_spi, my_read_spi};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ICM_20948_Device_t myICM;
lwgps_t hgps;
lwrb_t hgps_buff;
uint8_t hgps_buff_data[512];
FATFS FatFS;
FIL fil;
FRESULT fres;

volatile uint8_t update_sensors = 0;
volatile uint8_t update_screen = 0;
volatile uint8_t update_log = 0;
volatile uint32_t timebase = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  CDC_Transmit_FS(buffer, len);
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_QUADSPI1_Init();
  MX_SPI3_Init();
  MX_TIM8_Init();
  MX_ADC2_Init();
  MX_DAC3_Init();
  MX_OPAMP2_Init();
  MX_OPAMP4_Init();
  MX_TIM1_Init();
  MX_TIM20_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_UART5_Init();
  MX_I2C2_Init();
  MX_USB_Device_Init();
  MX_USART1_UART_Init();
  MX_ADC5_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(BUZZER_DAC_GPIO_Port, BUZZER_DAC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPS_RESET_GPIO_Port, GPS_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPS_FORCEON_GPIO_Port, GPS_FORCEON_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPS_EXT_INT_GPIO_Port, GPS_EXT_INT_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(SD_NCS_GPIO_Port, SD_NCS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_SPI_LS_EN_GPIO_Port, IMU_SPI_LS_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  myICM._dmp_firmware_available = true;
  myICM._firmware_loaded = false;
  myICM._last_bank = 255;
  myICM._last_mems_bank = 255;
  myICM._gyroSF = 0; 
  myICM._gyroSFpll = 0;
  myICM._enabled_Android_0 = 0; 
	myICM._enabled_Android_1 = 0; 
  myICM._enabled_Android_intr_0 = 0;
	myICM._enabled_Android_intr_1 = 0; 

  ICM_20948_link_serif( &myICM, &mySerif);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

  ssd1306_Init();

  ssd1306_SetContrast(0);

  HAL_Delay(500);

  ssd1306_Fill(Black);
  ssd1306_SetCursor(20,7);
  ssd1306_WriteString("Swansong", Font_11x18, White);
  ssd1306_UpdateScreen();

  for(int c = 0; c < 255; c++) {
    ssd1306_SetContrast(c);
    HAL_Delay(10);
  }

  HAL_Delay(500);

  ssd1306_Fill(Black);
  ssd1306_SetCursor(0,0);
  ssd1306_WriteString("Configure SD Card", Font_6x8, White);
  ssd1306_UpdateScreen();

  // Make sure FatFS struct is clean
  memset(&FatFS, 0, sizeof(FATFS));

  if(f_mount(&FatFS, "", 1) != FR_OK) {
    f_mount(NULL, "", 0);
    ssd1306_SetCursor(0,8);
    ssd1306_WriteString("Mount Failed", Font_6x8, White);
    ssd1306_UpdateScreen();
    while(1) {HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE);}
  } 
  ssd1306_SetCursor(0,8);
  ssd1306_WriteString("Filesystem Mounted", Font_6x8, White);
  ssd1306_UpdateScreen();

  //Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;

  FATFS* getFreeFs;
  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK) { while(1){} }

  //Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  char sdstr[16];
  sprintf(sdstr, "%lu/%luMiB Used", (total_sectors-free_sectors) / 2000, total_sectors / 2000);
  ssd1306_SetCursor(0,16);
  ssd1306_WriteString(sdstr, Font_6x8, White);
  ssd1306_UpdateScreen();

  HAL_Delay(1000);
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0,0);
  ssd1306_WriteString("Configure ICM20948", Font_6x8, White);
  ssd1306_UpdateScreen();

  myprintf("\r\nICM 20948 Setup\r\n");
  if(ICM_20948_check_id( &myICM ) == ICM_20948_Stat_Ok) {
    myprintf("- Read ID - PASS\r\n");
  } else {
    myprintf("- Read ID - FAIL (Remove Board Power)\r\n"); // TODO: Try a software reset?
    while(1) {};
  }

  if(ICM_20948_sw_reset( &myICM) == ICM_20948_Stat_Ok) {
    myprintf("- SW Reset - PASS\r\n");
  } else {
    myprintf("- SW Reset - FAIL\r\n"); // TODO: Try a software reset?
    while(1) {};
  }
  HAL_Delay(500);

  if(ICM_20948_sleep( &myICM, false ) == ICM_20948_Stat_Ok) {
    myprintf("- Wake from Sleep - PASS\r\n");
  } else {
    myprintf("- Wake from Sleep - FAIL\r\n"); // TODO: Try a software reset?
    while(1) {};
  }

  if(ICM_20948_low_power( &myICM, false ) == ICM_20948_Stat_Ok) {
    myprintf("- Full power mode - PASS\r\n");
  } else {
    myprintf("- Full power mode - FAIL\r\n"); // TODO: Try a software reset?
    while(1) {};
  }

  if(ICM_20948_i2c_master_passthrough( &myICM, false ) == ICM_20948_Stat_Ok) {
    myprintf("- Disable I2C Passthrough - PASS\r\n");
  } else {
    myprintf("- Disable I2C Passthrough - FAIL\r\n");
    while(1) {};
  }

  if(ICM_20948_i2c_master_enable( &myICM, true ) == ICM_20948_Stat_Ok) {
    myprintf("- Enable I2C Master - PASS\r\n");
  } else {
    myprintf("- Enable I2C Master - FAIL\r\n");
    while(1) {};
  }

  ssd1306_SetCursor(0,8);
  ssd1306_WriteString("Configure AK09916", Font_6x8, White);
  ssd1306_UpdateScreen();

  uint8_t AK09916_whoiam = 0;
  int ak_tries = 0;
  while(AK09916_whoiam != (MAG_AK09916_WHO_AM_I & 0xFF)) {

    if(ICM_20948_i2c_master_reset( &myICM ) == ICM_20948_Stat_Ok) {
      myprintf("- I2C Master Reset - PASS\r\n");
    } else {
      myprintf("- I2C Master Reset - FAIL\r\n");
      while(1) {};
    }

    HAL_Delay(500);
 
    ICM_20948_i2c_master_single_r( &myICM, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA2, &AK09916_whoiam);
    myprintf("- AK09916 WHOIAM - 0x%02x - 0x%02x\r\n", AK09916_whoiam, MAG_AK09916_WHO_AM_I & 0xFF);

    char trystr[10];
    sprintf(trystr, "Tries: %d", ak_tries);
    ssd1306_SetCursor(0,16);
    ssd1306_WriteString(trystr, Font_6x8, White);
    ssd1306_UpdateScreen();

    ak_tries++;
    if(ak_tries > 5) {
      ssd1306_Fill(Black);
      ssd1306_SetCursor(0,16);
      ssd1306_WriteString("Cannot Find AK09916", Font_6x8, White);
      ssd1306_UpdateScreen();
      while(1) {HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE);}
    }

  }

  AK09916_CNTL2_Reg_t regctrl2;
  regctrl2.MODE = AK09916_mode_cont_100hz;
  if(ICM_20948_i2c_master_single_w( &myICM, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, (uint8_t)&regctrl2) == ICM_20948_Stat_Ok) {
    myprintf("- AK09916 Configure Control 2 - PASS\r\n");
  } else {
    myprintf("- AK09916 Configure Control 2 - FAIL\r\n");
    while(1) {};
  }

  if(ICM_20948_i2c_controller_configure_peripheral( &myICM, 0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, true, true, false, false, false) == ICM_20948_Stat_Ok) {
    myprintf("- AK09916 Configure Status 1- PASS\r\n");
  } else {
    myprintf("- AK09916 Configure Status 1 - FAIL\r\n");
    while(1) {};
  }

  ssd1306_SetCursor(0,16);
  ssd1306_WriteString("Configured AK09916", Font_6x8, White);
  ssd1306_UpdateScreen();
  HAL_Delay(500);

  ssd1306_Fill(Black);
  ssd1306_SetCursor(0,0);
  ssd1306_WriteString("Configure ICM20948", Font_6x8, White);
  ssd1306_UpdateScreen();
  HAL_Delay(500);

  if( ICM_20948_set_clock_source( &myICM, ICM_20948_Clock_Auto ) == ICM_20948_Stat_Ok ) {
    myprintf("- Set Clock - PASS\r\n");
  } else {
    myprintf("- Set Clock - FAIL\r\n");
    while(1) {};
  }

  if( ICM_20948_set_bank( &myICM, 0 ) == ICM_20948_Stat_Ok ) {
    myprintf("- Set Bank 0 - PASS\r\n");
  } else {
    myprintf("- Set Bank 0 - FAIL\r\n");
    while(1) {};
  }

  uint8_t pwr_mgmt2 = 0x40;
  if ( ICM_20948_execute_w( &myICM, AGB0_REG_PWR_MGMT_2, &pwr_mgmt2, 1 ) == ICM_20948_Stat_Ok ) {
    myprintf("- Write Power Management 2 - PASS\r\n");
  } else {
    myprintf("- Write Power Management 2 - FAIL\r\n");
    while(1) {};
  }

  if ( ICM_20948_set_sample_mode( &myICM, (ICM_20948_Internal_Mst | ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Cycled) == ICM_20948_Stat_Ok ) {
    myprintf("- Set Sample Mode A G M - PASS\r\n");
  } else {
    myprintf("- Set Sample Mode A G M - FAIL\r\n");
    while(1) {};
  }

  if ( ICM_20948_enable_FIFO( &myICM, false) == ICM_20948_Stat_Ok ) {
    myprintf("- Disable FIFO - PASS\r\n");
  } else {
    myprintf("- Disable FIFO - FAIL\r\n");
    while(1) {};
  }

  if ( ICM_20948_enable_DMP( &myICM, false) == ICM_20948_Stat_Ok ) {
    myprintf("- Disable DMP - PASS\r\n");
  } else {
    myprintf("- Disable DMP - FAIL\r\n");
    while(1) {};
  }
  
  ICM_20948_fss_t myFSS;
  myFSS.a = gpm4;
  myFSS.g = dps2000;

  if ( ICM_20948_set_full_scale( &myICM, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS) == ICM_20948_Stat_Ok )  {
    myprintf("- Set Full Scale A G - PASS\r\n");
  } else {
    myprintf("- Set Full Scale A G - FAIL\r\n");
    while(1) {};
  }
  
  if( ICM_20948_set_bank( &myICM, 0 ) == ICM_20948_Stat_Ok ) {
    myprintf("- Set Bank 0 - PASS\r\n");
  } else {
    myprintf("- Set Bank 0 - FAIL\r\n");
    while(1) {};
  }

  uint8_t zero = 0;
  if ( ICM_20948_execute_w( &myICM, AGB0_REG_FIFO_EN_1, &zero, 1) == ICM_20948_Stat_Ok) {
    myprintf("- Set FIFO Enable 1 - PASS\r\n");
  } else {
    myprintf("- Set FIFO Enable 1 - FAIL\r\n");
    while(1) {};
  }

  if ( ICM_20948_execute_w( &myICM, AGB0_REG_FIFO_EN_2, &zero, 1) == ICM_20948_Stat_Ok) {
    myprintf("- Set FIFO Enable 2 - PASS\r\n");
  } else {
    myprintf("- Set FIFO Enable 2 - FAIL\r\n");
    while(1) {};
  }

  ssd1306_SetCursor(0,8);
  ssd1306_WriteString("Configure DMP", Font_6x8, White);
  ssd1306_UpdateScreen();
  HAL_Delay(500);

  // Disable data ready interrupt
  ICM_20948_INT_enable_t en;    
  if ( ICM_20948_int_enable( &myICM, NULL, &en ) == ICM_20948_Stat_Ok ) {
    myprintf("- Read Data Ready Interrupt Enable - PASS\r\n");
  } else {
    myprintf("- Read Data Ready Interrupt Enable - FAIL\r\n");
    while(1) {};
  }
  en.RAW_DATA_0_RDY_EN = false;                  
  if ( ICM_20948_int_enable( &myICM, &en, &en ) == ICM_20948_Stat_Ok ) {
    myprintf("- Set Data Ready Interrupt Disable - PASS\r\n");
  } else {
    myprintf("- Set Data Ready Interrupt Disable - FAIL\r\n");
    while(1) {};
  }

  if ( ICM_20948_reset_FIFO( &myICM ) == ICM_20948_Stat_Ok ) {
    myprintf("- Reset FIFO - PASS\r\n");
  } else {
    myprintf("- Reset FIFO - FAIL\r\n");
    while(1) {};
  }

  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  if ( ICM_20948_set_sample_rate( &myICM, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt ) == ICM_20948_Stat_Ok ) { // ** Note: comment this line to leave the sample rates at the maximum **
    myprintf("- Set Sample Rate A G - PASS\r\n");
  } else {
    myprintf("- Set Sample Rate A G - FAIL\r\n");
    while(1) {};
  }

  // Load DMP Firmware
  if ( ICM_20948_set_dmp_start_address( &myICM, DMP_START_ADDRESS) == ICM_20948_Stat_Ok ) {
    myprintf("- Set DMP Start Address - PASS\r\n");
  } else {
    myprintf("- Set DMP Start Address - FAIL\r\n");
    while(1) {};
  }

  if ( ICM_20948_firmware_load( &myICM ) == ICM_20948_Stat_Ok ) {
    myprintf("- Load DMP Firmware - PASS\r\n");
  } else {
    myprintf("- Load DMP Firmware - FAIL\r\n");
    while(1) {};
  }

  if ( ICM_20948_set_dmp_start_address( &myICM, DMP_START_ADDRESS) == ICM_20948_Stat_Ok ) {
    myprintf("- Set DMP Start Address Bytes - PASS\r\n");
  } else {
    myprintf("- Set DMP Start Address Bytes - FAIL\r\n");
    while(1) {};
  }

  ssd1306_SetCursor(0,16);
  ssd1306_WriteString("Bitstream Load", Font_6x8, White);
  ssd1306_UpdateScreen();
  HAL_Delay(500);

  if ( ICM_20948_set_bank( &myICM, 0) == ICM_20948_Stat_Ok ) {
    myprintf("- Set Bank 0 - PASS\r\n");
  } else {
    myprintf("- Set Bank 0 - FAIL\r\n");
    while(1) {};
  }

  uint8_t fix = 0x48;
  if ( ICM_20948_execute_w( &myICM, AGB0_REG_HW_FIX_DISABLE, &fix, 1) == ICM_20948_Stat_Ok) {
    myprintf("- Set HW Fix - PASS\r\n");
  } else {
    myprintf("- Set HW Fix - FAIL\r\n");
    while(1) {};
  }
  
  if ( ICM_20948_set_bank( &myICM, 0) == ICM_20948_Stat_Ok ) {
    myprintf("- Set Bank 0 - PASS\r\n");
  } else {
    myprintf("- Set Bank 0 - FAIL\r\n");
    while(1) {};
  }

  uint8_t fifoPrio = 0xE4;
  if ( ICM_20948_execute_w( &myICM, AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1) == ICM_20948_Stat_Ok) {
    myprintf("- FIFO Priority Select - PASS\r\n");
  } else {
    myprintf("- FIFO Priority Select - FAIL\r\n");
    while(1) {};
  }

  const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
  if ( inv_icm20948_write_mems( &myICM, ACC_SCALE, 4, accScale) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Accelerometer Scale 1 Select - PASS\r\n");
  } else {
    myprintf("- DMP Accelerometer Scale 1 Select - FAIL\r\n");
    while(1) {};
  }
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  if ( inv_icm20948_write_mems( &myICM, ACC_SCALE2, 4, accScale2) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Accelerometer Scale 2 Select - PASS\r\n");
  } else {
    myprintf("- DMP Accelerometer Scale 2 Select - FAIL\r\n");
    while(1) {};
  }

  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99}; // Value taken from InvenSense Nucleo example
  const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
  uint8_t cpass_mtx_stat = 1;
  cpass_mtx_stat &= ( inv_icm20948_write_mems( &myICM, CPASS_MTX_00, 4, &mountMultiplierPlus[0]) == ICM_20948_Stat_Ok);
  cpass_mtx_stat &= ( inv_icm20948_write_mems( &myICM, CPASS_MTX_01, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  cpass_mtx_stat &= ( inv_icm20948_write_mems( &myICM, CPASS_MTX_02, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  cpass_mtx_stat &= ( inv_icm20948_write_mems( &myICM, CPASS_MTX_10, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  cpass_mtx_stat &= ( inv_icm20948_write_mems( &myICM, CPASS_MTX_11, 4, &mountMultiplierMinus[0]) == ICM_20948_Stat_Ok);
  cpass_mtx_stat &= ( inv_icm20948_write_mems( &myICM, CPASS_MTX_12, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  cpass_mtx_stat &= ( inv_icm20948_write_mems( &myICM, CPASS_MTX_20, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  cpass_mtx_stat &= ( inv_icm20948_write_mems( &myICM, CPASS_MTX_21, 4, &mountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  cpass_mtx_stat &= ( inv_icm20948_write_mems( &myICM, CPASS_MTX_22, 4, &mountMultiplierMinus[0]) == ICM_20948_Stat_Ok);

  if(cpass_mtx_stat) {
    myprintf("- DMP Compass Mount Matrix Set - PASS\r\n");
  } else {
    myprintf("- DMP Compass Mount Matrix Set - Fail\r\n");
    while(1) {};
  }

  const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  uint8_t b2s_mtx_stat = 1;
  b2s_mtx_stat &= ( inv_icm20948_write_mems( &myICM, B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]) == ICM_20948_Stat_Ok);
  b2s_mtx_stat &= ( inv_icm20948_write_mems( &myICM, B2S_MTX_01, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  b2s_mtx_stat &= ( inv_icm20948_write_mems( &myICM, B2S_MTX_02, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  b2s_mtx_stat &= ( inv_icm20948_write_mems( &myICM, B2S_MTX_10, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  b2s_mtx_stat &= ( inv_icm20948_write_mems( &myICM, B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]) == ICM_20948_Stat_Ok);
  b2s_mtx_stat &= ( inv_icm20948_write_mems( &myICM, B2S_MTX_12, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  b2s_mtx_stat &= ( inv_icm20948_write_mems( &myICM, B2S_MTX_20, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  b2s_mtx_stat &= ( inv_icm20948_write_mems( &myICM, B2S_MTX_21, 4, &b2sMountMultiplierZero[0]) == ICM_20948_Stat_Ok);
  b2s_mtx_stat &= ( inv_icm20948_write_mems( &myICM, B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]) == ICM_20948_Stat_Ok);

  if(b2s_mtx_stat) {
    myprintf("- B2S Mount Matrix Set - PASS\r\n");
  } else {
    myprintf("- B2S Mount Matrix Set - Fail\r\n");
    while(1) {};
  }

  if( inv_icm20948_set_gyro_sf( &myICM, 19, 3) == ICM_20948_Stat_Ok) { // 19 = 55Hz (see above), 3 = 2000dps (see above)
    myprintf("- DMP Gyroscope Sample Rate Select - PASS\r\n");
  } else {
    myprintf("- DMP Gyroscope Sample Rate Select - Fail\r\n");
    while(1) {};
  }

  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  if ( inv_icm20948_write_mems( &myICM, GYRO_FULLSCALE, 4, gyroFullScale) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Gyroscope Full Scale Select - PASS\r\n");
  } else {
    myprintf("- DMP Gyroscope Full Scale Select - Fail\r\n");
    while(1) {};
  }
  
  const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  if ( inv_icm20948_write_mems( &myICM, ACCEL_ONLY_GAIN, 4, accelOnlyGain) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Accelerometer Gain Select - PASS\r\n");
  } else {
    myprintf("- DMP Accelerometer Gain Select - Fail\r\n");
    while(1) {};
  }
  
  const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  if ( inv_icm20948_write_mems( &myICM, ACCEL_ALPHA_VAR, 4, accelAlphaVar) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Accelerometer Alpha Select - PASS\r\n");
  } else {
    myprintf("- DMP Accelerometer Alpha Select - Fail\r\n");
    while(1) {};
  }
  
  const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  if ( inv_icm20948_write_mems( &myICM, ACCEL_A_VAR, 4, accelAVar) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Accelerometer A Var Select - PASS\r\n");
  } else {
    myprintf("- DMP Accelerometer A Var Select - Fail\r\n");
    while(1) {};
  }
  
  const unsigned char accelCalRate[4] = {0x00, 0x00};
  if ( inv_icm20948_write_mems( &myICM, ACCEL_CAL_RATE, 2, accelCalRate) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Accelerometer Calibration Rate Select - PASS\r\n");
  } else {
    myprintf("- DMP Accelerometer Calibration Rate Select - Fail\r\n");
    while(1) {};
  }
  
  const unsigned char compassRate[2] = {0x00, 0x64}; // 100Hz
  if ( inv_icm20948_write_mems( &myICM, CPASS_TIME_BUFFER, 2, compassRate) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Compass Sample Rate Select - PASS\r\n");
  } else {
    myprintf("- DMP Compass Sample Rate Select - Fail\r\n");
    while(1) {};
  }

  //if ( inv_icm20948_enable_dmp_sensor( &myICM, INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR | INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR | INV_ICM20948_SENSOR_ACCELEROMETER | INV_ICM20948_SENSOR_GYROSCOPE, 1) == ICM_20948_Stat_Ok) {
  if ( inv_icm20948_enable_dmp_sensor( &myICM, INV_ICM20948_SENSOR_ACCELEROMETER, 1) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Enable Accelerometer - PASS\r\n");
  } else {
    myprintf("- DMP Enable Accelerometer - Fail\r\n");
    while(1) {};
  }
  if ( inv_icm20948_enable_dmp_sensor( &myICM, INV_ICM20948_SENSOR_ROTATION_VECTOR, 1) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Enable Rotation Vector - PASS\r\n");
  } else {
    myprintf("- DMP Enable Rotation Vector - Fail\r\n");
    while(1) {};
  }
  /*if ( inv_icm20948_enable_dmp_sensor( &myICM, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 1) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Enable GRV - PASS\r\n");
  } else {
    myprintf("- DMP Enable GRV - Fail\r\n");
    while(1) {};
  }/*
  /*if ( inv_icm20948_enable_dmp_sensor( &myICM, INV_ICM20948_SENSOR_GYROSCOPE, 1) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Enable Gyroscope - PASS\r\n");
  } else {
    myprintf("- DMP Enable Gyroscope - Fail\r\n");
    while(1) {};
  }*/
  /*if ( inv_icm20948_enable_dmp_sensor( &myICM, INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD, 1) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Enable Geomagnetic Vector - PASS\r\n");
  } else {
    myprintf("- DMP Enable Geomagnetic Vector - Fail\r\n");
    while(1) {};
  }*/
  if ( inv_icm20948_set_dmp_sensor_period( &myICM, DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Set Sensor Period - PASS\r\n");
  } else {
    myprintf("- DMP Set Sensor Period - Fail\r\n");
    while(1) {};
  }
  if ( inv_icm20948_set_dmp_sensor_period( &myICM, DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Set Sensor Period - PASS\r\n");
  } else {
    myprintf("- DMP Set Sensor Period - Fail\r\n");
    while(1) {};
  }
  /*if ( inv_icm20948_set_dmp_sensor_period( &myICM, DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Set Sensor Period - PASS\r\n");
  } else {
    myprintf("- DMP Set Sensor Period - Fail\r\n");
    while(1) {};
  }
  if ( inv_icm20948_set_dmp_sensor_period( &myICM, DMP_ODR_Reg_Cpass_Calibr, 2) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Set Sensor Period - PASS\r\n");
  } else {
    myprintf("- DMP Set Sensor Period - Fail\r\n");
    while(1) {};
  }*/

  if ( ICM_20948_enable_FIFO( &myICM, 1) == ICM_20948_Stat_Ok ) {
    myprintf("- Enable FIFO - PASS\r\n");
  } else {
    myprintf("- Enable FIFO - Fail\r\n");
    while(1) {};
  }
  if ( ICM_20948_enable_DMP( &myICM, 1) == ICM_20948_Stat_Ok ) {
    myprintf("- Enable DMP - PASS\r\n");
  } else {
    myprintf("- Enable DMP - Fail\r\n");
    while(1) {};
  }
  
  ssd1306_SetCursor(0,16);
  ssd1306_WriteString("Configured DMP", Font_6x8, White);
  ssd1306_UpdateScreen();
  HAL_Delay(500);

  if ( ICM_20948_reset_DMP( &myICM ) == ICM_20948_Stat_Ok ) {
    myprintf("- Reset DMP - PASS\r\n");
  } else {
    myprintf("- Reset DMP - Fail\r\n");
    while(1) {};
  }
  if ( ICM_20948_reset_FIFO( &myICM ) == ICM_20948_Stat_Ok ) {
    myprintf("- Reset FIFO - PASS\r\n");
  } else {
    myprintf("- Reset FIFO - Fail\r\n");
    while(1) {};
  }

  ssd1306_Fill(Black);
  ssd1306_SetCursor(0,0);
  ssd1306_WriteString("Configure ICM20948", Font_6x8, White);
  ssd1306_SetCursor(0,8);
  ssd1306_WriteString("Configured ICM20948", Font_6x8, White);
  ssd1306_UpdateScreen();
  HAL_Delay(500);

  myprintf("\r\nICM 20948 DMP Enable - PASS\r\n");
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

  ssd1306_Fill(Black);
  ssd1306_SetCursor(0,0);
  ssd1306_WriteString("Configure MS5637", Font_6x8, White);
  ssd1306_UpdateScreen();
  HAL_Delay(100);

  uint8_t i2c_buf[4];

  if(ms5637_is_connected() == false) {
    myprintf("No response from ms5637");
    ssd1306_SetCursor(0,8);
    ssd1306_WriteString("Cannot find MS5637", Font_6x8, White);
    ssd1306_UpdateScreen();
    HAL_Delay(250);
    while(1) {HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE);}
  }
  ms5637_reset();
  float temperature = 0;
  float pressure = 0;
  float altitude = 0;

  ssd1306_SetCursor(0,8);
  ssd1306_WriteString("Configured MS5637", Font_6x8, White);
  ssd1306_UpdateScreen();

  HAL_Delay(1000);

  float avg_alt_sum = 0;
  uint32_t avg_batv_sum = 0;
  int avg_count = 0;
  float avg_alt = 0;
  uint32_t avg_batv = 0;

  char buf[32] = {0};
  uint8_t rx;

  ssd1306_Fill(Black);
  ssd1306_SetCursor(0,0);
  ssd1306_WriteString("Wait for GPS Fix", Font_6x8, White);
  ssd1306_UpdateScreen();
  HAL_Delay(100);

  __HAL_UART_ENABLE(&huart1);

  HAL_OPAMP_Start(&hopamp4);
  //HAL_OPAMP_Start(&hopamp2);

  lwgps_init(&hgps);
  lwrb_init(&hgps_buff, hgps_buff_data, 512);

  // Wait for a GPS fix to get UTC
  while(hgps.fix == false) {
    if (lwrb_get_full(&hgps_buff)) {        /* Check if anything in buffer now */
        while (lwrb_read(&hgps_buff, &rx, 1) == 1) {
            lwgps_process(&hgps, &rx, 1); 
        }
    }
    sprintf(buf, "%2d/%2d", hgps.sats_in_use, hgps.sats_in_view);
    ssd1306_SetCursor(0,8);
    ssd1306_WriteString(buf, Font_6x8, White);
    ssd1306_UpdateScreen();
    HAL_Delay(250);
  }

  ssd1306_SetCursor(0,16);
  ssd1306_WriteString("Got GPS Fix", Font_6x8, White);
  ssd1306_UpdateScreen();
  HAL_Delay(250);

  sprintf(buf, "log%u%u.csv", (hgps.hours+1)%24, hgps.minutes);

  // Create file for logging
  fres = f_open(&fil, buf, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
  if(fres != FR_OK) {
    ssd1306_SetCursor(0,24);
    ssd1306_WriteString(buf, Font_6x8, White);
    ssd1306_WriteString(" FAILED", Font_6x8, White);
    ssd1306_UpdateScreen();
    while(1) {};
  }

  ssd1306_SetCursor(0,24);
  ssd1306_WriteString(buf, Font_6x8, White);
  ssd1306_UpdateScreen();
  HAL_Delay(1000);

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  char logline[1024];

  uint16_t batv, bati;

  const float reftemp = 9.5f;
  const float refpres = 1029.3f;
  const float atmo_const = -29.27112;

  uint32_t rot_q1, rot_q2, rot_q3, rot_acc;
  uint32_t grv_q1, grv_q2, grv_q3;
  uint32_t acc_x, acc_y, acc_z;
  uint32_t gyr_x, gyr_y, gyr_z;
  uint32_t mag_x, mag_y, mag_z;

  rot_q1 = 0;
  rot_q2 = 0;
  rot_q3 = 0;
  rot_acc = 0;
  acc_x = 0;
  acc_y = 0;
  acc_z = 0;

  ICM_20948_Status_e read_stat;
  icm_20948_DMP_data_t data;

  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_RESET)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if(update_sensors) {
      HAL_ADC_Start(&hadc5);
      //HAL_ADC_Start(&hadc2); // Screwed up battery current sense hardware

      ms5637_read_temperature_and_pressure(&temperature, &pressure);
      HAL_ADC_PollForConversion(&hadc5, 1);
      batv = HAL_ADC_GetValue(&hadc5);
      
      // Screwed up battery current sense hardware
      // Only good for charging current not discharge :(
      // HAL_ADC_PollForConversion(&hadc2, 1);
      // bati = HAL_ADC_GetValue(&hadc2);

      altitude = (log(pressure)-log(refpres))*atmo_const*(reftemp+273.15);
      update_sensors = 0;
    }

    if (lwrb_get_full(&hgps_buff)) {        /* Check if anything in buffer now */
        while (lwrb_read(&hgps_buff, &rx, 1) == 1) {
            lwgps_process(&hgps, &rx, 1); 
        }
    }

    if(update_screen) {
      update_screen = 0;
      ssd1306_Fill(Black);
      sprintf(buf, "%2.1fC %4.0fhPa %3.1fm", temperature, pressure, altitude );
      ssd1306_SetCursor(0,0);
      ssd1306_WriteString(buf, Font_6x8, White);
      sprintf(buf, "%1.2fV %2d/%2d %3.1fm", (float)((2.5/4096)*batv)*2.0, hgps.sats_in_use, hgps.sats_in_view, hgps.altitude);
      ssd1306_SetCursor(0,8);
      ssd1306_WriteString(buf, Font_6x8, White);
      sprintf(buf, "%.7f %.7f", hgps.latitude, hgps.longitude);
      ssd1306_SetCursor(0,16);
      ssd1306_WriteString(buf, Font_6x8, White);
      sprintf(buf, "%3.1f %3.0f %02d:%02d:%02d", hgps.speed*1.852, hgps.course, (hgps.hours+1)%24, hgps.minutes, hgps.seconds);
      ssd1306_SetCursor(0,24);
      ssd1306_WriteString(buf, Font_6x8, White);
      ssd1306_UpdateScreen();
    }

    

    //float qwqwMinusHalf = q0 * q0 - 0.5f; // calculate common terms to avoid repeated operations
    //float roll = atan2f(q2 * q3 - q0 * q1, qwqwMinusHalf + q3 * q3);
    //float pitch = -1.0f * asinf(2.0f * (q1 * q3 + q0 * q2));
    //float yaw = atan2f(q1 * q2 - q0 * q3, qwqwMinusHalf + q1 * q1);
    
    do {
    read_stat = inv_icm20948_read_dmp_data( &myICM, &data );  
    if(read_stat == ICM_20948_Stat_Ok || read_stat == ICM_20948_Stat_FIFOMoreDataAvail ) {
      if( (data.header & DMP_header_bitmap_Quat9) > 0 ) { 
        rot_q1 = (data.Quat9.Data.Q1);
        rot_q2 = (data.Quat9.Data.Q2);
        rot_q3 = (data.Quat9.Data.Q3);

        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);
      }
      /*if( (data.header & DMP_header_bitmap_Quat6) > 0 ) { 
        grv_q1 = (data.Quat6.Data.Q1);
        grv_q2 = (data.Quat6.Data.Q2);
        grv_q3 = (data.Quat6.Data.Q3);

        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);
        //myprintf("Q6");
      }*/
      if( (data.header & DMP_header_bitmap_Accel) > 0 ) { 
        acc_x = data.Raw_Accel.Data.X;
        acc_y = data.Raw_Accel.Data.Y;
        acc_z = data.Raw_Accel.Data.Z;
        
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);
        //myprintf("A");
      }
      /*if( (data.header & DMP_header_bitmap_Gyro_Calibr) > 0) {
        gyr_x = data.Gyro_Calibr.Data.X;
        gyr_y = data.Gyro_Calibr.Data.Y;
        gyr_z = data.Gyro_Calibr.Data.Z;
        
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);
        //myprintf("G");
      }
      if( (data.header & DMP_header_bitmap_Compass_Calibr) > 0) {
        mag_x = data.Compass_Calibr.Data.X;
        mag_y = data.Compass_Calibr.Data.Y;
        mag_z = data.Compass_Calibr.Data.Z;

        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);
        //myprintf("M");
      }*/
    }
    }while(read_stat == ICM_20948_Stat_FIFOMoreDataAvail);

    //myprintf("\r\n");

    if(update_log) {
      update_log = 0;
      sprintf(logline, "%u:%u:%u %d %d %.2f %.3f %.3f %.3f %.7f %.7f %.2f %.2f %u %u %.1f %.1f %.1f %lu %lu %lu %lu %lu %lu\r\n", hgps.hours, hgps.minutes, hgps.seconds, timebase, batv, temperature, pressure, altitude, hgps.altitude, hgps.latitude, hgps.longitude, hgps.speed, hgps.course, hgps.sats_in_use, hgps.sats_in_view, hgps.dop_h, hgps.dop_p, hgps.dop_v, rot_q1, rot_q2, rot_q3, acc_x, acc_y, acc_z);
      UINT bytesWrote;
      fres = f_write(&fil, logline, strlen(logline), &bytesWrote);
    }
    //HAL_Delay(500);
  }

  f_close(&fil);
  f_mount(NULL, "", 0);
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0,0);
  ssd1306_WriteString("Saved Log File", Font_6x8, White);
  ssd1306_SetCursor(0,8);
  ssd1306_WriteString("Shutting Down", Font_6x8, White);
  ssd1306_UpdateScreen();
  
  for(int c = 255; c > 0; c--) {
    ssd1306_SetContrast(c);
    HAL_Delay(10);
  }

  ssd1306_SetDisplayOn(false);

  HAL_GPIO_WritePin(IMU_SPI_LS_EN_GPIO_Port, IMU_SPI_LS_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NAV_STAT_0_GPIO_Port, NAV_STAT_0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NAV_STAT_1_GPIO_Port, NAV_STAT_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPS_FORCEON_GPIO_Port, GPS_FORCEON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPS_EXT_INT_GPIO_Port, GPS_EXT_INT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPS_RESET_GPIO_Port, GPS_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPS_RESET_GPIO_Port, GPS_RESET_Pin, GPIO_PIN_SET);

  while(1) {HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE);}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_ADC345
                              |RCC_PERIPHCLK_QSPI;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
  PeriphClkInit.QspiClockSelection = RCC_QSPICLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#define LINEMAX 512
void UART_GPS_Callback() {

  static char rx_buffer[LINEMAX + 1]; // Local holding buffer to build line, w/NUL
  static int rx_index = 0;

  if (USART1->ISR & USART_ISR_RXNE) // Received character?
  {

  char rx = (char)(USART1->RDR & 0xFF);
  if ((rx == '\r') || (rx == '\n')) // Is this an end-of-line condition, either will suffice?
  {
  if (rx_index != 0) // Line has some content?
  {
  rx_buffer[rx_index++] = rx; // Add Final Character
  rx_buffer[rx_index++] = 0; // Add NUL if required down stream
  lwrb_write(&hgps_buff, &rx_buffer, rx_index); // Copy to queue from live dynamic receive buffer
  rx_index = 0; // Reset content pointer
  }
  }
  else
  {
  if ((rx == '$') || (rx_index == LINEMAX)) // If resync or overflows pull back to start
  rx_index = 0;
  rx_buffer[rx_index++] = rx; // Copy to buffer and increment
  }
  }
  //USART1->CR1 |= USART_CR1_RXNEIE; // Enable Interrupt
}

void timebase_callback() {
  if((timebase % 1000) == 0) {
    update_screen = 1;
  }
  if((timebase % 100)) {
    update_sensors = 1;
  }
  if((timebase % 20)) {
    update_log = 1;
  }
  timebase++;
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
