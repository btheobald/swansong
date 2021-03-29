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

  /*ssd1306_Init();
  ssd1306_Fill(White);
  ssd1306_WriteString("Swansong", Font_16x26, Black);
  ssd1306_UpdateScreen();*/

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
  HAL_Delay(100);

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

  if(ICM_20948_i2c_master_reset( &myICM ) == ICM_20948_Stat_Ok) {
    myprintf("- I2C Master Reset - PASS\r\n");
  } else {
    myprintf("- I2C Master Reset - FAIL\r\n");
    while(1) {};
  }

  HAL_Delay(100);

  if(ICM_20948_i2c_master_reset( &myICM ) == ICM_20948_Stat_Ok) {
    myprintf("- I2C Master Reset - PASS\r\n");
  } else {
    myprintf("- I2C Master Reset - FAIL\r\n");
    while(1) {};
  }

  HAL_Delay(100);

  if(ICM_20948_i2c_master_reset( &myICM ) == ICM_20948_Stat_Ok) {
    myprintf("- I2C Master Reset - PASS\r\n");
  } else {
    myprintf("- I2C Master Reset - FAIL\r\n");
    while(1) {};
  }

  HAL_Delay(100);

  uint8_t AK09916_whoiam = 0;
  ICM_20948_i2c_master_single_r( &myICM, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA2, &AK09916_whoiam);
  myprintf("- AK09916 WHOIAM - 0x%02x - 0x%02x\r\n", AK09916_whoiam, MAG_AK09916_WHO_AM_I & 0xFF);

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

  if ( inv_icm20948_enable_dmp_sensor( &myICM, INV_ICM20948_SENSOR_ORIENTATION, 1) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Enable Orientation - PASS\r\n");
  } else {
    myprintf("- DMP Enable Orientation - Fail\r\n");
    while(1) {};
  }
  if ( inv_icm20948_set_dmp_sensor_period( &myICM, DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok) {
    myprintf("- DMP Set Sensor Period - PASS\r\n");
  } else {
    myprintf("- DMP Set Sensor Period - Fail\r\n");
    while(1) {};
  }
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

  HAL_Delay(100);
  myprintf("\r\nICM 20948 DMP Enable - PASS\r\n");
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

  uint8_t i2c_buf[4];

  if(ms5637_is_connected() == false) {
    myprintf("No response from ms5637");
    while(1) {}
  }
  ms5637_reset();
  float temperature = 0;
  float pressure = 0;
  float altitude = 0;

  float avg_alt_sum = 0;
  uint32_t avg_batv_sum = 0;
  int avg_count = 0;
  float avg_alt = 0;
  uint32_t avg_batv = 0;

  ssd1306_Init();
  char buf[16] = {};

  __HAL_UART_ENABLE(&huart1);

  HAL_OPAMP_Start(&hopamp4);

  lwgps_init(&hgps);
  lwrb_init(&hgps_buff, hgps_buff_data, 512);
  
  char str[16];
  uint8_t rx;

  //USART1->CR1 |= USART_CR1_RXNEIE; // Enable Interrupt
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  while(1) {

    HAL_ADC_Start(&hadc5);

    if(ms5637_read_temperature_and_pressure(&temperature, &pressure) != ms5637_status_ok) {
      myprintf("Failed to get Temperature and Pressure\r\n");
      while(1) {}
    } else {
      //myprintf("%2.2fC, %4.2fhPa\r\n", temperature, pressure);
    }

    HAL_ADC_PollForConversion(&hadc5, 10);
    uint16_t batv = HAL_ADC_GetValue(&hadc5);

    const float reftemp = 10.0f;
    const float refpres = 1026.0f;
    const float atmo_const = -29.27112;

    altitude = (log(pressure)-log(refpres))*atmo_const*(temperature+273.15);
    avg_alt_sum += altitude;
    avg_batv_sum += batv;
    avg_count++;
    if(avg_count == 30) {
      avg_alt = avg_alt_sum/avg_count;
      avg_batv = avg_batv_sum/avg_count;
      avg_count = 0;
      avg_alt_sum = 0;
      avg_batv_sum = 0;
    }

    if (lwrb_get_full(&hgps_buff)) {        /* Check if anything in buffer now */
        while (lwrb_read(&hgps_buff, &rx, 1) == 1) {
            lwgps_process(&hgps, &rx, 1); 
        }
    }

    ssd1306_Fill(Black);
    sprintf(buf, "%2.1fC %4.0f %2.1fm", temperature, pressure, avg_alt );
    ssd1306_SetCursor(1,1);
    ssd1306_WriteString(buf, Font_7x10, White);
    sprintf(buf, "%2.1fV %d/%d %2.1fm", (float)((2.5/4096)*batv)*2.0, hgps.sats_in_use, hgps.sats_in_view, hgps.altitude );
    ssd1306_SetCursor(1,12);
    ssd1306_WriteString(buf, Font_7x10, White);
    sprintf(buf, "%.5f %.5f", hgps.latitude, hgps.longitude );
    ssd1306_SetCursor(1,22);
    ssd1306_WriteString(buf, Font_7x10, White);

    ssd1306_UpdateScreen();
    //HAL_Delay(5);
    HAL_GPIO_TogglePin(NAV_STAT_0_GPIO_Port, NAV_STAT_1_Pin);
  }

  float roll_cal = 0;
  float pitch_cal = 0;
  float yaw_cal = 0;

  uint8_t gps_fix_last = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    icm_20948_DMP_data_t data;
    ICM_20948_Status_e read_stat;
    read_stat = inv_icm20948_read_dmp_data( &myICM, &data );

    if(read_stat == ICM_20948_Stat_Ok || read_stat == ICM_20948_Stat_FIFOMoreDataAvail ) {
      if( (data.header & DMP_header_bitmap_Quat9) > 0 ) { // We have asked for orientation data so we should receive Quat9
        float q1 = ((float)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
        float q2 = ((float)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
        float q3 = ((float)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
        float q0 = sqrt( 1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

        //myprintf("{\"quat_w\":%.2f, \"quat_x\":%.2f, \"quat_y\":%.2f, \"quat_z\":%.2f}\r\n", q0, q1, q2, q3 );

        float qwqwMinusHalf = q0 * q0 - 0.5f; // calculate common terms to avoid repeated operations
        float roll = atan2f(q2 * q3 - q0 * q1, qwqwMinusHalf + q3 * q3);
        float pitch = -1.0f * asinf(2.0f * (q1 * q3 + q0 * q2));
        float yaw = atan2f(q1 * q2 - q0 * q3, qwqwMinusHalf + q1 * q1);

        if(HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == GPIO_PIN_SET) {
          pitch_cal = pitch;
          yaw_cal = yaw;
          roll_cal = roll;
        }

        //myprintf("%f; %f; %f\r\n", roll-roll_cal, pitch-pitch_cal, yaw-yaw_cal);

        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);

        if(read_stat != ICM_20948_Stat_FIFOMoreDataAvail) {
          HAL_Delay(10);
        }
      }
      if( (data.header & DMP_header_bitmap_Compass) > 0 ) { // We have asked for orientation data so we should receive Quat9
        uint32_t mx = data.Compass_Calibr.Data.X;
        uint32_t my = data.Compass_Calibr.Data.Y;
        uint32_t mz = data.Compass_Calibr.Data.Z;

        //myprintf("%d, %d, %d\r\n", mx, my, mz);

        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);

        if(read_stat != ICM_20948_Stat_FIFOMoreDataAvail) {
          HAL_Delay(10);
        }
      }
    }
    //HAL_Delay(500);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
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
  PeriphClkInit.QspiClockSelection = RCC_QSPICLKSOURCE_SYSCLK;

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
/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
