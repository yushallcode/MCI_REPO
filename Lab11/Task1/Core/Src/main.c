/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 11 Task 1 - Complementary Filter Angle Estimation
  *                   STM32F3Discovery: L3GD20 (SPI1) + LSM303DLHC (I2C1)
  *                   UART output via USART2 (PA2=TX, PA3=RX)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ---- LSM303DLHC Accelerometer (I2C1) ----
#define ACC_I2C_ADDR        0x32   // 7-bit 0x19 << 1
#define ACC_CTRL_REG1_A     0x20
#define ACC_CTRL_REG4_A     0x23
#define ACC_OUT_X_L_A       0x28

// ---- L3GD20 Gyroscope (SPI1) ----
#define GYRO_WHO_AM_I       0x0F
#define GYRO_CTRL_REG1      0x20
#define GYRO_CTRL_REG4      0x23
#define GYRO_OUT_X_L        0x28

// Sensitivities
#define GYRO_SENSITIVITY    8.75f    // mdps/digit at 250 dps => 0.00875 dps/digit
#define ACC_SENSITIVITY     1.0f     // mg/digit at ±2g

// Complementary filter
#define DT                  0.01f    // 10 ms loop delay
#define ALPHA               0.98f

// Radians to degrees
#define RAD2DEG             57.2957795f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */
void UART_Print(const char *msg);
void Gyro_Init(void);
void Accel_Init(void);
void Gyro_ReadXYZ(float *gx, float *gy, float *gz);
void Accel_ReadXYZ(float *ax, float *ay, float *az);
uint8_t Gyro_ReadReg(uint8_t reg);
void Gyro_WriteReg(uint8_t reg, uint8_t val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ==================== UART ====================
void UART_Print(const char *msg) {
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

// ==================== L3GD20 Gyro (SPI) ====================
void Gyro_WriteReg(uint8_t reg, uint8_t val) {
  uint8_t txData[2] = { reg, val };  // bit7=0 for write
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}

uint8_t Gyro_ReadReg(uint8_t reg) {
  uint8_t txData = reg | 0x80;  // bit7=1 for read
  uint8_t rxData = 0;
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, &rxData, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
  return rxData;
}

void Gyro_Init(void) {
  // Deselect gyro first
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  // CTRL_REG1: ODR=190Hz, BW=50Hz, enable X/Y/Z, power on
  // Bits: DR1=0 DR0=1 BW1=1 BW0=0 PD=1 Zen=1 Yen=1 Xen=1 = 0x6F
  Gyro_WriteReg(GYRO_CTRL_REG1, 0x6F);

  // CTRL_REG4: FS=250 dps (default), BDU=1
  Gyro_WriteReg(GYRO_CTRL_REG4, 0x80);
}

void Gyro_ReadXYZ(float *gx, float *gy, float *gz) {
  uint8_t txAddr = GYRO_OUT_X_L | 0x80 | 0x40;  // read + auto-increment
  uint8_t buf[6];

  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &txAddr, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, buf, 6, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);

  int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]);
  int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]);
  int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]);

  // Convert to degrees per second
  *gx = raw_x * GYRO_SENSITIVITY * 0.001f;
  *gy = raw_y * GYRO_SENSITIVITY * 0.001f;
  *gz = raw_z * GYRO_SENSITIVITY * 0.001f;
}

// ==================== LSM303DLHC Accel (I2C) ====================
void Accel_Init(void) {
  uint8_t data;

  // CTRL_REG1_A: ODR=100Hz, normal mode, X/Y/Z enabled
  // Bits: ODR=0101 LPen=0 Zen=1 Yen=1 Xen=1 = 0x57
  data = 0x57;
  HAL_I2C_Mem_Write(&hi2c1, ACC_I2C_ADDR, ACC_CTRL_REG1_A,
                    I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

  // CTRL_REG4_A: FS=±2g, HR=1 (high resolution), BDU=1
  data = 0x88;
  HAL_I2C_Mem_Write(&hi2c1, ACC_I2C_ADDR, ACC_CTRL_REG4_A,
                    I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

void Accel_ReadXYZ(float *ax, float *ay, float *az) {
  uint8_t buf[6];

  // Multi-byte read: set MSB of sub-address for auto-increment
  HAL_I2C_Mem_Read(&hi2c1, ACC_I2C_ADDR, ACC_OUT_X_L_A | 0x80,
                   I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);

  // LSM303DLHC in high-res mode: 12-bit data left-justified in 16 bits
  int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]) >> 4;
  int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]) >> 4;
  int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]) >> 4;

  // Convert to mg (±2g, 12-bit: 1 mg/LSB)
  *ax = raw_x * ACC_SENSITIVITY;
  *ay = raw_y * ACC_SENSITIVITY;
  *az = raw_z * ACC_SENSITIVITY;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(100);

  // Initialize sensors
  Gyro_Init();
  Accel_Init();

  HAL_Delay(100);

  // Verify gyro WHO_AM_I
  uint8_t who = Gyro_ReadReg(GYRO_WHO_AM_I);
  char msg[128];
  snprintf(msg, sizeof(msg), "Gyro WHO_AM_I: 0x%02X\r\n", who);
  UART_Print(msg);

  // Verify accel WHO_AM_I
  uint8_t acc_who = 0;
  HAL_I2C_Mem_Read(&hi2c1, ACC_I2C_ADDR, 0x0F,
                   I2C_MEMADD_SIZE_8BIT, &acc_who, 1, HAL_MAX_DELAY);
  snprintf(msg, sizeof(msg), "Accel WHO_AM_I: 0x%02X\r\n", acc_who);
  UART_Print(msg);

  HAL_Delay(500);

  float angle = 0.0f;
  float gx, gy, gz;
  float ax, ay, az;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(GPIOE, LD3_Pin); 
    HAL_Delay(500);  
    // Read sensors
    Gyro_ReadXYZ(&gx, &gy, &gz);
    Accel_ReadXYZ(&ax, &ay, &az);

    // Compute accelerometer tilt angle (degrees)
    // Using Y and Z axes for pitch on the F3 Discovery board
    // Adjust axes based on your board orientation on the robot
    float acc_angle = atan2f(ax, az) * RAD2DEG;

    // Complementary filter
    // gyro_y provides angular rate around the pitch axis
    // Adjust axis (gx, gy, gz) to match your mounting orientation
    angle = ALPHA * (angle + gy * DT) + (1.0f - ALPHA) * acc_angle;

    // Print: acc_angle, gyro_rate, filtered_angle (CSV for plotting)
    // NOTE: Add linker flag -u _printf_float if float printf doesn't work
    snprintf(msg, sizeof(msg), "%.2f,%.2f,%.2f\r\n", acc_angle, gy, angle);
    UART_Print(msg);

    HAL_Delay((uint32_t)(DT * 1000));  // 10 ms
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
