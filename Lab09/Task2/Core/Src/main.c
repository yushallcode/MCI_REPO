/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 9 - Accelerometer (I2C) & Gyroscope (SPI) Integration
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

// --- 1. LSM Data Structure (As requested by manual) ---
typedef struct {
    int16_t raw_acc_x, raw_acc_y, raw_acc_z;
    int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
    
    float acc_x, acc_y, acc_z;       // in mg
    float gyro_x, gyro_y, gyro_z;    // in dps
    
    float acc_offset_x, acc_offset_y, acc_offset_z;
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;
    
    float angle_x, angle_y;          // Calculated from Gyro
} LSM_Data_t;

LSM_Data_t lsm;

#define LOOP_TIME_SEC 0.1f // 100ms loop delay (dt)

void gyro_init(void);
void gyro_set_ctrl_reg4(void);
void reader(uint8_t reg_addr, uint8_t *rx_data);
int16_t read_16bit(uint8_t high_byte, uint8_t low_byte);

void Init_LSM(void);
void Offset_LSM(void);
void Read_LSM(void);
void Print_LSM(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ========================================================================= */
/* GYROSCOPE (SPI) FUNCTIONS (Your original names)                           */
/* ========================================================================= */
void reader(uint8_t reg_addr, uint8_t *rx_data) {
    uint8_t tx_buffer[2] = {reg_addr, 0x00}; 
    uint8_t rx_buffer[2] = {0x00, 0x00};     

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); 
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);   

    *rx_data = rx_buffer[1]; 
}

#define CTRL_REG1 0x20
#define CTRL_REG1_VAL 0b00001111 // PD = 1 (Power on), Xen/Yen/Zen enabled
void gyro_init(void) {
    uint8_t tx[2] = {CTRL_REG1, CTRL_REG1_VAL}; 
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

#define CTRL_REG4 0x23
#define CTRL_REG4_VAL 0b00000000 // FS [1:0] = 00 => +/- 250 dps
void gyro_set_ctrl_reg4(void) {
    uint8_t tx[2] = {CTRL_REG4, CTRL_REG4_VAL}; 
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

int16_t read_16bit(uint8_t high_byte, uint8_t low_byte) {
    return (int16_t)((high_byte << 8) | low_byte);
}

/* ========================================================================= */
/* ACCELEROMETER (I2C) & LAB 9 REQUIRED FUNCTIONS                            */
/* ========================================================================= */

void Init_LSM(void) {
    uint8_t ctrl_reg1 = 0x67; // 100Hz, Normal mode
    uint8_t ctrl_reg4 = 0x00; // Normal Mode

    HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x20, I2C_MEMADD_SIZE_8BIT, &ctrl_reg1, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x23, I2C_MEMADD_SIZE_8BIT, &ctrl_reg4, 1, HAL_MAX_DELAY);
}

void Offset_LSM(void) {
    float sum_acc_x = 0, sum_acc_y = 0;
    float sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;
    int num_samples = 20;
    
    uint8_t acc_data[6];
    uint8_t g_xH, g_xL, g_yH, g_yL, g_zH, g_zL;

    for (int i = 0; i < num_samples; i++) {
        // --- Read Accel ---
        HAL_I2C_Mem_Read(&hi2c1, 0x32, 0x28 | 0x80, I2C_MEMADD_SIZE_8BIT, acc_data, 6, HAL_MAX_DELAY);
        int16_t raw_a_x = (int16_t)((acc_data[1] << 8) | acc_data[0]) >> 6;
        int16_t raw_a_y = (int16_t)((acc_data[3] << 8) | acc_data[2]) >> 6;
        sum_acc_x += (raw_a_x * 3.9f);
        sum_acc_y += (raw_a_y * 3.9f);

        // --- Read Gyro ---
        reader(0x80 | 0x29, &g_xH); reader(0x80 | 0x28, &g_xL);
        reader(0x80 | 0x2B, &g_yH); reader(0x80 | 0x2A, &g_yL);
        reader(0x80 | 0x2D, &g_zH); reader(0x80 | 0x2C, &g_zL);
        sum_gyro_x += (read_16bit(g_xH, g_xL) * 0.00875f);
        sum_gyro_y += (read_16bit(g_yH, g_yL) * 0.00875f);
        sum_gyro_z += (read_16bit(g_zH, g_zL) * 0.00875f);

        HAL_Delay(20);
    }

    // Average Offsets
    lsm.acc_offset_x = sum_acc_x / num_samples;
    lsm.acc_offset_y = sum_acc_y / num_samples;
    lsm.acc_offset_z = 0; // Keep Earth's gravity intact

    lsm.gyro_offset_x = sum_gyro_x / num_samples;
    lsm.gyro_offset_y = sum_gyro_y / num_samples;
    lsm.gyro_offset_z = sum_gyro_z / num_samples;
}

void Read_LSM(void) {
    uint8_t acc_data[6];
    uint8_t g_xH, g_xL, g_yH, g_yL, g_zH, g_zL;

    // --- 1. Read & Scale Accelerometer ---
    HAL_I2C_Mem_Read(&hi2c1, 0x32, 0x28 | 0x80, I2C_MEMADD_SIZE_8BIT, acc_data, 6, HAL_MAX_DELAY);
    lsm.raw_acc_x = (int16_t)((acc_data[1] << 8) | acc_data[0]) >> 6;
    lsm.raw_acc_y = (int16_t)((acc_data[3] << 8) | acc_data[2]) >> 6;
    lsm.raw_acc_z = (int16_t)((acc_data[5] << 8) | acc_data[4]) >> 6;

    lsm.acc_x = (lsm.raw_acc_x * 3.9f) - lsm.acc_offset_x;
    lsm.acc_y = (lsm.raw_acc_y * 3.9f) - lsm.acc_offset_y;
    lsm.acc_z = (lsm.raw_acc_z * 3.9f) - lsm.acc_offset_z;

    // --- 2. Read & Scale Gyroscope ---
    reader(0x80 | 0x29, &g_xH); reader(0x80 | 0x28, &g_xL);
    reader(0x80 | 0x2B, &g_yH); reader(0x80 | 0x2A, &g_yL);
    reader(0x80 | 0x2D, &g_zH); reader(0x80 | 0x2C, &g_zL);
    
    lsm.raw_gyro_x = read_16bit(g_xH, g_xL);
    lsm.raw_gyro_y = read_16bit(g_yH, g_yL);
    lsm.raw_gyro_z = read_16bit(g_zH, g_zL);

    lsm.gyro_x = (lsm.raw_gyro_x * 0.00875f) - lsm.gyro_offset_x;
    lsm.gyro_y = (lsm.raw_gyro_y * 0.00875f) - lsm.gyro_offset_y;
    lsm.gyro_z = (lsm.raw_gyro_z * 0.00875f) - lsm.gyro_offset_z;

    // --- 3. Integrate Gyro for Angle ---
    // Angle = Previous_Angle + (Velocity * dt)
    lsm.angle_x += (lsm.gyro_x * LOOP_TIME_SEC);
    lsm.angle_y += (lsm.gyro_y * LOOP_TIME_SEC);
}

void Print_LSM(void) {
    char msg[150]; // Increased buffer size to hold the longer string

    // 1. Accelerometer (mg)
    int ax = (int)lsm.acc_x;
    int ay = (int)lsm.acc_y;
    int az = (int)lsm.acc_z;

    // 2. Gyroscope (dps)
    int gx = (int)lsm.gyro_x;
    int gy = (int)lsm.gyro_y;
    int gz = (int)lsm.gyro_z;

    // 3. Angles (degrees)
    int ang_x = (int)lsm.angle_x;
    int ang_y = (int)lsm.angle_y;

    // Format all values into a single string
    int len = sprintf(msg, "Acc:%d,%d,%d | Gyr:%d,%d,%d | Ang:%d,%d\r\n", 
                      ax, ay, az, gx, gy, gz, ang_x, ang_y);
                      
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, HAL_MAX_DELAY);
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  
  // 1. Initialize BOTH sensors
  Init_LSM();           // I2C Accel
  gyro_init();          // SPI Gyro
  gyro_set_ctrl_reg4(); // SPI Gyro Range
  HAL_Delay(100);

  // 2. Calibrate BOTH sensors (Keep the board completely flat and still!)
  char calib_msg[] = "Calibrating... DO NOT MOVE!\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)calib_msg, sizeof(calib_msg)-1, HAL_MAX_DELAY);
  
  lsm.angle_x = 0; // Reset angle
  lsm.angle_y = 0; 
  Offset_LSM();

  char start_msg[] = "Calibration Done! Starting Output:\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)start_msg, sizeof(start_msg)-1, HAL_MAX_DELAY);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */
    
    Read_LSM();
    Print_LSM();

    // Must match LOOP_TIME_SEC (0.1s = 100ms) for the math to be accurate!
    HAL_Delay(1000); 
    
    /* USER CODE END 3 */
  }
}

/* ------------------------------------------------------------------------- */
/* System Clock and Peripheral Initialization Functions Below                */
/* (Keep these identical to your previous code)                              */
/* ------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

static void MX_SPI1_Init(void) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  
  // FIX: Must be 8-BIT for the Gyroscope!
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT; 
  
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // Safer speed
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

static void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}