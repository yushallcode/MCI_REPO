/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 11 Task 2 - PID Controlled Self-Balancing Robot
  *                   STM32F3Discovery + Keyestudio KS0193 Balance Shield
  *                   - L3GD20 Gyro (SPI1), LSM303DLHC Accel (I2C1)
  *                   - TB6612FNG Motor Driver via PWM + GPIO
  *                   - Timer ISR for fixed-rate control loop (200 Hz)
  *                   - UART debug output via USART2 (PA2=TX)
  *                   - Bluetooth control via USART1 (PA9=TX, PA10=RX)
  *
  *   FIXES APPLIED (from previous revisions):
  *   1. 200-sample gyro calibration for stable offsets
  *   2. Accelerometer calibration (zero-angle correction)
  *   3. Complementary filter subtracts accel offsets before atan2
  *   4. Auto-measured resting angle as SETPOINT
  *   5. Startup grace period before safety cutoff activates
  *
  *   NEW: Bluetooth remote control
  *   - Funduino Bluetooth Bee (HC-05/06 class, XBee form factor) on USART1
  *   - 'F' / 'f' -> lean forward briefly  (rolls forward "a bit")
  *   - 'B' / 'b' -> lean backward briefly (rolls backward "a bit")
  *   - 'S' / 's' -> stop (recenter setpoint immediately)
  *   - Commands auto-expire after BT_COMMAND_MS to prevent runaway
  *     if the phone disconnects mid-command.
  *   - Implementation: nudges SETPOINT by +/-BT_LEAN_DEGREES instead of
  *     directly driving motors. The PID then "catches" the deliberate
  *     lean by accelerating the wheels in that direction.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
  float offset_x;
  float offset_y;
  float offset_z;
} Gyro_Data;

typedef struct {
  float offset_x;
  float offset_y;
  float offset_z;
} Accel_Data;

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

#define GYRO_CALIB_SAMPLES  200
#define ACC_CALIB_SAMPLES   200

// Sensitivities
#define GYRO_SENSITIVITY    8.75f    // mdps/digit at 250 dps
#define ACC_SENSITIVITY     1.0f     // mg/digit at +/-2g (LSM303DLHC)

// Control loop
#define DT                  0.005f   // 5 ms = 200 Hz timer ISR
#define ALPHA               0.98f    // Complementary filter weight
#define RAD2DEG             57.2957795f

// ---- PID Gains ----
#define KP                  60.0f
#define KI                  0.2f
#define KD                  2.5f

// PID limits
#define PID_OUTPUT_MAX      999.0f   // matches PWM period
#define PID_OUTPUT_MIN     -999.0f
#define INTEGRAL_MAX        500.0f   // anti-windup clamp

// SETPOINT auto-measured at startup (robot's natural resting angle)
static float SETPOINT = 0.0f;

// Safety cutoff: once |tilt angle| reaches this value, motors are disabled
#define MOTOR_OFF_ANGLE_DEG 30.0f

// PWM period (must match TIM2 Period = 999)
#define PWM_PERIOD          999

// ---- Bluetooth control parameters ----
// BT_LEAN_DEGREES: How much to bias the setpoint per command.
//   2.0 deg = gentle, slow movement. Try 3.0-4.0 if it barely moves.
// BT_COMMAND_MS:   How long the lean lasts before auto-stop.
//   400 ms gives a short "scoot". Bigger = longer travel per tap.
#define BT_LEAN_DEGREES     2.0f
#define BT_COMMAND_MS       400

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;


TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Shared variables between ISR and main loop
volatile float shared_angle = 0.0f;
volatile float shared_pid_output = 0.0f;
volatile uint8_t display_flag = 0;
volatile uint8_t shared_motor_cutoff = 0;
volatile float shared_gyroX = 0.0f;
volatile float shared_accX  = 0.0f;

// Calibration data
Gyro_Data  gyro_cal = {0.0f, 0.0f, 0.0f};
Accel_Data accel_cal = {0.0f, 0.0f, 0.0f};

// Motor cutoff
volatile uint8_t motor_cutoff_latched = 0;
#define STARTUP_GRACE_TICKS  200

// ---- Bluetooth state ----
volatile float setpoint_offset = 0.0f;    // added to SETPOINT each ISR tick
volatile uint32_t bt_command_expiry = 0;  // HAL_GetTick() when command auto-expires
volatile uint8_t bt_rx_byte = 0;                   // single-byte RX buffer for USART1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART_Print(const char *msg);
void BT_Print(const char *msg);
void Gyro_Init(void);
void Gyro_Calibration(Gyro_Data *data);
void Accel_Init(void);
void Accel_Calibration(Accel_Data *data);
void Gyro_ReadXYZ(float *gx, float *gy, float *gz);
void Accel_ReadXYZ(float *ax, float *ay, float *az);
uint8_t Gyro_ReadReg(uint8_t reg);
void Gyro_WriteReg(uint8_t reg, uint8_t val);
void Motor_SetSpeed(float pid_output);
float Measure_Resting_Angle(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ==================== UART (debug, USART2) ====================
void UART_Print(const char *msg) {
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

// ==================== Bluetooth UART (USART1) ====================
// Optional helper to send strings out the Bluetooth link (phone-side telemetry)
void BT_Print(const char *msg) {
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

// Called by HAL whenever a UART completes a 1-byte interrupt-driven receive.
// We only care about USART1 (the Bluetooth module). USART2 RX is unused.
// Commands handled:
//   'F' -> lean forward briefly  (drive forward "a bit")
//   'B' -> lean backward briefly (drive backward "a bit")
//   'S' -> stop (recenter setpoint immediately)
// Lowercase variants are also accepted in case the phone app sends them.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    switch (bt_rx_byte) {
      case 'F': case 'f':
        setpoint_offset = +BT_LEAN_DEGREES;
        bt_command_expiry = HAL_GetTick() + BT_COMMAND_MS;
        break;
      case 'B': case 'b':
        setpoint_offset = -BT_LEAN_DEGREES;
        bt_command_expiry = HAL_GetTick() + BT_COMMAND_MS;
        break;
      case 'S': case 's':
        setpoint_offset = 0.0f;
        bt_command_expiry = 0;
        break;
      default:
        break;  
    }
    // CRITICAL: Re-arm RX for the next byte. Without this, only ONE byte
    // is ever received and the robot stops responding after the first tap.
    HAL_UART_Receive_IT(&huart1, &bt_rx_byte, 1);
  }
}

// ==================== L3GD20 Gyro (SPI1) ====================
void Gyro_WriteReg(uint8_t reg, uint8_t val) {
  uint8_t txData[2] = { reg, val };
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}

uint8_t Gyro_ReadReg(uint8_t reg) {
  uint8_t txData = reg | 0x80;
  uint8_t rxData = 0;
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, &rxData, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
  return rxData;
}

void Gyro_Init(void) {
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  Gyro_WriteReg(GYRO_CTRL_REG1, 0x6F);
  Gyro_WriteReg(GYRO_CTRL_REG4, 0x80);
}

static void Gyro_ReadRawXYZ(float *gx, float *gy, float *gz) {
  uint8_t txAddr = GYRO_OUT_X_L | 0x80 | 0x40;
  uint8_t buf[6];

  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &txAddr, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, buf, 6, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);

  int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]);
  int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]);
  int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]);

  *gx = raw_x * GYRO_SENSITIVITY * 0.001f;
  *gy = raw_y * GYRO_SENSITIVITY * 0.001f;
  *gz = raw_z * GYRO_SENSITIVITY * 0.001f;
}

void Gyro_Calibration(Gyro_Data *data) {
  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  float gx, gy, gz;

  UART_Print("Calibrating gyro (keep robot STILL)...\r\n");

  for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
    Gyro_ReadRawXYZ(&gx, &gy, &gz);
    sum_x += gx;
    sum_y += gy;
    sum_z += gz;
    HAL_Delay(2);
  }

  data->offset_x = sum_x / (float)GYRO_CALIB_SAMPLES;
  data->offset_y = sum_y / (float)GYRO_CALIB_SAMPLES;
  data->offset_z = sum_z / (float)GYRO_CALIB_SAMPLES;
}

void Gyro_ReadXYZ(float *gx, float *gy, float *gz) {
  float raw_gx, raw_gy, raw_gz;
  Gyro_ReadRawXYZ(&raw_gx, &raw_gy, &raw_gz);

  *gx = raw_gx - gyro_cal.offset_x;
  *gy = raw_gy - gyro_cal.offset_y;
  *gz = raw_gz - gyro_cal.offset_z;
}

// ==================== LSM303DLHC Accel (I2C) ====================
void Accel_Init(void) {
  uint8_t data;
  data = 0x57;
  HAL_I2C_Mem_Write(&hi2c1, ACC_I2C_ADDR, ACC_CTRL_REG1_A,
                    I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
  data = 0x88;
  HAL_I2C_Mem_Write(&hi2c1, ACC_I2C_ADDR, ACC_CTRL_REG4_A,
                    I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

void Accel_ReadXYZ(float *ax, float *ay, float *az) {
  uint8_t buf[6];
  HAL_I2C_Mem_Read(&hi2c1, ACC_I2C_ADDR, ACC_OUT_X_L_A | 0x80,
                   I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);

  int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]) >> 4;
  int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]) >> 4;
  int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]) >> 4;

  *ax = raw_x * ACC_SENSITIVITY;
  *ay = raw_y * ACC_SENSITIVITY;
  *az = raw_z * ACC_SENSITIVITY;
}

void Accel_Calibration(Accel_Data *data) {
  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  float ax, ay, az;

  UART_Print("Calibrating accel (keep robot UPRIGHT and STILL)...\r\n");

  for (int i = 0; i < ACC_CALIB_SAMPLES; i++) {
    Accel_ReadXYZ(&ax, &ay, &az);
    sum_x += ax;
    sum_y += ay;
    sum_z += az;
    HAL_Delay(2);
  }

  data->offset_x = sum_x / (float)ACC_CALIB_SAMPLES;
  data->offset_y = sum_y / (float)ACC_CALIB_SAMPLES;
  data->offset_z = (sum_z / (float)ACC_CALIB_SAMPLES) - 1000.0f;
}

float Measure_Resting_Angle(void) {
  float angle = 0.0f;
  float sum = 0.0f;
  int samples = 100;

  for (int i = 0; i < 50; i++) {
    float gx, gy, gz, ax, ay, az;
    Gyro_ReadXYZ(&gx, &gy, &gz);
    Accel_ReadXYZ(&ax, &ay, &az);

    ax -= accel_cal.offset_x;
    az -= accel_cal.offset_z;

    float acc_angle = atan2f(ax, az) * RAD2DEG;
    angle = ALPHA * (angle + gy * 0.005f) + (1.0f - ALPHA) * acc_angle;
    HAL_Delay(5);
  }

  for (int i = 0; i < samples; i++) {
    float gx, gy, gz, ax, ay, az;
    Gyro_ReadXYZ(&gx, &gy, &gz);
    Accel_ReadXYZ(&ax, &ay, &az);

    ax -= accel_cal.offset_x;
    az -= accel_cal.offset_z;

    float acc_angle = atan2f(ax, az) * RAD2DEG;
    angle = ALPHA * (angle + gy * 0.005f) + (1.0f - ALPHA) * acc_angle;
    sum += angle;
    HAL_Delay(5);
  }

  return sum / (float)samples;
}

// ==================== Motor Control ====================
void Motor_SetSpeed(float pid_output) {
  if (pid_output > PWM_PERIOD) pid_output = PWM_PERIOD;
  if (pid_output < -PWM_PERIOD) pid_output = -PWM_PERIOD;

  uint32_t pwm_val = (uint32_t)fabsf(pid_output);

  if (pid_output >= 0) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_val);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_val);
}

// ==================== Timer ISR (200 Hz) ====================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    static float tilt_angle = 0.0f;
    static float integral = 0.0f;
    static float prev_error = 0.0f;
    static int counter = 0;
    static uint32_t startup_ticks = 0;
    static uint8_t filter_seeded = 0;

    // ---- Read sensors ----
    float gx, gy, gz;
    float ax, ay, az;
    Gyro_ReadXYZ(&gx, &gy, &gz);
    Accel_ReadXYZ(&ax, &ay, &az);

    ax -= accel_cal.offset_x;
    az -= accel_cal.offset_z;

    // ---- Complementary filter ----
    float acc_angle = atan2f(ax, az) * RAD2DEG;

    if (!filter_seeded) {
      tilt_angle = acc_angle;
      filter_seeded = 1;
    } else {
      tilt_angle = ALPHA * (tilt_angle + gy * DT) + (1.0f - ALPHA) * acc_angle;
    }

    // ---- Grace period & safety cutoff ----
    if (startup_ticks < STARTUP_GRACE_TICKS) {
      startup_ticks++;
    } else {
      if (!motor_cutoff_latched && (fabsf(tilt_angle) >= MOTOR_OFF_ANGLE_DEG)) {
        motor_cutoff_latched = 1;
      }
    }

    if (motor_cutoff_latched) {
      Motor_SetSpeed(0.0f);
      integral = 0.0f;
      prev_error = 0.0f;

      // Clear any pending BT command so it doesn't restart motion on recovery
      setpoint_offset = 0.0f;
      bt_command_expiry = 0;

      shared_angle      = tilt_angle;
      shared_gyroX      = gy;          
      shared_accX       = ax;          
      shared_pid_output = 0.0f;
      shared_motor_cutoff = 1;

      counter++;
      if (counter >= 20) {
        display_flag = 1;
        counter = 0;
      }
      return;
    }

    // ---- Bluetooth: auto-expire lean commands ----
    // If a phone command was issued but BT_COMMAND_MS elapsed, recenter.
    // This prevents runaway if the phone disconnects mid-command.
    if (bt_command_expiry != 0 && (HAL_GetTick() - bt_command_expiry) < 0x80000000U)
    {
    setpoint_offset = 0.0f;
    bt_command_expiry = 0;
    }

    // ---- PID Controller ----
    // Note: SETPOINT is the natural resting angle. setpoint_offset shifts it
    // forward/backward briefly to make the robot lean and roll in that direction.
    float effective_setpoint = SETPOINT + setpoint_offset;
    float error = effective_setpoint - tilt_angle;

    float p_term = KP * error;

    integral += error * DT;
    if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
    if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;
    float i_term = KI * integral;

    float derivative = (error - prev_error) / DT;
    float d_term = KD * derivative;
    prev_error = error;

    float output = p_term + i_term + d_term;

    if (output > PID_OUTPUT_MAX) output = PID_OUTPUT_MAX;
    if (output < PID_OUTPUT_MIN) output = PID_OUTPUT_MIN;

    Motor_SetSpeed(output);

    shared_angle      = tilt_angle;
    shared_gyroX      = gy;          // ADD
    shared_accX       = ax;          // ADD
    shared_pid_output = output;
    shared_motor_cutoff = 0;

    counter++;
    if (counter >= 20) {
      display_flag = 1;
      counter = 0;
    }
  }
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
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(100);

  // ---- Initialize sensors ----
  Gyro_Init();
  Accel_Init();
  HAL_Delay(100);

  char msg[128];

  uint8_t who = Gyro_ReadReg(GYRO_WHO_AM_I);
  snprintf(msg, sizeof(msg), "Gyro WHO_AM_I: 0x%02X (expect 0xD4 or 0xD7)\r\n", who);
  UART_Print(msg);

  uint8_t acc_who = 0;
  HAL_I2C_Mem_Read(&hi2c1, ACC_I2C_ADDR, 0x0F,
                   I2C_MEMADD_SIZE_8BIT, &acc_who, 1, HAL_MAX_DELAY);
  snprintf(msg, sizeof(msg), "Accel WHO_AM_I: 0x%02X (expect 0x33)\r\n", acc_who);
  UART_Print(msg);

  // ---- Calibrate ----
  UART_Print("=== CALIBRATION: Hold robot UPRIGHT and STILL ===\r\n");
  HAL_Delay(1000);

  Gyro_Calibration(&gyro_cal);
  snprintf(msg, sizeof(msg), "Gyro offsets (x1000 dps): X=%d Y=%d Z=%d\r\n",
           (int)(gyro_cal.offset_x * 1000),
           (int)(gyro_cal.offset_y * 1000),
           (int)(gyro_cal.offset_z * 1000));
  UART_Print(msg);

  Accel_Calibration(&accel_cal);
  snprintf(msg, sizeof(msg), "Accel offsets (mg): X=%d Y=%d Z_dev=%d\r\n",
           (int)(accel_cal.offset_x),
           (int)(accel_cal.offset_y),
           (int)(accel_cal.offset_z));
  UART_Print(msg);

  UART_Print("Measuring resting angle...\r\n");
  SETPOINT = Measure_Resting_Angle();
  snprintf(msg, sizeof(msg), "Auto SETPOINT: %d (x100=%d)\r\n",
           (int)SETPOINT, (int)(SETPOINT * 100));
  UART_Print(msg);

  snprintf(msg, sizeof(msg), "PID: Kp=%d Ki_x10=%d Kd_x10=%d\r\n",
           (int)KP, (int)(KI * 10), (int)(KD * 10));
  UART_Print(msg);

  HAL_Delay(500);

  // ---- Start PWM channels ----
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

  // ---- Start 200 Hz control loop ----
  HAL_TIM_Base_Start_IT(&htim6);

  UART_Print("Control loop started at 200 Hz\r\n");
  UART_Print("Format: angle,pid_output,cutoff,bt_offset\r\n");

  // ---- Arm Bluetooth RX (interrupt-driven, 1 byte at a time) ----
  // Without this, the RxCpltCallback never fires and BT does nothing.
  HAL_UART_Receive_IT(&huart1, &bt_rx_byte, 1);
  UART_Print("Bluetooth RX armed on USART1 @ 9600 (F/B/S commands)\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (display_flag == 1) {
    display_flag = 0;
    float angle  = shared_angle;
    float pid    = shared_pid_output;
    float gx_val = shared_gyroX;
    float ax_val = shared_accX;
    uint8_t cutoff = shared_motor_cutoff;

    // Encode floats as integers to avoid float-printf dependency
    int angle_int  = (int)angle;
    int angle_frac = (int)(fabsf(angle - (float)angle_int) * 10);
    int gyro_x100  = (int)(gx_val * 100.0f);  // dps × 100  (2 decimal places)
    int acc_int    = (int)(ax_val);            // mg (integer precision is fine)
    int pid_int    = (int)pid;

    // Format: angle, gyroX_x100, accX, pid, cutoff
    snprintf(msg, sizeof(msg), "%d.%d,%d,%d,%d,%u\r\n",
             angle_int, angle_frac, gyro_x100, acc_int, pid_int, cutoff);
    UART_Print(msg);
}
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
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
  hi2c1.Init.Timing = 0x2000090E;
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PB0 PB1 PB2 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
