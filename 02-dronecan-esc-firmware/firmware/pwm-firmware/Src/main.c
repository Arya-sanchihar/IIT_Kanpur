/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ESC Firmware — STM32F303RET6 + DRV8353SRTAT
  *                   Hardcoded throttle test — verifies PWM output via UART
  *
  * Motor control from:
  *   AM32 MultiRotor ESC Firmware (AlkaMotors/AM32-MultiRotor-ESC-firmware)
  *   Sapog open-source ESC (PX4/sapog)
  *   open-esc-firmware (open-esc/open-esc-firmware)
  *
  * UART output → ST-Link VCP → COM5 → PowerShell terminal on PC
  * Read with: $p=New-Object IO.Ports.SerialPort COM5,115200,None,8,One;$p.Open();while($true){try{Write-Host $p.ReadLine()}catch{}}
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "comp.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* --- DRV8353S REGISTER ADDRESSES [open-esc-firmware drv8353rs.h] --- */
#define DRIVER_CONTROL          0x02
#define GATE_DRIVE_HIGH_CONTROL 0x03
#define GATE_DRIVE_LOW_CONTROL  0x04
#define OVERCURRENT_CONTROL     0x05
#define CURRENT_SENSE_CONTROL   0x06
#define DRIVER_CONFIGURATION    0x07
#define BOTTOM_11_BIT_MASK      0x7FF
#define BOTTOM_4_BIT_MASK       0x0F

/* --- MOTOR CONSTANTS --- */
#define PWM_ARR                 2100
#define RAMP_STEP               3
#define MIN_STARTUP_DUTY        150
#define BEMF_FILTER_COUNT       3
#define STARTUP_STEP_DELAY_START_US   8000
#define STARTUP_STEP_DELAY_MIN_US     800
#define STARTUP_HANDOFF_CROSSES       10

/* --- TEST THROTTLE ---
 * Hardcoded throttle value for PWM verification test
 * Range: 0-8191 (maps to 0-100% duty)
 * 2000 = ~24% duty cycle — safe for bench test without motor connected
 */
#define TEST_THROTTLE_VALUE     2000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* --- Motor command state --- */
static int16_t          g_throttle          = 0;
static uint32_t         g_current_duty      = 0;
static uint32_t         g_target_duty       = 0;
static uint8_t          g_armed             = 0;

/* --- Commutation state [AM32 main.c pattern] --- */
static uint8_t          g_step              = 0;
static uint8_t          g_direction         = 0;
static uint8_t          g_running           = 0;
static uint32_t         g_comm_intervals[6] = {9000,9000,9000,9000,9000,9000};
static uint8_t          g_interval_idx      = 0;
static uint32_t         g_avg_interval      = 9000;
static uint32_t         g_last_comm_tick    = 0;
static uint8_t          g_zero_crosses      = 0;
static uint8_t          g_bemf_rising       = 1;

/* --- Startup state [Sapog pattern] --- */
static uint32_t         g_startup_delay_us  = STARTUP_STEP_DELAY_START_US;
static uint8_t          g_startup_step      = 0;

/* --- Telemetry --- */
static float            g_rpm               = 0.0f;
static float            g_current_amps      = 0.0f;
static float            g_voltage_volts     = 0.0f;
static uint32_t         g_error_count       = 0;
static uint8_t          g_node_health       = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ================================================================
   UART RETARGET — printf → USART2 → ST-Link VCP → PC terminal
   ================================================================ */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* ================================================================
   DRV8353S SPI DRIVER
   [open-esc-firmware drv8353rs.c — ChibiOS calls replaced with HAL]
   ================================================================ */

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000UL);
    while ((DWT->CYCCNT - start) < ticks);
}

static uint16_t spi_exchange_word(uint16_t tx_word)
{
    uint8_t tx[2], rx[2];
    tx[0] = (uint8_t)(tx_word >> 8);
    tx[1] = (uint8_t)(tx_word & 0xFF);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return (uint16_t)((rx[0] << 8) | rx[1]);
}

static uint16_t drv_read_reg(uint8_t addr)
{
    addr &= BOTTOM_4_BIT_MASK;
    return spi_exchange_word((1 << 15) | (addr << 11)) & BOTTOM_11_BIT_MASK;
}

static void drv_write_reg(uint8_t addr, uint16_t data)
{
    addr &= BOTTOM_4_BIT_MASK;
    data &= BOTTOM_11_BIT_MASK;
    delay_us(1);
    spi_exchange_word((0 << 15) | (addr << 11) | data);
}

static void drv_enable(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    delay_us(40);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}

static void drv_lock_spi(void)
{
    uint16_t cmd = drv_read_reg(GATE_DRIVE_HIGH_CONTROL);
    cmd &= ~(0b111 << 8);
    cmd |=  (0b110 << 8);
    drv_write_reg(GATE_DRIVE_HIGH_CONTROL, cmd);
}

static void drv_unlock_spi(void)
{
    uint16_t cmd = drv_read_reg(GATE_DRIVE_HIGH_CONTROL);
    cmd &= ~(0b111 << 8);
    cmd |=  (0b011 << 8);
    drv_write_reg(GATE_DRIVE_HIGH_CONTROL, cmd);
}

static bool drv_has_fault(void)
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET;
}

static void DRV8353_Init(void)
{
    drv_enable();
    drv_unlock_spi();

    uint16_t tx_driver_control =
          0 << 10 | 0 << 9 | 0 << 8 | 1 << 7
        | 0 << 5  | 0 << 4 | 0 << 3 | 0 << 2
        | 0 << 1  | 0 << 0;
    drv_write_reg(DRIVER_CONTROL, tx_driver_control);

    uint16_t tx_gate_drive_high =
          0b000  << 8
        | 0b0100 << 4
        | 0b0011 << 0;
    drv_write_reg(GATE_DRIVE_HIGH_CONTROL, tx_gate_drive_high);

    uint16_t tx_gate_drive_low =
          0    << 10
        | 0b01 <<  8
        | 0b0100 << 4
        | 0b0011 << 0;
    drv_write_reg(GATE_DRIVE_LOW_CONTROL, tx_gate_drive_low);

    uint16_t tx_overcurrent_control =
          0    << 10
        | 0b01 <<  8
        | 0b01 <<  6
        | 0b10 <<  4
        | 0b0000 << 0;
    drv_write_reg(OVERCURRENT_CONTROL, tx_overcurrent_control);

    uint16_t tx_current_sense_control =
          0    << 10
        | 1    <<  9
        | 0    <<  8
        | 0b11 <<  6
        | 0    <<  5
        | 0    <<  4
        | 0    <<  3
        | 0    <<  2
        | 0b11 <<  0;
    drv_write_reg(CURRENT_SENSE_CONTROL, tx_current_sense_control);

    uint16_t tx_driver_configuration = 1 << 0;
    drv_write_reg(DRIVER_CONFIGURATION, tx_driver_configuration);

    drv_lock_spi();
}

static void DRV8353_Calibrate(void)
{
    drv_unlock_spi();
    uint16_t csa = drv_read_reg(CURRENT_SENSE_CONTROL);
    csa |= 0b111 << 2;
    drv_write_reg(CURRENT_SENSE_CONTROL, csa);
    HAL_Delay(1000);
    csa &= ~(0b111 << 2);
    drv_write_reg(CURRENT_SENSE_CONTROL, csa);
    drv_lock_spi();
}

static void DRV8353_HandleFault(void)
{
    uint16_t fault1 = drv_read_reg(0x00);
    if (fault1 & (1 << 7)) g_node_health = 1;
    if (fault1 & (1 << 6)) g_node_health = 1;
    if (fault1 & 0x3F)
    {
        g_error_count++;
        g_throttle = 0; g_armed = 0; g_running = 0;
    }
    drv_unlock_spi();
    drv_write_reg(DRIVER_CONTROL, 0x001);
    HAL_Delay(1);
    drv_write_reg(DRIVER_CONTROL, 0x000);
    drv_lock_spi();
}

/* ================================================================
   PHASE OUTPUT CONTROL [AM32 phaseouts.c]
   ================================================================ */

static void phaseA_PWM(uint32_t d) { TIM1->CCR1=d; TIM1->CCER|=(TIM_CCER_CC1E|TIM_CCER_CC1NE); }
static void phaseA_LOW(void)       { TIM1->CCR1=0; TIM1->CCER&=~TIM_CCER_CC1E;  TIM1->CCER|=TIM_CCER_CC1NE; }
static void phaseA_FLOAT(void)     { TIM1->CCR1=0; TIM1->CCER&=~(TIM_CCER_CC1E|TIM_CCER_CC1NE); }
static void phaseB_PWM(uint32_t d) { TIM1->CCR2=d; TIM1->CCER|=(TIM_CCER_CC2E|TIM_CCER_CC2NE); }
static void phaseB_LOW(void)       { TIM1->CCR2=0; TIM1->CCER&=~TIM_CCER_CC2E;  TIM1->CCER|=TIM_CCER_CC2NE; }
static void phaseB_FLOAT(void)     { TIM1->CCR2=0; TIM1->CCER&=~(TIM_CCER_CC2E|TIM_CCER_CC2NE); }
static void phaseC_PWM(uint32_t d) { TIM1->CCR3=d; TIM1->CCER|=(TIM_CCER_CC3E|TIM_CCER_CC3NE); }
static void phaseC_LOW(void)       { TIM1->CCR3=0; TIM1->CCER&=~TIM_CCER_CC3E;  TIM1->CCER|=TIM_CCER_CC3NE; }
static void phaseC_FLOAT(void)     { TIM1->CCR3=0; TIM1->CCER&=~(TIM_CCER_CC3E|TIM_CCER_CC3NE); }
static void allPhases_OFF(void)
{
    TIM1->CCR1=0; TIM1->CCR2=0; TIM1->CCR3=0;
    TIM1->CCER &= ~(TIM_CCER_CC1E|TIM_CCER_CC1NE|
                    TIM_CCER_CC2E|TIM_CCER_CC2NE|
                    TIM_CCER_CC3E|TIM_CCER_CC3NE);
}

/* ================================================================
   COMMUTATION [AM32 main.c commutate()]
   ================================================================ */

static void applyStep(uint8_t step, uint32_t duty)
{
    switch(step)
    {
        case 0: phaseA_PWM(duty); phaseB_LOW();     phaseC_FLOAT(); g_bemf_rising=0; break;
        case 1: phaseA_PWM(duty); phaseB_FLOAT();   phaseC_LOW();   g_bemf_rising=1; break;
        case 2: phaseA_FLOAT();   phaseB_PWM(duty); phaseC_LOW();   g_bemf_rising=0; break;
        case 3: phaseA_LOW();     phaseB_PWM(duty); phaseC_FLOAT(); g_bemf_rising=1; break;
        case 4: phaseA_LOW();     phaseB_FLOAT();   phaseC_PWM(duty); g_bemf_rising=0; break;
        case 5: phaseA_FLOAT();   phaseB_LOW();     phaseC_PWM(duty); g_bemf_rising=1; break;
        default: allPhases_OFF(); break;
    }
}

static void commutate(void)
{
    uint32_t now      = TIM2->CNT;
    uint32_t interval = now - g_last_comm_tick;
    g_last_comm_tick  = now;
    g_comm_intervals[g_interval_idx] = interval;
    g_interval_idx = (g_interval_idx + 1) % 6;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 6; i++) sum += g_comm_intervals[i];
    g_avg_interval = sum / 6;
    if (g_avg_interval > 0)
        g_rpm = 60000000.0f / (g_avg_interval * 6.0f * 7.0f);
    if (g_direction == 0) g_step = (g_step + 1) % 6;
    else                  g_step = (g_step + 5) % 6;
    applyStep(g_step, g_current_duty);
}

static void pollBEMF(void)
{
    static uint8_t filter_count = 0;
    static uint8_t zc_found     = 0;
    uint8_t comp_out = (uint8_t)((COMP1->CSR >> 14) & 0x1);
    if (comp_out == g_bemf_rising)
    {
        filter_count++;
        if (filter_count >= BEMF_FILTER_COUNT && !zc_found)
        {
            zc_found = 1; filter_count = 0;
            uint32_t advance  = g_avg_interval / 6;
            uint32_t waitTime = g_avg_interval / 2;
            if (waitTime > advance) waitTime -= advance;
            __HAL_TIM_SET_AUTORELOAD(&htim2, waitTime);
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            HAL_TIM_Base_Start_IT(&htim2);
        }
    }
    else { filter_count = 0; zc_found = 0; }
}

/* ================================================================
   STARTUP [Sapog motor.cpp open-loop pattern]
   ================================================================ */

static void runStartup(void)
{
    uint32_t duty = (g_current_duty > MIN_STARTUP_DUTY) ? g_current_duty : MIN_STARTUP_DUTY;
    applyStep(g_startup_step, duty);
    delay_us(g_startup_delay_us);
    g_startup_step = (g_startup_step + 1) % 6;
    if (g_startup_delay_us > STARTUP_STEP_DELAY_MIN_US + 50)
        g_startup_delay_us -= 50;
    uint8_t comp_out = (uint8_t)((COMP1->CSR >> 14) & 0x1);
    if (comp_out == g_bemf_rising) g_zero_crosses++;
    if (g_zero_crosses >= STARTUP_HANDOFF_CROSSES)
    {
        g_running = 1; g_zero_crosses = 0;
        g_startup_delay_us = STARTUP_STEP_DELAY_START_US;
        HAL_NVIC_EnableIRQ(COMP1_2_3_IRQn);
    }
}

/* ================================================================
   THROTTLE
   ================================================================ */

static void apply_throttle(int16_t raw)
{
    if (!g_armed) { if (raw == 0) g_armed = 1; return; }
    if (raw == 0)
    {
        g_target_duty = 0; g_running = 0;
        g_startup_delay_us = STARTUP_STEP_DELAY_START_US;
        g_zero_crosses = 0; allPhases_OFF(); return;
    }
    if (raw < 0) { g_direction = 1; raw = -raw; }
    else           g_direction = 0;
    g_target_duty = (uint32_t)((raw / 8191.0f) * PWM_ARR);
    if (g_current_duty < g_target_duty)
    {
        g_current_duty += RAMP_STEP;
        if (g_current_duty > g_target_duty) g_current_duty = g_target_duty;
    }
    else if (g_current_duty > g_target_duty)
    {
        if (g_current_duty >= RAMP_STEP) g_current_duty -= RAMP_STEP;
        else g_current_duty = 0;
    }
    if (!g_running) runStartup();
    else            applyStep(g_step, g_current_duty);
}

/* ================================================================
   ADC SENSING
   ================================================================ */

static void updateADC(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
    {
        uint32_t raw = HAL_ADC_GetValue(&hadc1);
        float v_so   = (raw / 4095.0f) * 3.3f;
        g_current_amps = (v_so - 1.65f) * 50.0f;
    }
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
    {
        uint32_t raw = HAL_ADC_GetValue(&hadc1);
        g_voltage_volts = ((raw / 4095.0f) * 3.3f) * 11.0f;
    }
    HAL_ADC_Stop(&hadc1);
}

/* ================================================================
   INTERRUPT HANDLERS
   ================================================================ */

void COMP1_2_3_IRQHandler(void)
{
    uint32_t elapsed = TIM2->CNT - g_last_comm_tick;
    if (elapsed < (g_avg_interval / 2))
    {
        uint8_t comp_out = (uint8_t)((COMP1->CSR >> 14) & 0x1);
        if (comp_out != g_bemf_rising) { EXTI->PR |= EXTI_PR_PR21; return; }
    }
    uint32_t advance  = g_avg_interval / 6;
    uint32_t waitTime = g_avg_interval / 2;
    if (waitTime > advance) waitTime -= advance;
    if (waitTime < 10) waitTime = 10;
    HAL_TIM_Base_Stop(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim2, waitTime);
    HAL_TIM_Base_Start_IT(&htim2);
    EXTI->PR |= EXTI_PR_PR21;
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_Base_Stop(&htim2);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    commutate();
}

void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    if (drv_has_fault()) DRV8353_HandleFault();
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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

    /* Enable DWT cycle counter for microsecond delays */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    /* Safe state */
    allPhases_OFF();

    printf("\r\n=== ESC PWM TEST FIRMWARE ===\r\n");
    printf("Initializing DRV8353...\r\n");

    /* Start TIM1 — all 6 complementary PWM channels */
    HAL_TIM_PWM_Start(&htim1,    TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,    TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,    TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* Init DRV8353S gate driver */
    DRV8353_Init();

    /* Check for DRV fault immediately after init */
    if (drv_has_fault())
    {
        printf("ERROR: DRV8353 fault detected after init!\r\n");
        printf("Check SPI wiring and power supply.\r\n");
    }
    else
    {
        printf("DRV8353 init OK\r\n");
    }

    DRV8353_Calibrate();
    printf("DRV8353 calibrate OK\r\n");

    /* Disable COMP interrupt until motor is running */
    HAL_NVIC_DisableIRQ(COMP1_2_3_IRQn);

    /* Force armed + fixed throttle for bench test */
    g_armed    = 1;
    g_throttle = TEST_THROTTLE_VALUE;

    printf("Armed. Throttle = %d (target duty ~%lu / %d)\r\n",
           TEST_THROTTLE_VALUE,
           (uint32_t)((TEST_THROTTLE_VALUE / 8191.0f) * PWM_ARR),
           PWM_ARR);
    printf("Watching CCR registers every 500ms...\r\n\r\n");

    uint32_t t_throttle = 0;
    uint32_t t_adc      = 0;
    uint32_t t_print    = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* Apply throttle every 1ms */
        if ((now - t_throttle) >= 1)
        {
            t_throttle = now;
            apply_throttle(g_throttle);
        }

        /* ADC every 10ms */
        if ((now - t_adc) >= 10)
        {
            t_adc = now;
            updateADC();
        }

        /* BEMF polling at low RPM */
        if (g_running && g_avg_interval > 5000)
            pollBEMF();

        /* Print status every 500ms */
        if ((now - t_print) >= 500)
        {
            t_print = now;
            printf("CCR1=%4lu CCR2=%4lu CCR3=%4lu | duty=%4lu target=%4lu | step=%d running=%d | fault=%d\r\n",
                   TIM1->CCR1,
                   TIM1->CCR2,
                   TIM1->CCR3,
                   g_current_duty,
                   g_target_duty,
                   g_step,
                   g_running,
                   drv_has_fault() ? 1 : 0);
        }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
    __disable_irq();
    while (1) { }
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
