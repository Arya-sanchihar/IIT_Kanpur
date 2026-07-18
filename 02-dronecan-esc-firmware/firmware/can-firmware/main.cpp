/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ESC Firmware — STM32F303RET6 + DRV8353SRTAT + DroneCAN
  *                   CubeMX generated code + ESC application code
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "canard.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* --- ESC IDENTITY --- */
#define MY_NODE_ID              10
#define MY_ESC_INDEX            0

/* --- DroneCAN IDs [libcanard ESCNode example] --- */
#define UAVCAN_NODE_STATUS_ID                       341
#define UAVCAN_NODE_STATUS_SIGNATURE                0x0F0868D0777949D3ULL
#define UAVCAN_NODE_STATUS_SIZE                     7
#define UAVCAN_GET_NODE_INFO_ID                     1
#define UAVCAN_GET_NODE_INFO_SIGNATURE              0xEE468A8121C46A9EULL
#define UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID          1030
#define UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE   0x217F5C87D7EC951DULL
#define UAVCAN_EQUIPMENT_ESC_STATUS_ID              1034
#define UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE       0xA9AF28AEA2FBB254ULL
#define UAVCAN_EQUIPMENT_ESC_STATUS_SIZE            11

/* --- NODE HEALTH --- */
#define NODE_HEALTH_OK          0
#define NODE_HEALTH_WARNING     1
#define NODE_MODE_OPERATIONAL   0

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
#define WATCHDOG_TIMEOUT_MS     500
#define MIN_STARTUP_DUTY        150
#define BEMF_FILTER_COUNT       3
#define STARTUP_STEP_DELAY_START_US   8000
#define STARTUP_STEP_DELAY_MIN_US     800
#define STARTUP_HANDOFF_CROSSES       10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* --- libcanard [libcanard ESCNode example] --- */
static CanardInstance   g_canard;
static uint8_t          g_canard_pool[1024];
static uint8_t          g_node_status_tid   = 0;
static uint8_t          g_esc_status_tid    = 0;
static uint32_t         g_uptime_sec        = 0;
static uint8_t          g_node_health       = NODE_HEALTH_OK;
static volatile uint32_t g_can_rx_count     = 0;

/* --- Motor command state --- */
static int16_t          g_throttle          = 0;
static uint32_t         g_current_duty      = 0;
static uint32_t         g_target_duty       = 0;
static uint8_t          g_armed             = 0;
static uint32_t         g_last_cmd_tick     = 0;

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
static float            g_temp_kelvin       = 293.0f;
static uint32_t         g_error_count       = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

    /* 6x PWM mode [CHANGED from original 3x] */
    uint16_t tx_driver_control =
          0 << 10 | 0 << 9 | 0 << 8 | 1 << 7
        | 0 << 5  | 0 << 4 | 0 << 3 | 0 << 2
        | 0 << 1  | 0 << 0;
    drv_write_reg(DRIVER_CONTROL, tx_driver_control);

    /* 300mA gate drive [identical to open-esc-firmware] */
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

    /* OCP [identical to open-esc-firmware] */
    uint16_t tx_overcurrent_control =
          0    << 10
        | 0b01 <<  8
        | 0b01 <<  6
        | 0b10 <<  4
        | 0b0000 << 0;
    drv_write_reg(OVERCURRENT_CONTROL, tx_overcurrent_control);

    /* CSA 40V/V [identical to open-esc-firmware] */
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
    if (fault1 & (1 << 7)) g_node_health = NODE_HEALTH_WARNING;
    if (fault1 & (1 << 6)) g_node_health = NODE_HEALTH_WARNING;
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
   DroneCAN CALLBACKS [libcanard ESCNode_C++ example]
   ================================================================ */

static bool shouldAcceptTransfer(
    const CanardInstance* ins, uint64_t* out_data_type_signature,
    uint16_t data_type_id, CanardTransferType transfer_type,
    uint8_t source_node_id)
{
    (void)ins; (void)transfer_type; (void)source_node_id;
    if (data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID)
    {
        *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
        return true;
    }
    if (data_type_id == UAVCAN_GET_NODE_INFO_ID)
    {
        *out_data_type_signature = UAVCAN_GET_NODE_INFO_SIGNATURE;
        return true;
    }
    return false;
}

static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    (void)ins;
    if (transfer->data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID)
    {
        uint8_t num_escs = transfer->payload_len / 2;
        if (MY_ESC_INDEX >= num_escs) return;
        int16_t raw = 0;
        uint8_t* p  = (uint8_t*)transfer->payload_head;
        memcpy(&raw, &p[MY_ESC_INDEX * 2], 2);
        g_throttle      = raw;
        g_last_cmd_tick = HAL_GetTick();
    }
}

/* ================================================================
   HEARTBEAT + TELEMETRY [libcanard ESCNode_C++ example]
   ================================================================ */

static void sendNodeStatus(void)
{
    uint8_t buf[UAVCAN_NODE_STATUS_SIZE];
    buf[0] = (uint8_t)(g_uptime_sec);
    buf[1] = (uint8_t)(g_uptime_sec >> 8);
    buf[2] = (uint8_t)(g_uptime_sec >> 16);
    buf[3] = (uint8_t)(g_uptime_sec >> 24);
    buf[4] = (g_node_health << 6) | (NODE_MODE_OPERATIONAL << 3);
    buf[5] = 0; buf[6] = 0;
    canardBroadcast(&g_canard, UAVCAN_NODE_STATUS_SIGNATURE,
                    UAVCAN_NODE_STATUS_ID, &g_node_status_tid,
                    CANARD_TRANSFER_PRIORITY_LOW, buf, UAVCAN_NODE_STATUS_SIZE);
}

static void sendESCStatus(void)
{
    uint8_t buf[UAVCAN_EQUIPMENT_ESC_STATUS_SIZE];
    memset(buf, 0, sizeof(buf));
    buf[0] = (uint8_t)(g_error_count);
    buf[1] = (uint8_t)(g_error_count >> 8);
    buf[2] = (uint8_t)(g_error_count >> 16);
    buf[3] = (uint8_t)(g_error_count >> 24);
    uint16_t v = (uint16_t)(g_voltage_volts * 100.0f);
    buf[4] = (uint8_t)(v); buf[5] = (uint8_t)(v >> 8);
    int16_t i = (int16_t)(g_current_amps * 100.0f);
    buf[6] = (uint8_t)(i); buf[7] = (uint8_t)(i >> 8);
    uint16_t t = (uint16_t)(g_temp_kelvin * 100.0f);
    buf[8] = (uint8_t)(t); buf[9] = (uint8_t)(t >> 8);
    buf[10] = (uint8_t)(MY_ESC_INDEX & 0x1F);
    canardBroadcast(&g_canard, UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
                    UAVCAN_EQUIPMENT_ESC_STATUS_ID, &g_esc_status_tid,
                    CANARD_TRANSFER_PRIORITY_LOW, buf,
                    UAVCAN_EQUIPMENT_ESC_STATUS_SIZE);
}

/* ================================================================
   PHASE OUTPUT CONTROL [AM32 phaseouts.c — adapted for STM32F303]
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
   COMMUTATION [AM32 main.c commutate() + zcfoundroutine()]
   ================================================================ */

static void applyStep(uint8_t step, uint32_t duty)
{
    switch(step)
    {
        case 0: phaseA_PWM(duty); phaseB_LOW();      phaseC_FLOAT(); g_bemf_rising=0; break;
        case 1: phaseA_PWM(duty); phaseB_FLOAT();    phaseC_LOW();   g_bemf_rising=1; break;
        case 2: phaseA_FLOAT();   phaseB_PWM(duty);  phaseC_LOW();   g_bemf_rising=0; break;
        case 3: phaseA_LOW();     phaseB_PWM(duty);  phaseC_FLOAT(); g_bemf_rising=1; break;
        case 4: phaseA_LOW();     phaseB_FLOAT();    phaseC_PWM(duty); g_bemf_rising=0; break;
        case 5: phaseA_FLOAT();   phaseB_LOW();      phaseC_PWM(duty); g_bemf_rising=1; break;
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
   THROTTLE + WATCHDOG
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

static void safety_watchdog_check(void)
{
    if (g_armed && (HAL_GetTick() - g_last_cmd_tick > WATCHDOG_TIMEOUT_MS))
    {
        g_throttle = 0; g_armed = 0; g_running = 0;
        g_current_duty = 0; g_node_health = NODE_HEALTH_WARNING;
        allPhases_OFF();
    }
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
    if (g_voltage_volts < 10.5f && g_voltage_volts > 1.0f)
        g_node_health = NODE_HEALTH_WARNING;
}

/* ================================================================
   CAN HARDWARE [STM32CubeF3 CAN_Networking example + libcanard]
   ================================================================ */

static void CAN_Filter_Init(void)
{
    CAN_FilterTypeDef f;
    f.FilterBank=0; f.FilterMode=CAN_FILTERMODE_IDMASK;
    f.FilterScale=CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh=0; f.FilterIdLow=0;
    f.FilterMaskIdHigh=0; f.FilterMaskIdLow=0;
    f.FilterFIFOAssignment=CAN_RX_FIFO0;
    f.FilterActivation=ENABLE; f.SlaveStartFilterBank=14;
    HAL_CAN_ConfigFilter(&hcan, &f);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_p)
{
	g_can_rx_count++;
	CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hcan_p, CAN_RX_FIFO0, &hdr, data) != HAL_OK) return;
    CanardCANFrame frame;
    frame.id       = hdr.ExtId | CANARD_CAN_FRAME_EFF;
    frame.data_len = (uint8_t)hdr.DLC;
    memcpy(frame.data, data, frame.data_len);
    canardHandleRxFrame(&g_canard, &frame, (uint64_t)HAL_GetTick() * 1000ULL);
}

static void CAN_TransmitQueue(void)
{
    const CanardCANFrame* txf;
    while ((txf = canardPeekTxQueue(&g_canard)) != NULL)
    {
        CAN_TxHeaderTypeDef txh;
        uint8_t txd[8]; uint32_t mb;
        txh.ExtId=txf->id & CANARD_CAN_EXT_ID_MASK;
        txh.IDE=CAN_ID_EXT; txh.RTR=CAN_RTR_DATA;
        txh.DLC=txf->data_len; txh.TransmitGlobalTime=DISABLE;
        memcpy(txd, txf->data, txf->data_len);
        if (HAL_CAN_AddTxMessage(&hcan, &txh, txd, &mb) == HAL_OK)
            canardPopTxQueue(&g_canard);
        else break;
    }
}

/* ================================================================
   INTERRUPT HANDLERS [AM32 stm32f0xx_it.c COMP pattern]
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
  /* USER CODE BEGIN 2 */

    /* Enable DWT cycle counter for microsecond delays */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    /* Safe state */
    allPhases_OFF();

    /* Start TIM1 — all 6 complementary PWM channels */
    HAL_TIM_PWM_Start(&htim1,    TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,    TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,    TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* Init DRV8353S gate driver */
    DRV8353_Init();
    DRV8353_Calibrate();

    /* Init CAN filter and start */
    CAN_Filter_Init();

    /* Init libcanard */
    canardInit(&g_canard,
               g_canard_pool, sizeof(g_canard_pool),
               onTransferReceived,
               shouldAcceptTransfer,
               NULL);
    g_canard.node_id = MY_NODE_ID;

    /* Disable COMP interrupt until motor is running */
    HAL_NVIC_DisableIRQ(COMP1_2_3_IRQn);

    uint32_t t_heartbeat = 0;
    uint32_t t_telemetry = 0;
    uint32_t t_watchdog  = 0;
    uint32_t t_cleanup   = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* Drain CAN TX queue */
        CAN_TransmitQueue();

        /* Heartbeat every 1 second */
        if ((now - t_heartbeat) >= 1000)
        {
            t_heartbeat = now;
            g_uptime_sec++;
            sendNodeStatus();
        }

        /* ESC telemetry every 10ms */
        if ((now - t_telemetry) >= 10)
        {
            t_telemetry = now;
            updateADC();
            sendESCStatus();
        }

        /* Throttle + watchdog every 1ms */
        if ((now - t_watchdog) >= 1)
        {
            t_watchdog = now;
            apply_throttle(g_throttle);
            safety_watchdog_check();
        }

        /* BEMF polling at low RPM */
        if (g_running && g_avg_interval > 5000)
            pollBEMF();

        /* Cleanup stale libcanard transfers every 1 second */
        if ((now - t_cleanup) >= 1000)
        {
            t_cleanup = now;
            canardCleanupStaleTransfers(&g_canard, (uint64_t)now * 1000ULL);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
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
