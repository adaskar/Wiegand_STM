/*
 * wiegand_port_stm32.c - STM32H5 (HAL) bidirectional port for wiegand.h
 */
#include "wiegand_port_stm32.h"
#include <string.h>
#include <stdio.h>

/* Board pin configuration */
#ifndef WIEGAND_D0_PORT
#define WIEGAND_D0_PORT    WIE_D0_GPIO_Port
#define WIEGAND_D0_PIN     WIE_D0_Pin
#endif
#ifndef WIEGAND_D1_PORT
#define WIEGAND_D1_PORT    WIE_D1_GPIO_Port
#define WIEGAND_D1_PIN     WIE_D1_Pin
#endif

TIM_HandleTypeDef htim2;

static wiegand_t     s_wiegand;
static wiegand_dir_t s_current_dir = WIEGAND_DIR_RX;
static bool          s_initialized = false;

/* Microsecond timestamp from TIM2 free-running counter */
static inline uint32_t now_us(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

uint32_t Wiegand_Port_GetNowUs(void)
{
    return now_us();
}

void Wiegand_Port_DelayUs(uint32_t us)
{
    uint32_t start = now_us();
    while ((now_us() - start) < us)
    {
        __NOP();
    }
}

uint8_t Wiegand_Port_ReadLevels(void)
{
    uint8_t lv = 0U;

    if (HAL_GPIO_ReadPin(WIEGAND_D0_PORT, WIEGAND_D0_PIN) == GPIO_PIN_SET)
    {
        lv |= WIEGAND_LEVEL_D0;
    }
    if (HAL_GPIO_ReadPin(WIEGAND_D1_PORT, WIEGAND_D1_PIN) == GPIO_PIN_SET)
    {
        lv |= WIEGAND_LEVEL_D1;
    }
    return lv;
}

static void on_edge(wiegand_line_t line, bool rising)
{
    if (s_current_dir != WIEGAND_DIR_RX)
    {
        return;
    }

    /*
     * D0 (EXTI0) and D1 (EXTI1) are configured at the same preemption priority,
     * so neither ISR can preempt the other. Disabling IRQ is not required.
     */
    (void)Wiegand_PushEdge(&s_wiegand, line, rising, now_us());
}

void Wiegand_Port_OnExtiRising(uint16_t pin)
{
    if (pin == WIEGAND_D0_PIN)      { on_edge(WIEGAND_LINE_D0, true);  }
    else if (pin == WIEGAND_D1_PIN) { on_edge(WIEGAND_LINE_D1, true);  }
}

void Wiegand_Port_OnExtiFalling(uint16_t pin)
{
    if (pin == WIEGAND_D0_PIN)      { on_edge(WIEGAND_LINE_D0, false); }
    else if (pin == WIEGAND_D1_PIN) { on_edge(WIEGAND_LINE_D1, false); }
}

void Wiegand_Port_EXTI_IRQHandler(uint16_t pin)
{
    if (s_current_dir != WIEGAND_DIR_RX)
    {
        __HAL_GPIO_EXTI_CLEAR_RISING_IT(pin);
        __HAL_GPIO_EXTI_CLEAR_FALLING_IT(pin);
        return;
    }

    wiegand_line_t line = (pin == WIEGAND_D1_PIN) ? WIEGAND_LINE_D1 : WIEGAND_LINE_D0;

    /*
     * For active-low Wiegand pulses (idle high), a falling edge arrives first,
     * then a rising edge. If both flags are pending, handle fall first so
     * pulse ordering is preserved.
     */
    if (__HAL_GPIO_EXTI_GET_FALLING_IT(pin) != 0U)
    {
        __HAL_GPIO_EXTI_CLEAR_FALLING_IT(pin);
        on_edge(line, false);
    }
    if (__HAL_GPIO_EXTI_GET_RISING_IT(pin) != 0U)
    {
        __HAL_GPIO_EXTI_CLEAR_RISING_IT(pin);
        on_edge(line, true);
    }
}

/**
  * @brief Interrupt handler for EXTI Line0 (WIE_D0 on PF0).
  * Overrides weak symbol from startup_stm32h563xx.s without modifying stm32h5xx_it.c.
  */
void EXTI0_IRQHandler(void)
{
    Wiegand_Port_EXTI_IRQHandler(WIEGAND_D0_PIN);
}

/**
  * @brief Interrupt handler for EXTI Line1 (WIE_D1 on PF1).
  * Overrides weak symbol from startup_stm32h563xx.s without modifying stm32h5xx_it.c.
  */
void EXTI1_IRQHandler(void)
{
    Wiegand_Port_EXTI_IRQHandler(WIEGAND_D1_PIN);
}

/* Weak callbacks override for HAL EXTI */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t pin)
{
    Wiegand_Port_OnExtiRising(pin);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t pin)
{
    Wiegand_Port_OnExtiFalling(pin);
}

void Wiegand_Port_SetDirection(wiegand_dir_t dir)
{
    __HAL_RCC_GPIOF_CLK_ENABLE();

    if (dir == WIEGAND_DIR_RX)
    {
        /* Disable EXTI during reconfiguration */
        HAL_NVIC_DisableIRQ(EXTI0_IRQn);
        HAL_NVIC_DisableIRQ(EXTI1_IRQn);

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = WIEGAND_D0_PIN | WIEGAND_D1_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(WIEGAND_D0_PORT, &GPIO_InitStruct);

        /* Clear pending flags */
        __HAL_GPIO_EXTI_CLEAR_RISING_IT(WIEGAND_D0_PIN | WIEGAND_D1_PIN);
        __HAL_GPIO_EXTI_CLEAR_FALLING_IT(WIEGAND_D0_PIN | WIEGAND_D1_PIN);
        HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
        HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);

        /* Enable interrupts with highest priority */
        HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);
        HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(EXTI1_IRQn);

        s_current_dir = WIEGAND_DIR_RX;

        if (s_initialized)
        {
            Wiegand_Reset(&s_wiegand, now_us(), Wiegand_Port_ReadLevels());
        }
    }
    else /* WIEGAND_DIR_TX */
    {
        /* Disable EXTI interrupts so our transmissions don't trigger false edges */
        HAL_NVIC_DisableIRQ(EXTI0_IRQn);
        HAL_NVIC_DisableIRQ(EXTI1_IRQn);
        __HAL_GPIO_EXTI_CLEAR_RISING_IT(WIEGAND_D0_PIN | WIEGAND_D1_PIN);
        __HAL_GPIO_EXTI_CLEAR_FALLING_IT(WIEGAND_D0_PIN | WIEGAND_D1_PIN);
        HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
        HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);

        /* Pre-set pins HIGH before switching mode to prevent glitching */
        HAL_GPIO_WritePin(WIEGAND_D0_PORT, WIEGAND_D0_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(WIEGAND_D1_PORT, WIEGAND_D1_PIN, GPIO_PIN_SET);

        /* Configure as Open Drain with Pull-up (SIA standard: open-collector) */
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = WIEGAND_D0_PIN | WIEGAND_D1_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(WIEGAND_D0_PORT, &GPIO_InitStruct);

        /* Re-affirm idle HIGH */
        HAL_GPIO_WritePin(WIEGAND_D0_PORT, WIEGAND_D0_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(WIEGAND_D1_PORT, WIEGAND_D1_PIN, GPIO_PIN_SET);

        s_current_dir = WIEGAND_DIR_TX;
    }
}

wiegand_dir_t Wiegand_Port_GetDirection(void)
{
    return s_current_dir;
}

const char *Wiegand_Port_GetDirectionStr(void)
{
    return (s_current_dir == WIEGAND_DIR_RX) ? "RX (Receiver)" : "TX (Transmitter)";
}

bool Wiegand_Port_Init(const wiegand_config_t *cfg, wiegand_dir_t initial_dir)
{
    /* Enable TIM2 clock */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* Determine TIM2 input clock frequency from RCC */
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    RCC_ClkInitTypeDef clk;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clk, &latency);
    uint32_t tim_clock = (clk.APB1CLKDivider == RCC_HCLK_DIV1) ? pclk1 : (pclk1 * 2U);

    /* 1 tick = 1 microsecond (1 MHz) */
    uint32_t prescaler = (tim_clock / 1000000U) - 1U;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = prescaler;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFFU;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        return false;
    }
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
    {
        return false;
    }

    /* Configure GPIO for requested initial direction */
    Wiegand_Port_SetDirection(initial_dir);

    const bool ok = Wiegand_Init(&s_wiegand, cfg, now_us(), Wiegand_Port_ReadLevels());

    s_initialized = ok;
    return ok;
}

void Wiegand_Port_Process(void)
{
    if (s_current_dir != WIEGAND_DIR_RX)
    {
        return;
    }

    const uint32_t t  = now_us();
    const uint8_t  lv = Wiegand_Port_ReadLevels();

    Wiegand_Process(&s_wiegand, t, lv);
}

void Wiegand_Port_Reset(void)
{
    Wiegand_Reset(&s_wiegand, now_us(), Wiegand_Port_ReadLevels());
}

void Wiegand_Port_GetStats(wiegand_stats_t *out)
{
    Wiegand_GetStats(&s_wiegand, out);
}

bool Wiegand_Port_TransmitRaw(const uint8_t *bits, uint16_t bit_count,
                              uint32_t pulse_width_us, uint32_t pulse_gap_us)
{
    if ((bits == NULL) || (bit_count == 0U))
    {
        return false;
    }

    if (pulse_width_us == 0U) { pulse_width_us = 50U; }
    if (pulse_gap_us == 0U)   { pulse_gap_us = 2000U; }

    wiegand_dir_t prev_dir = s_current_dir;
    if (prev_dir != WIEGAND_DIR_TX)
    {
        Wiegand_Port_SetDirection(WIEGAND_DIR_TX);
    }

    /* Ensure line is quiescent HIGH before first bit */
    HAL_GPIO_WritePin(WIEGAND_D0_PORT, WIEGAND_D0_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WIEGAND_D1_PORT, WIEGAND_D1_PIN, GPIO_PIN_SET);
    Wiegand_Port_DelayUs(1000U);

    for (uint16_t i = 0U; i < bit_count; i++)
    {
        bool is_one = (bits[i >> 3] & (0x80U >> (i & 7U))) != 0U;

        if (is_one)
        {
            HAL_GPIO_WritePin(WIEGAND_D1_PORT, WIEGAND_D1_PIN, GPIO_PIN_RESET);
            Wiegand_Port_DelayUs(pulse_width_us);
            HAL_GPIO_WritePin(WIEGAND_D1_PORT, WIEGAND_D1_PIN, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(WIEGAND_D0_PORT, WIEGAND_D0_PIN, GPIO_PIN_RESET);
            Wiegand_Port_DelayUs(pulse_width_us);
            HAL_GPIO_WritePin(WIEGAND_D0_PORT, WIEGAND_D0_PIN, GPIO_PIN_SET);
        }

        if (i < (bit_count - 1U))
        {
            Wiegand_Port_DelayUs(pulse_gap_us);
        }
    }

    /* Inter-frame silence gap */
    Wiegand_Port_DelayUs(25000U);

    if (prev_dir != WIEGAND_DIR_TX)
    {
        Wiegand_Port_SetDirection(prev_dir);
    }

    return true;
}

static unsigned count_ones_u32(uint32_t val)
{
    unsigned count = 0;
    while (val)
    {
        count += (val & 1U);
        val >>= 1U;
    }
    return count;
}

bool Wiegand_Port_Transmit26(uint8_t facility, uint16_t card)
{
    /*
     * SIA 26-bit structure:
     *   Bit 0      : Even parity over bits 1..12
     *   Bits 1..8  : Facility code (8 bits)
     *   Bits 9..24 : Card number (16 bits)
     *   Bit 25     : Odd parity over bits 13..24
     */
    uint32_t data24 = (((uint32_t)facility) << 16) | ((uint32_t)card);

    /* Bits 1..12 are bits 23..12 of data24 */
    uint32_t upper12 = (data24 >> 12) & 0x0FFFU;
    /* Bits 13..24 are bits 11..0 of data24 */
    uint32_t lower12 = data24 & 0x0FFFU;

    /* P1: even parity over 12 bits -> 1 if odd number of ones */
    uint8_t p1 = (count_ones_u32(upper12) & 1U) ? 1U : 0U;

    /* P2: odd parity over 12 bits -> 1 if even number of ones */
    uint8_t p2 = (count_ones_u32(lower12) & 1U) ? 0U : 1U;

    uint32_t frame26 = (((uint32_t)p1) << 25) | (data24 << 1) | ((uint32_t)p2);

    /* Align so bit 0 is at bits[0] MSB (bit 7) */
    uint8_t bits[4];
    bits[0] = (uint8_t)(frame26 >> 18);
    bits[1] = (uint8_t)(frame26 >> 10);
    bits[2] = (uint8_t)(frame26 >> 2);
    bits[3] = (uint8_t)((frame26 & 0x03U) << 6);

    return Wiegand_Port_TransmitRaw(bits, 26U, 50U, 2000U);
}

bool Wiegand_Port_Transmit34(uint16_t facility, uint16_t card)
{
    /*
     * 34-bit structure:
     *   Bit 0      : Even parity over bits 1..16
     *   Bits 1..16 : Facility code (16 bits)
     *   Bits 17..32: Card number (16 bits)
     *   Bit 33     : Odd parity over bits 17..32
     */
    uint8_t p1 = (count_ones_u32(facility) & 1U) ? 1U : 0U;
    uint8_t p2 = (count_ones_u32(card) & 1U) ? 0U : 1U;

    uint8_t bits[5] = {0};

    /* Bit 0 */
    if (p1) { bits[0] |= 0x80U; }

    /* Bits 1..16: facility */
    for (int i = 0; i < 16; i++)
    {
        if ((facility >> (15 - i)) & 1U)
        {
            uint16_t bit_idx = (uint16_t)(1 + i);
            bits[bit_idx >> 3] |= (uint8_t)(0x80U >> (bit_idx & 7U));
        }
    }

    /* Bits 17..32: card */
    for (int i = 0; i < 16; i++)
    {
        if ((card >> (15 - i)) & 1U)
        {
            uint16_t bit_idx = (uint16_t)(17 + i);
            bits[bit_idx >> 3] |= (uint8_t)(0x80U >> (bit_idx & 7U));
        }
    }

    /* Bit 33: p2 */
    if (p2)
    {
        bits[33 >> 3] |= (uint8_t)(0x80U >> (33 & 7U));
    }

    return Wiegand_Port_TransmitRaw(bits, 34U, 50U, 2000U);
}

static int hex_char_to_val(char c)
{
    if (c >= '0' && c <= '9') { return c - '0'; }
    if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
    if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
    return -1;
}

bool Wiegand_Port_TransmitHex(uint16_t bit_count, const char *hex_str)
{
    if (hex_str == NULL || bit_count == 0 || bit_count > WIEGAND_MAX_BITS)
    {
        return false;
    }

    if (hex_str[0] == '0' && (hex_str[1] == 'x' || hex_str[1] == 'X'))
    {
        hex_str += 2;
    }

    size_t hex_len = strlen(hex_str);
    if (hex_len == 0)
    {
        return false;
    }

    uint8_t bits[WIEGAND_MAX_BITS / 8U] = {0};
    uint16_t total_nibbles = (bit_count + 3U) / 4U;

    /* Left-align bits from hex string */
    for (uint16_t i = 0; i < total_nibbles && i < hex_len; i++)
    {
        int val = hex_char_to_val(hex_str[i]);
        if (val < 0) { return false; }

        if ((i & 1U) == 0U)
        {
            bits[i / 2U] |= (uint8_t)(val << 4);
        }
        else
        {
            bits[i / 2U] |= (uint8_t)(val & 0x0FU);
        }
    }

    return Wiegand_Port_TransmitRaw(bits, bit_count, 50U, 2000U);
}

bool Wiegand_Port_Decode26(const wiegand_frame_t *frame, wiegand_card26_t *out)
{
    if (frame == NULL || out == NULL || frame->bit_count != 26U)
    {
        return false;
    }

    out->bit_count = frame->bit_count;

    uint32_t val = 0;
    for (uint16_t i = 0; i < 26; i++)
    {
        val = (val << 1) | (Wiegand_GetBit(frame, i) ? 1U : 0U);
    }

    uint8_t  p1 = (uint8_t)((val >> 25) & 1U);
    uint32_t fac = (val >> 17) & 0xFFU;
    uint32_t crd = (val >> 1)  & 0xFFFFU;
    uint8_t  p2 = (uint8_t)(val & 1U);

    out->facility_code = (uint8_t)fac;
    out->card_number   = (uint16_t)crd;
    out->parity_valid  = Wiegand_ValidateParity26(frame, NULL);

    (void)p1;
    (void)p2;
    return true;
}

const char *Wiegand_StatusToString(wiegand_status_t status)
{
    switch (status)
    {
        case WIEGAND_STATUS_OK:               return "OK";
        case WIEGAND_STATUS_PULSE_TOO_SHORT:  return "Pulse too short (filtered runt)";
        case WIEGAND_STATUS_PULSE_TOO_LONG:   return "Pulse too long";
        case WIEGAND_STATUS_GAP_TOO_SHORT:    return "Gap too short (Tpi fault)";
        case WIEGAND_STATUS_OVERLAP:          return "D0/D1 pulse overlap";
        case WIEGAND_STATUS_STUCK_LOW:        return "Line stuck low";
        case WIEGAND_STATUS_EDGE_SEQUENCE:    return "Edge sequence error";
        case WIEGAND_STATUS_QUEUE_OVERFLOW:   return "Queue overflow";
        case WIEGAND_STATUS_TOO_MANY_BITS:    return "Too many bits";
        case WIEGAND_STATUS_BIT_COUNT:        return "Invalid bit count";
        case WIEGAND_STATUS_PARITY:           return "Parity check failed";
        default:                              return "Unknown status";
    }
}
