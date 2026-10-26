// SPDX-License-Identifier: MIT
/*
 * wiegand_port_stm32.c - STM32H5 (HAL) bidirectional port for wiegand.h
 */

#include "wiegand_port_stm32.h"
#include <string.h>
#include <stdio.h>

/* Board pin configuration */
#ifndef WIEGAND_D0_PORT
#define WIEGAND_D0_PORT		WIE_D0_GPIO_Port
#define WIEGAND_D0_PIN		WIE_D0_Pin
#endif
#ifndef WIEGAND_D1_PORT
#define WIEGAND_D1_PORT		WIE_D1_GPIO_Port
#define WIEGAND_D1_PIN		WIE_D1_Pin
#endif

TIM_HandleTypeDef htim2;

static wiegand_t	s_wiegand;
static wiegand_dir_t	s_current_dir = WIEGAND_DIR_RX;
static bool		s_initialized;

static inline uint32_t now_us(void)
{
	return __HAL_TIM_GET_COUNTER(&htim2);
}

uint32_t wiegand_port_get_now_us(void)
{
	return now_us();
}

void wiegand_port_delay_us(uint32_t us)
{
	uint32_t start = now_us();

	while ((now_us() - start) < us)
		__NOP();
}

uint8_t wiegand_port_read_levels(void)
{
	uint8_t lv = 0U;

	if (HAL_GPIO_ReadPin(WIEGAND_D0_PORT, WIEGAND_D0_PIN) == GPIO_PIN_SET)
		lv |= WIEGAND_LEVEL_D0;
	if (HAL_GPIO_ReadPin(WIEGAND_D1_PORT, WIEGAND_D1_PIN) == GPIO_PIN_SET)
		lv |= WIEGAND_LEVEL_D1;

	return lv;
}

static void on_edge(wiegand_line_t line, bool rising)
{
	if (s_current_dir != WIEGAND_DIR_RX)
		return;

	/*
	 * D0 (EXTI0) and D1 (EXTI1) are configured at the same preemption priority,
	 * so neither ISR can preempt the other. Disabling IRQ is not required.
	 */
	(void)wiegand_push_edge(&s_wiegand, line, rising, now_us());
}

void wiegand_port_on_exti_rising(uint16_t pin)
{
	if (pin == WIEGAND_D0_PIN)
		on_edge(WIEGAND_LINE_D0, true);
	else if (pin == WIEGAND_D1_PIN)
		on_edge(WIEGAND_LINE_D1, true);
}

void wiegand_port_on_exti_falling(uint16_t pin)
{
	if (pin == WIEGAND_D0_PIN)
		on_edge(WIEGAND_LINE_D0, false);
	else if (pin == WIEGAND_D1_PIN)
		on_edge(WIEGAND_LINE_D1, false);
}

void wiegand_port_exti_irq_handler(uint16_t pin)
{
	wiegand_line_t line;

	if (s_current_dir != WIEGAND_DIR_RX) {
		__HAL_GPIO_EXTI_CLEAR_RISING_IT(pin);
		__HAL_GPIO_EXTI_CLEAR_FALLING_IT(pin);
		return;
	}

	line = (pin == WIEGAND_D1_PIN) ? WIEGAND_LINE_D1 : WIEGAND_LINE_D0;

	/*
	 * For active-low Wiegand pulses (idle high), a falling edge arrives first,
	 * then a rising edge. If both flags are pending, handle fall first so
	 * pulse ordering is preserved.
	 */
	if (__HAL_GPIO_EXTI_GET_FALLING_IT(pin)) {
		__HAL_GPIO_EXTI_CLEAR_FALLING_IT(pin);
		on_edge(line, false);
	}
	if (__HAL_GPIO_EXTI_GET_RISING_IT(pin)) {
		__HAL_GPIO_EXTI_CLEAR_RISING_IT(pin);
		on_edge(line, true);
	}
}

/**
 * EXTI0_IRQHandler - Handles EXTI Line0 for WIE_D0 (PF0).
 * Overrides weak symbol from startup without editing stm32h5xx_it.c.
 */
void EXTI0_IRQHandler(void)
{
	wiegand_port_exti_irq_handler(WIEGAND_D0_PIN);
}

/**
 * EXTI1_IRQHandler - Handles EXTI Line1 for WIE_D1 (PF1).
 * Overrides weak symbol from startup without editing stm32h5xx_it.c.
 */
void EXTI1_IRQHandler(void)
{
	wiegand_port_exti_irq_handler(WIEGAND_D1_PIN);
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t pin)
{
	wiegand_port_on_exti_rising(pin);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t pin)
{
	wiegand_port_on_exti_falling(pin);
}

void wiegand_port_set_direction(wiegand_dir_t dir)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOF_CLK_ENABLE();

	if (dir == WIEGAND_DIR_RX) {
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		HAL_NVIC_DisableIRQ(EXTI1_IRQn);

		gpio.Pin = WIEGAND_D0_PIN | WIEGAND_D1_PIN;
		gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
		gpio.Pull = GPIO_PULLUP;
		gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(WIEGAND_D0_PORT, &gpio);

		__HAL_GPIO_EXTI_CLEAR_RISING_IT(WIEGAND_D0_PIN | WIEGAND_D1_PIN);
		__HAL_GPIO_EXTI_CLEAR_FALLING_IT(WIEGAND_D0_PIN | WIEGAND_D1_PIN);
		HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
		HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);

		HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI1_IRQn);

		s_current_dir = WIEGAND_DIR_RX;

		if (s_initialized)
			wiegand_reset(&s_wiegand, now_us(), wiegand_port_read_levels());
	} else {
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		HAL_NVIC_DisableIRQ(EXTI1_IRQn);
		__HAL_GPIO_EXTI_CLEAR_RISING_IT(WIEGAND_D0_PIN | WIEGAND_D1_PIN);
		__HAL_GPIO_EXTI_CLEAR_FALLING_IT(WIEGAND_D0_PIN | WIEGAND_D1_PIN);
		HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
		HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);

		HAL_GPIO_WritePin(WIEGAND_D0_PORT, WIEGAND_D0_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(WIEGAND_D1_PORT, WIEGAND_D1_PIN, GPIO_PIN_SET);

		gpio.Pin = WIEGAND_D0_PIN | WIEGAND_D1_PIN;
		gpio.Mode = GPIO_MODE_OUTPUT_OD;
		gpio.Pull = GPIO_PULLUP;
		gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(WIEGAND_D0_PORT, &gpio);

		HAL_GPIO_WritePin(WIEGAND_D0_PORT, WIEGAND_D0_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(WIEGAND_D1_PORT, WIEGAND_D1_PIN, GPIO_PIN_SET);

		s_current_dir = WIEGAND_DIR_TX;
	}
}

wiegand_dir_t wiegand_port_get_direction(void)
{
	return s_current_dir;
}

const char *wiegand_port_get_direction_str(void)
{
	return (s_current_dir == WIEGAND_DIR_RX) ? "RX (Receiver)" : "TX (Transmitter)";
}

bool wiegand_port_init(const wiegand_config_t *cfg, wiegand_dir_t initial_dir)
{
	RCC_ClkInitTypeDef clk;
	uint32_t latency;
	uint32_t pclk1;
	uint32_t tim_clock;
	uint32_t prescaler;
	bool ok;

	__HAL_RCC_TIM2_CLK_ENABLE();

	pclk1 = HAL_RCC_GetPCLK1Freq();
	HAL_RCC_GetClockConfig(&clk, &latency);
	tim_clock = (clk.APB1CLKDivider == RCC_HCLK_DIV1) ? pclk1 : (pclk1 * 2U);
	prescaler = (tim_clock / 1000000U) - 1U;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = prescaler;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0xFFFFFFFFU;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
		return false;
	if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
		return false;

	wiegand_port_set_direction(initial_dir);

	ok = wiegand_init(&s_wiegand, cfg, now_us(), wiegand_port_read_levels());
	s_initialized = ok;

	return ok;
}

void wiegand_port_process(void)
{
	uint32_t t;
	uint8_t lv;

	if (s_current_dir != WIEGAND_DIR_RX)
		return;

	t = now_us();
	lv = wiegand_port_read_levels();

	wiegand_process(&s_wiegand, t, lv);
}

void wiegand_port_reset(void)
{
	wiegand_reset(&s_wiegand, now_us(), wiegand_port_read_levels());
}

void wiegand_port_get_stats(wiegand_stats_t *out)
{
	wiegand_get_stats(&s_wiegand, out);
}

bool wiegand_port_transmit_raw(const uint8_t *bits, uint16_t bit_count,
			       uint32_t pulse_width_us, uint32_t pulse_gap_us)
{
	wiegand_dir_t prev_dir;
	uint16_t i;

	if (!bits || bit_count == 0U)
		return false;

	if (pulse_width_us == 0U)
		pulse_width_us = 50U;
	if (pulse_gap_us == 0U)
		pulse_gap_us = 2000U;

	prev_dir = s_current_dir;
	if (prev_dir != WIEGAND_DIR_TX)
		wiegand_port_set_direction(WIEGAND_DIR_TX);

	HAL_GPIO_WritePin(WIEGAND_D0_PORT, WIEGAND_D0_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(WIEGAND_D1_PORT, WIEGAND_D1_PIN, GPIO_PIN_SET);
	wiegand_port_delay_us(1000U);

	for (i = 0U; i < bit_count; i++) {
		bool is_one = (bits[i >> 3] & (0x80U >> (i & 7U))) != 0U;

		if (is_one) {
			HAL_GPIO_WritePin(WIEGAND_D1_PORT, WIEGAND_D1_PIN, GPIO_PIN_RESET);
			wiegand_port_delay_us(pulse_width_us);
			HAL_GPIO_WritePin(WIEGAND_D1_PORT, WIEGAND_D1_PIN, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(WIEGAND_D0_PORT, WIEGAND_D0_PIN, GPIO_PIN_RESET);
			wiegand_port_delay_us(pulse_width_us);
			HAL_GPIO_WritePin(WIEGAND_D0_PORT, WIEGAND_D0_PIN, GPIO_PIN_SET);
		}

		if (i < (bit_count - 1U))
			wiegand_port_delay_us(pulse_gap_us);
	}

	wiegand_port_delay_us(25000U);

	if (prev_dir != WIEGAND_DIR_TX)
		wiegand_port_set_direction(prev_dir);

	return true;
}

static unsigned int count_ones_u32(uint32_t val)
{
	unsigned int count = 0;

	while (val) {
		count += (val & 1U);
		val >>= 1U;
	}
	return count;
}

bool wiegand_port_transmit26(uint8_t facility, uint16_t card)
{
	uint32_t data24 = (((uint32_t)facility) << 16) | ((uint32_t)card);
	uint32_t upper12 = (data24 >> 12) & 0x0FFFU;
	uint32_t lower12 = data24 & 0x0FFFU;
	uint8_t p1 = (count_ones_u32(upper12) & 1U) ? 1U : 0U;
	uint8_t p2 = (count_ones_u32(lower12) & 1U) ? 0U : 1U;
	uint32_t frame26 = (((uint32_t)p1) << 25) | (data24 << 1) | ((uint32_t)p2);
	uint8_t bits[4];

	bits[0] = (uint8_t)(frame26 >> 18);
	bits[1] = (uint8_t)(frame26 >> 10);
	bits[2] = (uint8_t)(frame26 >> 2);
	bits[3] = (uint8_t)((frame26 & 0x03U) << 6);

	return wiegand_port_transmit_raw(bits, 26U, 50U, 2000U);
}

bool wiegand_port_transmit34(uint16_t facility, uint16_t card)
{
	uint8_t p1 = (count_ones_u32(facility) & 1U) ? 1U : 0U;
	uint8_t p2 = (count_ones_u32(card) & 1U) ? 0U : 1U;
	uint8_t bits[5] = {0};
	int i;

	if (p1)
		bits[0] |= 0x80U;

	for (i = 0; i < 16; i++) {
		if ((facility >> (15 - i)) & 1U) {
			uint16_t bit_idx = (uint16_t)(1 + i);

			bits[bit_idx >> 3] |= (uint8_t)(0x80U >> (bit_idx & 7U));
		}
	}

	for (i = 0; i < 16; i++) {
		if ((card >> (15 - i)) & 1U) {
			uint16_t bit_idx = (uint16_t)(17 + i);

			bits[bit_idx >> 3] |= (uint8_t)(0x80U >> (bit_idx & 7U));
		}
	}

	if (p2)
		bits[33 >> 3] |= (uint8_t)(0x80U >> (33 & 7U));

	return wiegand_port_transmit_raw(bits, 34U, 50U, 2000U);
}

static int hex_char_to_val(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

bool wiegand_port_transmit_hex(uint16_t bit_count, const char *hex_str)
{
	uint8_t bits[WIEGAND_MAX_BITS / 8U] = {0};
	uint16_t total_nibbles;
	uint16_t pad_nibbles;
	size_t hex_len;
	uint16_t i;

	if (!hex_str || bit_count == 0 || bit_count > WIEGAND_MAX_BITS)
		return false;

	if (hex_str[0] == '0' && (hex_str[1] == 'x' || hex_str[1] == 'X'))
		hex_str += 2;

	hex_len = strlen(hex_str);
	if (hex_len == 0)
		return false;

	total_nibbles = (bit_count + 3U) / 4U;

	/* Strip excess leading zeros if hex string is longer than required nibbles */
	while (hex_len > total_nibbles && *hex_str == '0') {
		hex_str++;
		hex_len--;
	}

	/* Right-align if fewer nibbles provided than total_nibbles */
	pad_nibbles = (hex_len < total_nibbles) ? (total_nibbles - (uint16_t)hex_len) : 0U;

	for (i = 0; i < total_nibbles; i++) {
		int val = 0;

		if (i >= pad_nibbles) {
			uint16_t src_idx = i - pad_nibbles;

			if (src_idx < hex_len) {
				val = hex_char_to_val(hex_str[src_idx]);
				if (val < 0)
					return false;
			}
		}

		if ((i & 1U) == 0U)
			bits[i / 2U] |= (uint8_t)(val << 4);
		else
			bits[i / 2U] |= (uint8_t)(val & 0x0FU);
	}

	return wiegand_port_transmit_raw(bits, bit_count, 50U, 2000U);
}

bool wiegand_port_decode26(const wiegand_frame_t *frame, wiegand_card26_t *out)
{
	uint32_t val = 0;
	uint16_t i;

	if (!frame || !out || frame->bit_count != 26U)
		return false;

	out->bit_count = frame->bit_count;

	for (i = 0; i < 26; i++)
		val = (val << 1) | (wiegand_get_bit(frame, i) ? 1U : 0U);

	out->facility_code = (uint8_t)((val >> 17) & 0xFFU);
	out->card_number = (uint16_t)((val >> 1) & 0xFFFFU);
	out->parity_valid = wiegand_validate_parity26(frame, NULL);

	return true;
}

const char *wiegand_status_to_string(wiegand_status_t status)
{
	switch (status) {
	case WIEGAND_STATUS_OK:
		return "OK";
	case WIEGAND_STATUS_PULSE_TOO_SHORT:
		return "Pulse too short (filtered runt)";
	case WIEGAND_STATUS_PULSE_TOO_LONG:
		return "Pulse too long";
	case WIEGAND_STATUS_GAP_TOO_SHORT:
		return "Gap too short (Tpi fault)";
	case WIEGAND_STATUS_OVERLAP:
		return "D0/D1 pulse overlap";
	case WIEGAND_STATUS_STUCK_LOW:
		return "Line stuck low";
	case WIEGAND_STATUS_EDGE_SEQUENCE:
		return "Edge sequence error";
	case WIEGAND_STATUS_QUEUE_OVERFLOW:
		return "Queue overflow";
	case WIEGAND_STATUS_TOO_MANY_BITS:
		return "Too many bits";
	case WIEGAND_STATUS_BIT_COUNT:
		return "Invalid bit count";
	case WIEGAND_STATUS_PARITY:
		return "Parity check failed";
	default:
		return "Unknown status";
	}
}
