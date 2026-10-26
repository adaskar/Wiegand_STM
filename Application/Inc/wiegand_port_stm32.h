/* SPDX-License-Identifier: MIT */
/*
 * wiegand_port_stm32.h - STM32H5 (HAL) bidirectional port for wiegand.h
 *
 * Supports both RX (Wiegand Reader / Receiver) and TX (Wiegand Controller / Transmitter).
 *
 * Hardware connections:
 *   - WIE_D0: PF0 (EXTI0 in RX mode, GPIO Output Open-Drain with pull-up in TX mode)
 *   - WIE_D1: PF1 (EXTI1 in RX mode, GPIO Output Open-Drain with pull-up in TX mode)
 *   - Timestamp Timer: TIM2 (32-bit free-running at 1 MHz, 1 us per tick)
 */

#ifndef WIEGAND_PORT_STM32_H
#define WIEGAND_PORT_STM32_H

#include "wiegand.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	WIEGAND_DIR_RX = 0,	/* Listening/Receiver mode (Input, EXTI) */
	WIEGAND_DIR_TX = 1,	/* Transmitter mode (Output OD, idle HIGH) */
} wiegand_dir_t;

typedef struct {
	uint16_t	bit_count;
	uint8_t		facility_code;
	uint16_t	card_number;
	bool		parity_valid;
} wiegand_card26_t;

/* Initialize timer (TIM2, 32-bit 1 MHz), GPIO, and receiver core */
bool wiegand_port_init(const wiegand_config_t *cfg, wiegand_dir_t initial_dir);

/* Main loop service function. Drains edges, checks timeouts, triggers callbacks */
void wiegand_port_process(void);

/* Drop everything in flight and resynchronize receiver */
void wiegand_port_reset(void);

/* Retrieve cumulative statistics */
void wiegand_port_get_stats(wiegand_stats_t *out);

/* Set port direction (WIEGAND_DIR_RX or WIEGAND_DIR_TX) */
void wiegand_port_set_direction(wiegand_dir_t dir);

/* Get current port direction */
wiegand_dir_t wiegand_port_get_direction(void);
const char *wiegand_port_get_direction_str(void);

/* ISR entry points called from EXTI ISR or HAL EXTI callbacks */
void wiegand_port_on_exti_rising(uint16_t pin);
void wiegand_port_on_exti_falling(uint16_t pin);
void wiegand_port_exti_irq_handler(uint16_t pin);

/* Transmit APIs */
bool wiegand_port_transmit_raw(const uint8_t *bits, uint16_t bit_count,
			       uint32_t pulse_width_us, uint32_t pulse_gap_us);
bool wiegand_port_transmit26(uint8_t facility, uint16_t card);
bool wiegand_port_transmit34(uint16_t facility, uint16_t card);
bool wiegand_port_transmit_hex(uint16_t bit_count, const char *hex_str);

/* Microsecond delay using 32-bit hardware timer */
void wiegand_port_delay_us(uint32_t us);

/* Read current pin levels: bitmask of WIEGAND_LEVEL_D0 / WIEGAND_LEVEL_D1 */
uint8_t wiegand_port_read_levels(void);

/* Current timestamp in microseconds */
uint32_t wiegand_port_get_now_us(void);

/* Helper to decode standard 26-bit SIA frame */
bool wiegand_port_decode26(const wiegand_frame_t *frame, wiegand_card26_t *out);

/* Convert error status to human-readable string */
const char *wiegand_status_to_string(wiegand_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* WIEGAND_PORT_STM32_H */
