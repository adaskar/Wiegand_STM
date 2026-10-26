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

typedef enum
{
    WIEGAND_DIR_RX = 0,     /* Listening/Receiver mode (Input, EXTI interrupt) */
    WIEGAND_DIR_TX = 1      /* Transmitter mode (Output OD, idle HIGH) */
} wiegand_dir_t;

typedef struct
{
    uint16_t bit_count;
    uint8_t  facility_code;
    uint16_t card_number;
    bool     parity_valid;
} wiegand_card26_t;

/* Initialize timer (TIM2, 32-bit 1 MHz), GPIO, and receiver core. */
bool Wiegand_Port_Init(const wiegand_config_t *cfg, wiegand_dir_t initial_dir);

/* Main loop service function. Drains edges, checks timeouts, triggers callbacks. */
void Wiegand_Port_Process(void);

/* Drop everything in flight and resynchronize receiver. */
void Wiegand_Port_Reset(void);

/* Retrieve cumulative statistics. */
void Wiegand_Port_GetStats(wiegand_stats_t *out);

/* Set port direction (WIEGAND_DIR_RX or WIEGAND_DIR_TX) with proper GPIO initialization. */
void Wiegand_Port_SetDirection(wiegand_dir_t dir);

/* Get current port direction. */
wiegand_dir_t Wiegand_Port_GetDirection(void);
const char *Wiegand_Port_GetDirectionStr(void);

/* ISR entry points called from EXTI ISR or HAL EXTI callbacks. */
void Wiegand_Port_OnExtiRising(uint16_t pin);
void Wiegand_Port_OnExtiFalling(uint16_t pin);
void Wiegand_Port_EXTI_IRQHandler(uint16_t pin);

/* Transmit APIs */
bool Wiegand_Port_TransmitRaw(const uint8_t *bits, uint16_t bit_count,
                              uint32_t pulse_width_us, uint32_t pulse_gap_us);
bool Wiegand_Port_Transmit26(uint8_t facility, uint16_t card);
bool Wiegand_Port_Transmit34(uint16_t facility, uint16_t card);
bool Wiegand_Port_TransmitHex(uint16_t bit_count, const char *hex_str);

/* Microsecond delay using 32-bit hardware timer */
void Wiegand_Port_DelayUs(uint32_t us);

/* Read current pin levels: returns bitmask of WIEGAND_LEVEL_D0 / WIEGAND_LEVEL_D1 */
uint8_t Wiegand_Port_ReadLevels(void);

/* Current timestamp in microseconds */
uint32_t Wiegand_Port_GetNowUs(void);

/* Helper to decode standard 26-bit SIA frame */
bool Wiegand_Port_Decode26(const wiegand_frame_t *frame, wiegand_card26_t *out);

/* Convert error status to human-readable string */
const char *Wiegand_StatusToString(wiegand_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* WIEGAND_PORT_STM32_H */

