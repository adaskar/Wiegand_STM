#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"
#include <stdio.h>
#include <main.h>

void main_app(void)
{
    uint8_t rx_byte;

    while (1)
    {
        /* Block until a byte is received over COM1 */
        if (HAL_UART_Receive(&hcom_uart[COM1], &rx_byte, 1, HAL_MAX_DELAY) == HAL_OK)
        {
            /* Echo received byte back to the sender */
            HAL_UART_Transmit(&hcom_uart[COM1], &rx_byte, 1, HAL_MAX_DELAY);
            BSP_LED_Toggle(LED_GREEN);
        }
        else
        {
            /* Clear potential UART error flags (overrun, noise, framing) to avoid lockup */
            __HAL_UART_CLEAR_OREFLAG(&hcom_uart[COM1]);
            __HAL_UART_CLEAR_FLAG(&hcom_uart[COM1], UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
            hcom_uart[COM1].ErrorCode = HAL_UART_ERROR_NONE;
            hcom_uart[COM1].RxState = HAL_UART_STATE_READY;
        }
    }
}
