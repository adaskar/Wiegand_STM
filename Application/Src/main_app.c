/*
 * main_app.c - Bidirectional Wiegand application with UART command listener & debug channel
 */
#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"
#include "main.h"
#include "wiegand.h"
#include "wiegand_port_stm32.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#define CMD_BUFFER_SIZE  128U

static char s_cmd_buf[CMD_BUFFER_SIZE];
static uint16_t s_cmd_len = 0U;

/* -------------------------------------------------------------------------- */
/* Wiegand Event Callbacks                                                    */
/* -------------------------------------------------------------------------- */

static void on_wiegand_frame(void *ctx, const wiegand_frame_t *frame)
{
    (void)ctx;
    BSP_LED_On(LED_GREEN);

    printf("\r\n================ [WIEGAND RX FRAME] ================\r\n");
    printf("  Bit Count     : %u bits\r\n", frame->bit_count);
    printf("  Duration      : %lu us (Tfirst: %lu, Tlast: %lu)\r\n",
           (unsigned long)(frame->last_us - frame->first_us),
           (unsigned long)frame->first_us,
           (unsigned long)frame->last_us);
    printf("  Runts/Glitches: %u\r\n", frame->runts);

    /* Raw Hex Dump */
    printf("  Raw Hex       : 0x");
    uint16_t bytes = (frame->bit_count + 7U) / 8U;
    for (uint16_t i = 0U; i < bytes; i++)
    {
        printf("%02X", frame->bits[i]);
    }
    printf("\r\n");

    /* Raw Binary Dump */
    printf("  Raw Binary    : ");
    for (uint16_t i = 0U; i < frame->bit_count; i++)
    {
        if (i > 0 && (i % 8 == 0))
        {
            putchar(' ');
        }
        putchar(Wiegand_GetBit(frame, i) ? '1' : '0');
    }
    printf("\r\n");

    /* Format-specific Decoding */
    if (frame->bit_count == 26U)
    {
        wiegand_card26_t card;
        if (Wiegand_Port_Decode26(frame, &card))
        {
            printf("  Format        : SIA Standard 26-bit (H10301)\r\n");
            printf("  Facility Code : %u (0x%02X)\r\n", card.facility_code, card.facility_code);
            printf("  Card Number   : %u (0x%04X)\r\n", card.card_number, card.card_number);
            printf("  Parity Status : %s\r\n", card.parity_valid ? "VALID (OK)" : "INVALID (Parity Error)");
        }
    }
    else if (frame->bit_count == 34U)
    {
        uint32_t fac = 0;
        uint32_t crd = 0;
        for (uint16_t i = 1; i <= 16; i++)
        {
            fac = (fac << 1) | (Wiegand_GetBit(frame, i) ? 1U : 0U);
        }
        for (uint16_t i = 17; i <= 32; i++)
        {
            crd = (crd << 1) | (Wiegand_GetBit(frame, i) ? 1U : 0U);
        }
        printf("  Format        : Standard 34-bit\r\n");
        printf("  Facility Code : %lu (0x%04lX)\r\n", (unsigned long)fac, (unsigned long)fac);
        printf("  Card Number   : %lu (0x%04lX)\r\n", (unsigned long)crd, (unsigned long)crd);
    }
    printf("====================================================\r\n\r\nwiegand> ");
    fflush(stdout);

    BSP_LED_Off(LED_GREEN);
}

static void on_wiegand_error(void *ctx, wiegand_status_t status)
{
    (void)ctx;
    BSP_LED_Toggle(LED_RED);

    /* Filter minor runt notices if desired, or display all */
    if (status != WIEGAND_STATUS_PULSE_TOO_SHORT)
    {
        printf("\r\n[WIEGAND WARN/ERR] %s (code %d)\r\n\r\nwiegand> ",
               Wiegand_StatusToString(status), (int)status);
        fflush(stdout);
    }
}

/* -------------------------------------------------------------------------- */
/* UART Command Line Interface                                                */
/* -------------------------------------------------------------------------- */

static void print_banner(void)
{
    printf("\r\n");
    printf("************************************************************\r\n");
    printf("*        STM32H563 Wiegand Bidirectional Controller        *\r\n");
    printf("*        D0: PF0 (WIE_D0)  |  D1: PF1 (WIE_D1)             *\r\n");
    printf("*        Timer: TIM2 32-bit (1 MHz counter)                *\r\n");
    printf("************************************************************\r\n");
    printf("Current Direction: %s\r\n", Wiegand_Port_GetDirectionStr());
    printf("Type 'help' for command list.\r\n\r\n");
    printf("wiegand> ");
    fflush(stdout);
}

static void print_help(void)
{
    printf("\r\nAvailable Commands:\r\n");
    printf("  help                    : Show this help message\r\n");
    printf("  status                  : Show port status, direction, GPIO levels, and counts\r\n");
    printf("  mode rx                 : Set port to RECEIVER mode (Input EXTI, Pull-Up)\r\n");
    printf("  mode tx                 : Set port to TRANSMITTER mode (Output OD, Pull-Up)\r\n");
    printf("  tx26 <fac> <card>       : Transmit 26-bit card (e.g. 'tx26 42 1337')\r\n");
    printf("  tx34 <fac> <card>       : Transmit 34-bit card (e.g. 'tx34 100 54321')\r\n");
    printf("  txhex <bits> <hex>      : Transmit arbitrary raw hex bits (e.g. 'txhex 26 0x02A00A72')\r\n");
    printf("  stats                   : Display receiver statistics table\r\n");
    printf("  reset                   : Reset receiver state machine and statistics\r\n");
    printf("  loopback                : Transmit test card (Facility: 42, Card: 1337)\r\n");
    printf("  clear                   : Clear terminal screen\r\n");
}

static void print_status(void)
{
    uint8_t lv = Wiegand_Port_ReadLevels();
    printf("\r\n--- Wiegand Port Status ---\r\n");
    printf("  Direction  : %s\r\n", Wiegand_Port_GetDirectionStr());
    printf("  Pins       : D0 (PF0) = %s, D1 (PF1) = %s\r\n",
           (lv & WIEGAND_LEVEL_D0) ? "HIGH (idle)" : "LOW (active)",
           (lv & WIEGAND_LEVEL_D1) ? "HIGH (idle)" : "LOW (active)");
    printf("  Timestamp  : %lu us\r\n", (unsigned long)Wiegand_Port_GetNowUs());

    wiegand_stats_t st;
    Wiegand_Port_GetStats(&st);
    printf("  Frames OK  : %lu\r\n", (unsigned long)st.frames_ok);
    printf("  Bits Recv  : D0: %lu, D1: %lu\r\n", (unsigned long)st.d0_bits, (unsigned long)st.d1_bits);
    printf("  Queue Peak : %lu / %u entries\r\n", (unsigned long)st.queue_peak, (unsigned)WIEGAND_QUEUE_SIZE);
}

static void print_stats(void)
{
    wiegand_stats_t st;
    Wiegand_Port_GetStats(&st);

    printf("\r\n--- Wiegand Receiver Detailed Statistics ---\r\n");
    printf("  Frames OK        : %lu\r\n", (unsigned long)st.frames_ok);
    printf("  D0 Accepted Bits : %lu\r\n", (unsigned long)st.d0_bits);
    printf("  D1 Accepted Bits : %lu\r\n", (unsigned long)st.d1_bits);
    printf("  Pulse Too Short  : %lu (glitches/runts filtered)\r\n", (unsigned long)st.pulse_too_short);
    printf("  Pulse Too Long   : %lu\r\n", (unsigned long)st.pulse_too_long);
    printf("  Gap Too Short    : %lu\r\n", (unsigned long)st.gap_too_short);
    printf("  Overlap Errors   : %lu\r\n", (unsigned long)st.overlap);
    printf("  Stuck Low Errors : %lu\r\n", (unsigned long)st.stuck_low);
    printf("  Edge Sequence Err: %lu\r\n", (unsigned long)st.edge_sequence);
    printf("  Queue Overflows  : %lu\r\n", (unsigned long)st.queue_overflow);
    printf("  Too Many Bits    : %lu\r\n", (unsigned long)st.too_many_bits);
    printf("  Invalid Bit Count: %lu\r\n", (unsigned long)st.bit_count);
    printf("  Parity Failures  : %lu\r\n", (unsigned long)st.parity);
    printf("  Queue Peak       : %lu\r\n", (unsigned long)st.queue_peak);
}

static void execute_command(char *line)
{
    /* Trim leading whitespace */
    while (*line && isspace((unsigned char)*line))
    {
        line++;
    }

    /* Trim trailing whitespace */
    char *end = line + strlen(line) - 1;
    while (end >= line && isspace((unsigned char)*end))
    {
        *end = '\0';
        end--;
    }

    if (*line == '\0')
    {
        return;
    }

    char *cmd = strtok(line, " ");
    if (cmd == NULL)
    {
        return;
    }

    if (strcmp(cmd, "help") == 0)
    {
        print_help();
    }
    else if (strcmp(cmd, "status") == 0 || strcmp(cmd, "info") == 0)
    {
        print_status();
    }
    else if (strcmp(cmd, "mode") == 0)
    {
        char *arg = strtok(NULL, " ");
        if (arg == NULL)
        {
            printf("Current mode: %s\r\nUsage: mode <rx|tx>\r\n", Wiegand_Port_GetDirectionStr());
        }
        else if (strcmp(arg, "rx") == 0)
        {
            Wiegand_Port_SetDirection(WIEGAND_DIR_RX);
            printf("Port switched to %s\r\nGPIO PF0 (D0) & PF1 (D1) set to Input IT Rising/Falling with Pull-Up.\r\n",
                   Wiegand_Port_GetDirectionStr());
        }
        else if (strcmp(arg, "tx") == 0)
        {
            Wiegand_Port_SetDirection(WIEGAND_DIR_TX);
            printf("Port switched to %s\r\nGPIO PF0 (D0) & PF1 (D1) set to Output Open-Drain with Pull-Up (idle HIGH).\r\n",
                   Wiegand_Port_GetDirectionStr());
        }
        else
        {
            printf("Invalid mode '%s'. Use 'mode rx' or 'mode tx'.\r\n", arg);
        }
    }
    else if (strcmp(cmd, "tx26") == 0)
    {
        char *fac_str  = strtok(NULL, " ");
        char *card_str = strtok(NULL, " ");
        if (fac_str == NULL || card_str == NULL)
        {
            printf("Usage: tx26 <facility: 0..255> <card: 0..65535>\r\nExample: tx26 42 1337\r\n");
            return;
        }

        uint32_t fac = (uint32_t)strtoul(fac_str, NULL, 0);
        uint32_t card = (uint32_t)strtoul(card_str, NULL, 0);

        if (fac > 255U || card > 65535U)
        {
            printf("Error: Facility must be 0..255 and Card 0..65535.\r\n");
            return;
        }

        printf("Transmitting SIA 26-bit Wiegand frame: Facility = %lu, Card = %lu ...\r\n",
               (unsigned long)fac, (unsigned long)card);

        BSP_LED_On(LED_YELLOW);
        if (Wiegand_Port_Transmit26((uint8_t)fac, (uint16_t)card))
        {
            printf("Transmission successful (26 pulses, Tpw=50us, Tpi=2000us, Tfg=25ms).\r\n");
        }
        else
        {
            printf("Transmission failed.\r\n");
        }
        BSP_LED_Off(LED_YELLOW);
    }
    else if (strcmp(cmd, "tx34") == 0)
    {
        char *fac_str  = strtok(NULL, " ");
        char *card_str = strtok(NULL, " ");
        if (fac_str == NULL || card_str == NULL)
        {
            printf("Usage: tx34 <facility: 0..65535> <card: 0..65535>\r\nExample: tx34 100 54321\r\n");
            return;
        }

        uint32_t fac = (uint32_t)strtoul(fac_str, NULL, 0);
        uint32_t card = (uint32_t)strtoul(card_str, NULL, 0);

        if (fac > 65535U || card > 65535U)
        {
            printf("Error: Facility and Card must be 0..65535.\r\n");
            return;
        }

        printf("Transmitting 34-bit Wiegand frame: Facility = %lu, Card = %lu ...\r\n",
               (unsigned long)fac, (unsigned long)card);

        BSP_LED_On(LED_YELLOW);
        if (Wiegand_Port_Transmit34((uint16_t)fac, (uint16_t)card))
        {
            printf("Transmission successful (34 pulses).\r\n");
        }
        else
        {
            printf("Transmission failed.\r\n");
        }
        BSP_LED_Off(LED_YELLOW);
    }
    else if (strcmp(cmd, "txhex") == 0)
    {
        char *bits_str = strtok(NULL, " ");
        char *hex_str  = strtok(NULL, " ");
        if (bits_str == NULL || hex_str == NULL)
        {
            printf("Usage: txhex <bit_count> <hex_string>\r\nExample: txhex 26 0x02A00A72\r\n");
            return;
        }

        uint16_t bit_count = (uint16_t)strtoul(bits_str, NULL, 0);
        if (bit_count == 0 || bit_count > WIEGAND_MAX_BITS)
        {
            printf("Error: bit count must be 1..%u\r\n", (unsigned)WIEGAND_MAX_BITS);
            return;
        }

        printf("Transmitting %u bits from hex %s ...\r\n", bit_count, hex_str);
        BSP_LED_On(LED_YELLOW);
        if (Wiegand_Port_TransmitHex(bit_count, hex_str))
        {
            printf("Transmission successful.\r\n");
        }
        else
        {
            printf("Transmission failed.\r\n");
        }
        BSP_LED_Off(LED_YELLOW);
    }
    else if (strcmp(cmd, "stats") == 0)
    {
        print_stats();
    }
    else if (strcmp(cmd, "reset") == 0)
    {
        Wiegand_Port_Reset();
        printf("Wiegand receiver state machine and edge queues reset.\r\n");
    }
    else if (strcmp(cmd, "loopback") == 0)
    {
        printf("Sending loopback test frame (Facility: 42, Card: 1337) ...\r\n");
        Wiegand_Port_Transmit26(42, 1337);
        printf("Loopback frame transmitted.\r\n");
    }
    else if (strcmp(cmd, "clear") == 0 || strcmp(cmd, "cls") == 0)
    {
        printf("\033[2J\033[H");
        print_banner();
        return;
    }
    else
    {
        printf("Unknown command '%s'. Type 'help' for available commands.\r\n", cmd);
    }
}

/* -------------------------------------------------------------------------- */
/* Application Entry Point                                                    */
/* -------------------------------------------------------------------------- */

void main_app(void)
{
    /* Initialize Wiegand configuration */
    wiegand_config_t cfg;
    Wiegand_ConfigDefault(&cfg);
    cfg.min_bits   = 4U;
    cfg.max_bits   = (uint16_t)WIEGAND_MAX_BITS;
    cfg.on_frame   = on_wiegand_frame;
    cfg.on_error   = on_wiegand_error;
    cfg.validator  = NULL; /* allow all valid bit lengths; 26-bit validator can be set */
    cfg.ctx        = NULL;

    /* Initialize Wiegand port in RX mode */
    if (!Wiegand_Port_Init(&cfg, WIEGAND_DIR_RX))
    {
        printf("FATAL: Failed to initialize Wiegand port!\r\n");
        Error_Handler();
    }

    print_banner();

    uint32_t last_heartbeat = HAL_GetTick();
    uint8_t  rx_byte;

    while (1)
    {
        /* Drain Wiegand edges and run state machine */
        Wiegand_Port_Process();

        /* Non-blocking UART reception for command listener */
        if (HAL_UART_Receive(&hcom_uart[COM1], &rx_byte, 1, 0) == HAL_OK)
        {
            /* Handle Enter / Carriage Return */
            if (rx_byte == '\r' || rx_byte == '\n')
            {
                printf("\r\n");
                s_cmd_buf[s_cmd_len] = '\0';
                if (s_cmd_len > 0)
                {
                    execute_command(s_cmd_buf);
                    s_cmd_len = 0;
                }
                printf("wiegand> ");
                fflush(stdout);
            }
            /* Handle Backspace */
            else if (rx_byte == '\b' || rx_byte == 0x7F)
            {
                if (s_cmd_len > 0)
                {
                    s_cmd_len--;
                    /* Backspace erase on VT100 / standard terminal */
                    printf("\b \b");
                    fflush(stdout);
                }
            }
            /* Printable characters */
            else if (rx_byte >= 32 && rx_byte <= 126)
            {
                if (s_cmd_len < (CMD_BUFFER_SIZE - 1U))
                {
                    s_cmd_buf[s_cmd_len++] = (char)rx_byte;
                    putchar(rx_byte);
                    fflush(stdout);
                }
            }
        }
        else
        {
            /* Clear potential UART error flags (overrun, noise, framing) to prevent lockup */
            __HAL_UART_CLEAR_OREFLAG(&hcom_uart[COM1]);
            __HAL_UART_CLEAR_FLAG(&hcom_uart[COM1], UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
            hcom_uart[COM1].ErrorCode = HAL_UART_ERROR_NONE;
            hcom_uart[COM1].RxState   = HAL_UART_STATE_READY;
        }

        /* Periodic heartbeat LED */
        uint32_t now = HAL_GetTick();
        if ((now - last_heartbeat) >= 1000U)
        {
            last_heartbeat = now;
            BSP_LED_Toggle(LED_GREEN);
        }
    }
}
