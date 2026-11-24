#include "hc06.h"
#include <string.h>

/* HC-06 UART1: PA9 = STM32_TX -> HC-06_RX
 *              PA10 = STM32_RX <- HC-06_TX
 * Baud rate: 9600, 8 data bits, no parity, 1 stop bit.
 */
UART_HandleTypeDef huart1;
static uint8_t rxByte;
extern volatile int32_t dbgCmd;   /* bien dieu khien mode trong mm_app.c */

void HC06_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }

    /* Nhan ky tu bang interrupt de van nhan lenh khi robot dang chay. */
    HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    HC06_SendString("MicroMouse ready. Send 1-6 to select mode, S to stop.\r\n");
}

void HC06_SendString(const char *s)
{
    if (s != NULL) {
        HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 100);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        char c = (char)rxByte;

        if (c >= '1' && c <= '6') {
            dbgCmd = c - '0';   /* chon va chay mode 1..6 */
        } else if (c == 'S' || c == 's') {
            dbgCmd = -1;        /* dung/abort mode dang chay */
        }

        HAL_UART_Receive_IT(&huart1, &rxByte, 1); /* bat dau nhan ky tu tiep */
    }
}
