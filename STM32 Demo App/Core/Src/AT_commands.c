#include "AT_commands.h"

#include "stm32l4xx_hal.h"
#include "string.h"
#include "FreeRTOS.h"
#include "usbd_cdc_if.h"
#include "stream_buffer.h"

AT_HandleTypeDef AT;

AT_StatusTypeDef AT_init(UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma, osThreadId_t taskHandle)
{
	AT.huart = huart;
	AT.hdma = hdma;
	AT.taskHandle = taskHandle;

	AT.q_item = malloc(sizeof(LOGGING_Q_ITEM));
	AT.q_item->buffer = AT.uart_rx_buffer;

	return AT_OK;
}

AT_StatusTypeDef AT_send_command(char* command, char* expected_response)
{
	// Sending AT command to device

	// Clears the RX buffer
	memset(AT.uart_rx_buffer, 0, UART_BUF_LEN);

	// Resets the offset to the RX buffer
	AT.uart_rx_buffer_offset = 0;

	// Starting DMA reception of UART3 RX and receive interrupt when line is idle
	if (HAL_UARTEx_ReceiveToIdle_DMA(AT.huart, AT.uart_rx_buffer, UART_BUF_LEN) != HAL_OK)
		return AT_DMA_ERROR;
	// Disabling DMA Half-Transfer Interrupt
	__HAL_DMA_DISABLE_IT(AT.hdma, DMA_IT_HT);


	AT_StatusTypeDef res = AT_READY;
	do
	{
		// Sending the command via DMA to UART3 TX
		if (HAL_UART_Transmit_DMA(AT.huart, (uint8_t*)command, strlen(command)) != HAL_OK)
			return AT_DMA_ERROR;

		do
		{
			// Waiting for the interrupt (Non-blocking). If MAX_SLEEP_TIME (in ms) passes AT_TIMEOUT is returned
			uint32_t notify_res = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MAX_SLEEP_TIME));
			if (notify_res == 0)
			{
				res = AT_TIMEOUT;
				break;
			}

			// Parsing the response
			if (strstr((char*)AT.uart_rx_buffer, expected_response) != NULL)
				res = AT_OK;
			else if (strstr((char*)AT.uart_rx_buffer, "ERROR\r\n") != NULL)
				res = AT_ERROR;
			else if (strstr((char*)AT.uart_rx_buffer, "busy...\r\n") != NULL)
				res = AT_BUSY;
		} while (res == AT_READY); // If the response doesn't contain the following strings then we need to receive again

	} while (res == AT_BUSY); // If the device is busy we resend the command

	// Stopping the DMA reception
	if (HAL_UART_DMAStop(AT.huart) != HAL_OK)
		return AT_DMA_ERROR;

	//log_string_via_USB(logging_q, AT.q_item);

	return res;
}


AT_StatusTypeDef AT_log_return_value_via_USB(AT_StatusTypeDef rv)
{
	// Logging the response via USB Full-Speed
	char usb_buf[32];

	osDelay(1);

	switch (rv)
	{
	case AT_OK:
		sprintf(usb_buf, "Return value: AT_OK\r\n");
		break;
	case AT_ERROR:
		sprintf(usb_buf, "Return value: AT_ERROR\r\n");
		break;
	case AT_DMA_ERROR:
		sprintf(usb_buf, "Return value: AT_DMA_ERROR\r\n");
		break;
	case AT_BUSY:
		sprintf(usb_buf, "Return value: AT_BUSY\r\n");
		break;
	case AT_READY:
		sprintf(usb_buf, "Return value: AT_READY\r\n");
		break;
	case AT_TIMEOUT:
		sprintf(usb_buf, "Return value: AT_TIMEOUT\r\n");
		break;
	}

	USBD_StatusTypeDef res = CDC_Transmit_FS((uint8_t*)usb_buf, strlen(usb_buf));

	if (res == USBD_OK)
		return AT_OK;
	else if (res == USBD_BUSY)
		return AT_BUSY;
	else
		return AT_ERROR;
}

AT_StatusTypeDef AT_log_response_via_USB()
{
	// Logging the return value via USB Full-Speed

	osDelay(1);

	USBD_StatusTypeDef res = CDC_Transmit_FS(AT.uart_rx_buffer, strlen((char*)AT.uart_rx_buffer));

	if (res == USBD_OK)
		return AT_OK;
	else if (res == USBD_BUSY)
		return AT_BUSY;
	else
		return AT_ERROR;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	// UART RX idle line ISR

	// Incrementing the buffer offset by the number of characters received
	AT.uart_rx_buffer_offset += Size;

	HAL_UARTEx_ReceiveToIdle_DMA(AT.huart, AT.uart_rx_buffer+AT.uart_rx_buffer_offset, UART_BUF_LEN-AT.uart_rx_buffer_offset);
	__HAL_DMA_DISABLE_IT(AT.hdma, DMA_IT_HT);

	// Indicator that higher priority task has woken while we are in ISR
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// Waking up task
	vTaskNotifyGiveFromISR(AT.taskHandle, &xHigherPriorityTaskWoken);

	// Run the scheduler upon exit of ISR
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}
