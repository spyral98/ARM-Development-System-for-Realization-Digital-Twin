#pragma once

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "queue.h"
#include "data_logger.h"

#define UART_BUF_LEN 256
#define MAX_SLEEP_TIME 10000

typedef enum
{
	AT_OK = 0,
	AT_ERROR,
	AT_DMA_ERROR,
	AT_BUSY,
	AT_READY,
	AT_TIMEOUT
} AT_StatusTypeDef;

typedef struct
{
	UART_HandleTypeDef* huart;
	DMA_HandleTypeDef* hdma;
	osThreadId_t taskHandle;
	uint32_t uart_rx_buffer_offset;
	uint8_t uart_rx_buffer[UART_BUF_LEN];
	LOGGING_Q_ITEM* q_item;
} AT_HandleTypeDef;

AT_HandleTypeDef AT;

AT_StatusTypeDef AT_init(UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma, osThreadId_t taskHandle);

AT_StatusTypeDef AT_send_command(char* command, char* expected_response);

AT_StatusTypeDef AT_log_response(QueueHandle_t q);

AT_StatusTypeDef AT_log_return_value_via_USB(AT_StatusTypeDef rv);
