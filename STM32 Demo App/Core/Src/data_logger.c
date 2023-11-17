#include "data_logger.h"
#include "main.h"

QueueHandle_t logging_q;

void log_string_via_USB(QueueHandle_t q, LOGGING_Q_ITEM* q_item)
{
	q_item->size = strlen(q_item->buffer);
	xQueueSend(q, (void*) q_item, portMAX_DELAY);
}

void log_raw_via_USB(QueueHandle_t q, LOGGING_Q_ITEM* q_item)
{
	xQueueSend(q, (void*) q_item, portMAX_DELAY);
}

void log_via_USB(void* buf, uint32_t size, LOG_ID id)
{
	LOGGING_Q_ITEM q_item;
	q_item.buffer = buf;
	q_item.size = size;
	q_item.id = id;

	if (xQueueSend(logging_q, (void*)&q_item, 0) != pdPASS)
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
}
