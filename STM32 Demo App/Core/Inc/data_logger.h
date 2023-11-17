#pragma once

#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"

typedef enum
{
	LOG_MIC,
	LOG_CUR,
	LOG_IMU,
	LOG_MIC_FFT,
	LOG_CUR_FFT,
	LOG_ACC_X_FFT,
	LOG_ACC_Y_FFT,
	LOG_ACC_Z_FFT,
	LOG_GYR_X_FFT,
	LOG_GYR_Y_FFT,
	LOG_GYR_Z_FFT
} LOG_ID;

typedef struct
{
	void* buffer;
	uint32_t size;
	LOG_ID id;
} LOGGING_Q_ITEM;

QueueHandle_t logging_q;

void log_string_via_USB(QueueHandle_t q, LOGGING_Q_ITEM* q_item);

void log_raw_via_USB(QueueHandle_t q, LOGGING_Q_ITEM* q_item);

void log_via_USB(void* buf, uint32_t size, LOG_ID id);
