#pragma once

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include <stdint.h>

#define TMP102_I2C_ADDR ((uint16_t)(0x48 << 1))

#define TMP102_REG_TEMP 	0x00
#define TMP102_REG_CONFIG 	0x01
#define TMP102_REG_TLOW 	0x02
#define TMP102_REG_THIGH 	0x03

#define TMP102_CONV_FACTOR 0.0625f

#define TMP102_INIT_TIMEOUT 100
#define TMP102_MAX_SLEEP_TIME 2000
#define TMP102_INITIAL_CONV_TIME 26

typedef enum
{
	TMP102_OK = 0,
	TMP102_ERROR,
	TMP102_DMA_ERROR,
	TMP102_BUSY,
	TMP102_READY,
	TMP102_TIMEOUT
} TMP102_StatusTypeDef;

typedef struct
{
	I2C_HandleTypeDef* hi2c;
	uint8_t buf[2];
	float current_temperature;
} TMP102_Handle_Typedef;

TMP102_Handle_Typedef TMP102;

TMP102_StatusTypeDef TMP102_init(I2C_HandleTypeDef* hi2c);
TMP102_StatusTypeDef TMP102_read_temperature();
