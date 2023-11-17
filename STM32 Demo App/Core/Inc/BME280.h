#pragma once

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include <stdint.h>

#define BME280_I2C_ADDR (0x77 << 1) // 0x76 if SD0=0 / 0x77 if SD0=1

#define BME280_REG_ID 					0xd0
#define BME280_REG_RESET				0xe0
#define BME280_REG_CTRL_MEAS			0xf4
#define BME280_REG_CTRL_HUM				0xf2
#define BME280_REG_PRESS_MSB			0xf7
#define BME280_REG_CALIB00				0x88
#define BME280_REG_CALIB25 				0xa1
#define BME280_REG_CALIB26				0xe1
#define BME280_REG_CALIB29 				0xe4
#define BME280_REG_CALIB32				0xe7



#define BME280_INIT_TIMEOUT 100
#define BME280_MAX_SLEEP_TIME 2000
#define BME280_INITIAL_CONV_TIME 26

typedef enum
{
	BME280_OK = 0,
	BME280_ERROR,
	BME280_DMA_ERROR,
	BME280_BUSY,
	BME280_READY,
	BME280_TIMEOUT
} BME280_StatusTypeDef;

typedef struct
{
	I2C_HandleTypeDef* hi2c;
	uint8_t buf[8];
	float hum;
	float temp;
	float press;

	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;

	int32_t t_fine;

} BME280_Handle_Typedef;

BME280_Handle_Typedef BME280;

BME280_StatusTypeDef BME280_init(I2C_HandleTypeDef* hi2c);
void BME280_read_data();
void BME280_convert_from_raw();
