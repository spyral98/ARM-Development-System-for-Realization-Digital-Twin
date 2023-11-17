#pragma once

#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include <stdint.h>

#define MPU6050_I2C_ADDR (0x68 << 1) // 0x68 if AD0=0 / 0x69 if AD0=1

#define MPU6050_REG_PWR_MGMT_1 			0x6b
#define MPU6050_REG_SIGNAL_PATH_RESET 	0x68
#define MPU6050_REG_CONFIG 				0x1a
#define MPU6050_REG_SMPRT_DIV 			0x19
#define MPU6050_REG_GYRO_CONFIG 		0x1b
#define MPU6050_REG_ACCEL_CONFIG 		0x1c
#define MPU6050_REG_INT_PIN_CFG 		0x37
#define MPU6050_REG_INT_ENABLE	 		0x38
#define MPU6050_REG_WHO_AM_I			0x75
#define MPU6050_REG_ACCEL_XOUT_H		0x3b


#define MPU6050_CONV_FACTOR 0.0625f

#define MPU6050_INIT_TIMEOUT 100
#define MPU6050_MAX_SLEEP_TIME 2000
#define MPU6050_INITIAL_CONV_TIME 26

typedef enum
{
	MPU6050_OK = 0,
	MPU6050_ERROR,
	MPU6050_DMA_ERROR,
	MPU6050_BUSY,
	MPU6050_READY,
	MPU6050_TIMEOUT
} MPU6050_StatusTypeDef;

typedef struct
{
	I2C_HandleTypeDef* hi2c;
	DMA_HandleTypeDef* hdma;
	uint8_t buf[14];
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float die_temperature;
	uint8_t data_ready;
} MPU6050_Handle_Typedef;

MPU6050_Handle_Typedef MPU6050;

MPU6050_StatusTypeDef MPU6050_init(I2C_HandleTypeDef* hi2c, DMA_HandleTypeDef* hdma);
void MPU6050_convert_from_raw();
