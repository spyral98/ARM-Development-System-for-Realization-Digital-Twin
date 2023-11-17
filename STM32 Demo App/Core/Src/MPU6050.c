#include "MPU6050.h"
#include "main.h"

#define PI 3.14159265358979323846

MPU6050_Handle_Typedef MPU6050;

MPU6050_StatusTypeDef MPU6050_init(I2C_HandleTypeDef* hi2c, DMA_HandleTypeDef* hdma)
{
	MPU6050.hi2c = hi2c;
	MPU6050.hdma = hdma;
	MPU6050.data_ready = 0;

	uint8_t errors = 0;

	osDelay(100);

	if (HAL_I2C_IsDeviceReady(MPU6050.hi2c, MPU6050_I2C_ADDR, 3, HAL_MAX_DELAY) != HAL_OK)
		errors++;

	MPU6050.buf[0] = 0x80; // Device Reset
	if (HAL_I2C_Mem_Write(MPU6050.hi2c, MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, MPU6050.buf, 1, 5000) != HAL_OK)
		errors++;

	osDelay(100);

	MPU6050.buf[0] = 0x07; // Signal Path Reset
	if (HAL_I2C_Mem_Write(MPU6050.hi2c, MPU6050_I2C_ADDR, MPU6050_REG_SIGNAL_PATH_RESET, I2C_MEMADD_SIZE_8BIT, MPU6050.buf, 1, 5000) != HAL_OK)
		errors++;

	osDelay(100);

	MPU6050.buf[0] = 0x01; // Gyro X axis as PLL clk ref
	if (HAL_I2C_Mem_Write(MPU6050.hi2c, MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, MPU6050.buf, 1, 5000) != HAL_OK)
		errors++;

	MPU6050.buf[0] = 0x01; // DLPF Fs=1khz, BW = 184Hz (accel) and 188Hz (gyro)
	if (HAL_I2C_Mem_Write(MPU6050.hi2c, MPU6050_I2C_ADDR, MPU6050_REG_CONFIG, I2C_MEMADD_SIZE_8BIT, MPU6050.buf, 1, 5000) != HAL_OK)
		errors++;

	//MPU6050.buf[0] = 0x00; // Sample rate = 1kHz
	//if (HAL_I2C_Mem_Write(MPU6050.hi2c, MPU6050_I2C_ADDR, MPU6050_REG_SMPRT_DIV, I2C_MEMADD_SIZE_8BIT, MPU6050.buf, 1, 5000) != HAL_OK)
	//	errors++;

//	//MPU6050.buf[0] = 0x18; // Gyro full scale range = +- 2000 deg / s
//	//HAL_I2C_Mem_Write(MPU6050.hi2c, MPU6050_I2C_ADDR, MPU6050_REG_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, MPU6050.buf, 1, 5000);
//
//	//MPU6050.buf[0] = 0x18; // Accel full scale range = +- 16g
//	//HAL_I2C_Mem_Write(MPU6050.hi2c, MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, MPU6050.buf, 1, 5000);

	MPU6050.buf[0] = 0x30; // Interrupt status bits are cleared on any read
	if (HAL_I2C_Mem_Write(MPU6050.hi2c, MPU6050_I2C_ADDR, MPU6050_REG_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, MPU6050.buf, 1, 5000) != HAL_OK)
		errors++;

	MPU6050.buf[0] = 0x01; // Enabling Data ready Interrupt
	if (HAL_I2C_Mem_Write(MPU6050.hi2c, MPU6050_I2C_ADDR, MPU6050_REG_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, MPU6050.buf, 1, 5000) != HAL_OK)
		errors++;

//	MPU6050.buf[0] = 0xff; // Checking device ID
//	HAL_I2C_Mem_Read(MPU6050.hi2c, MPU6050_I2C_ADDR, MPU6050_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, MPU6050.buf, 1, 5000);

	if (errors > 0)
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

	return MPU6050_OK;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == I2C2_INT_Pin)
		HAL_I2C_Mem_Read_DMA(MPU6050.hi2c, MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, MPU6050.buf, 14);
}

void MPU6050_convert_from_raw()
{
	int16_t raw;

	// Converting accelerometer values
	raw = (int16_t)(MPU6050.buf[0] << 8 | MPU6050.buf[1]);
	MPU6050.accel_x = raw / 16384.0f;
	raw = (int16_t)(MPU6050.buf[2] << 8 | MPU6050.buf[3]);
	MPU6050.accel_y = raw / 16384.0f;
	raw = (int16_t)(MPU6050.buf[4] << 8 | MPU6050.buf[5]);
	MPU6050.accel_z = raw / 16384.0f;

	// Converting temperature values
	raw = (int16_t)(MPU6050.buf[6] << 8 | MPU6050.buf[7]);
	MPU6050.die_temperature = raw / 340.0f + 36.53f;

	// Converting gyroscope values
	raw = (int16_t)(MPU6050.buf[8]  << 8 | MPU6050.buf[9]);
	//MPU6050.gyro_x = raw / 131.0f; //degrees/sec
	MPU6050.gyro_x = raw * (PI / (131.0f*180.0f));
	raw = (int16_t)(MPU6050.buf[10] << 8 | MPU6050.buf[11]);
	MPU6050.gyro_y = raw * (PI / (131.0f*180.0f));
	raw = (int16_t)(MPU6050.buf[12] << 8 | MPU6050.buf[13]);
	MPU6050.gyro_z = raw * (PI / (131.0f*180.0f));
}
