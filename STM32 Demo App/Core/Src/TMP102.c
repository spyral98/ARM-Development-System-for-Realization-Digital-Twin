#include "TMP102.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "main.h"

TMP102_Handle_Typedef TMP102;

TMP102_StatusTypeDef TMP102_init(I2C_HandleTypeDef* hi2c)
{
	TMP102.hi2c = hi2c;
	TMP102.current_temperature = 0.0f;

	// Waiting for the initial conversion to finish
	osDelay(TMP102_INITIAL_CONV_TIME);

	// Checking if the device is ready
	if (HAL_I2C_IsDeviceReady(TMP102.hi2c, TMP102_I2C_ADDR, 1, TMP102_INIT_TIMEOUT) == HAL_OK)
		return TMP102_OK;
	else
		return TMP102_ERROR;
}

TMP102_StatusTypeDef TMP102_read_temperature()
{
	// Sending the register address we want to read from (Temperature register)
	TMP102.buf[0] = TMP102_REG_TEMP;

	// Sending the address
	if (HAL_I2C_Mem_Read(TMP102.hi2c, TMP102_I2C_ADDR, TMP102_REG_TEMP, I2C_MEMADD_SIZE_8BIT, TMP102.buf, 2, TMP102_MAX_SLEEP_TIME) != HAL_OK)
		return TMP102_ERROR;

	// Combine the bytes
	int16_t val = ((int16_t)TMP102.buf[0] << 4) | (TMP102.buf[1] >> 4);

	// Sign extension
	if ( val > 0x7FF )
	  val |= 0xF000;

	// Convert temperature value to degrees Celsius
	TMP102.current_temperature = val * TMP102_CONV_FACTOR;

	return TMP102_OK;
}


