#include "BME280.h"
#include "main.h"
#include "cmsis_os.h"

BME280_Handle_Typedef BME280;

BME280_StatusTypeDef BME280_init(I2C_HandleTypeDef* hi2c)
{
	/* Normal mode -> cycling measurements Mode[1:0] = 11
	   Standby time t_sb[2:0]
	   Cycle time = measure_time + standby_time
	*/

	BME280.hi2c = hi2c;

	uint8_t errors = 0;

	osDelay(100);

	// Reset
	//BME280.buf[0] = 0xb6;
	//if (HAL_I2C_Mem_Write(BME280.hi2c, BME280_I2C_ADDR, BME280_REG_RESET, I2C_MEMADD_SIZE_8BIT, BME280.buf, 1, 3000) != HAL_OK)
	//	errors++;

	//osDelay(100);

	// Reading device id
	if (HAL_I2C_Mem_Read(BME280.hi2c, BME280_I2C_ADDR, BME280_REG_ID, I2C_MEMADD_SIZE_8BIT, BME280.buf, 1, 3000) != HAL_OK)
		errors++;

//	BME280.buf[0] = BME280_REG_ID;
//	if (HAL_I2C_Master_Transmit(BME280.hi2c, BME280_I2C_ADDR, BME280.buf, 1, 3000) != HAL_OK)
//		errors++;
//	if (HAL_I2C_Master_Receive(BME280.hi2c, BME280_I2C_ADDR, BME280.buf, 1, 3000) != HAL_OK)
//		errors++;

	//if (BME280.buf[0] == 0x60)
		//HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

	BME280.buf[0] = 0x01; // Setting oversampling for humidity to x1
	if (HAL_I2C_Mem_Write(BME280.hi2c, BME280_I2C_ADDR, BME280_REG_CTRL_HUM, I2C_MEMADD_SIZE_8BIT, BME280.buf, 1, 3000) != HAL_OK)
		errors++;

	BME280.buf[0] = 0x27; // Setting oversampling for temp and press to x1 and mode to normal
	if (HAL_I2C_Mem_Write(BME280.hi2c, BME280_I2C_ADDR, BME280_REG_CTRL_MEAS, I2C_MEMADD_SIZE_8BIT, BME280.buf, 1, 3000) != HAL_OK)
		errors++;

	// Reading calibration parameters
	if (HAL_I2C_Mem_Read(BME280.hi2c, BME280_I2C_ADDR, BME280_REG_CALIB00, I2C_MEMADD_SIZE_8BIT, (uint8_t*) &BME280.dig_T1, 24, 3000) != HAL_OK)
		errors++;
	if (HAL_I2C_Mem_Read(BME280.hi2c, BME280_I2C_ADDR, BME280_REG_CALIB25, I2C_MEMADD_SIZE_8BIT, (uint8_t*) &BME280.dig_H1, 1, 3000) != HAL_OK)
		errors++;

	if (HAL_I2C_Mem_Read(BME280.hi2c, BME280_I2C_ADDR, BME280_REG_CALIB26, I2C_MEMADD_SIZE_8BIT, (uint8_t*) &BME280.dig_H2, 3, 3000) != HAL_OK)
		errors++;

	uint8_t temp[3];
	if (HAL_I2C_Mem_Read(BME280.hi2c, BME280_I2C_ADDR, BME280_REG_CALIB29, I2C_MEMADD_SIZE_8BIT, (uint8_t*) &temp, 3, 3000) != HAL_OK)
		errors++;

	BME280.dig_H4 = (int16_t)(temp[0] << 8) >> 4;
	BME280.dig_H4 |= (temp[1] & 0x0f);

	BME280.dig_H5 = (int16_t)(temp[2] << 8) >> 4;
	BME280.dig_H5 |= (temp[1] & 0xf0);

	if (HAL_I2C_Mem_Read(BME280.hi2c, BME280_I2C_ADDR, BME280_REG_CALIB32, I2C_MEMADD_SIZE_8BIT, (uint8_t*) &BME280.dig_H6, 1, 3000) != HAL_OK)
		errors++;

	if (errors > 0)
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);

	return BME280_OK;
}

void BME280_convert_from_raw()
{
	int32_t raw_press, raw_temp, raw_hum;

	raw_press = ((BME280.buf[0] << 8) | BME280.buf[1]) << 4;
	raw_temp = ((BME280.buf[3] << 8) | BME280.buf[4]) << 4;
	raw_hum = ((BME280.buf[6] << 8) | BME280.buf[7]);

	{
		int32_t var1,var2,T;
		var1 = ((((raw_temp>>3) - ((int32_t)BME280.dig_T1<<1))) * ((int32_t)BME280.dig_T2)) >> 11;
		var2 = (((((raw_temp>>4) - ((int32_t)BME280.dig_T1)) * ((raw_temp>>4) - ((int32_t)BME280.dig_T1))) >> 12) * ((int32_t)BME280.dig_T3)) >> 14;
		BME280.t_fine = var1 + var2;
		T = (BME280.t_fine * 5 + 128) >> 8;
		BME280.temp = T * 0.01f;
	}


	{
		int64_t var1, var2, p;
		var1 = ((int64_t)BME280.t_fine) - 128000;
		var2 = var1 * var1 * (int64_t)BME280.dig_P6;
		var2 = var2 + ((var1*(int64_t)BME280.dig_P5)<<17);
		var2 = var2 + (((int64_t)BME280.dig_P4)<<35);
		var1 = ((var1 * var1 * (int64_t)BME280.dig_P3)>>8) + ((var1 * (int64_t)BME280.dig_P2)<<12);
		var1 = (((((int64_t)1)<<47)+var1))*((int64_t)BME280.dig_P1)>>33;
		//if (var1 == 0) return 0;
		p = 1048576 - raw_press;
		p = (((p<<31)-var2)*3125)/var1;
		var1 = (((int64_t)BME280.dig_P9) * (p>>13) * (p>>13)) >> 25;
		var2 = (((int64_t)BME280.dig_P8) * p) >> 19;
		p = ((p + var1 + var2) >> 8) + (((int64_t)BME280.dig_P7)<<4);
		BME280.press = (uint32_t)p / 25600.0f;
	}


	{
		int32_t var;
		var = (BME280.t_fine - ((int32_t)76800));
		var = (((((raw_hum << 14) - (((int32_t)BME280.dig_H4) << 20) - (((int32_t)BME280.dig_H5) *
				var)) + ((int32_t)16384)) >> 15) * (((((((var *
				((int32_t)BME280.dig_H6)) >> 10) * (((var * ((int32_t)BME280.dig_H3)) >> 11) +
				((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)BME280.dig_H2) +
				8192) >> 14));
		var = (var - (((((var >> 15) * (var >> 15)) >> 7) * ((int32_t)BME280.dig_H1)) >> 4));
		var = (var < 0 ? 0 : var);
		var = (var > 419430400 ? 419430400 : var);
		BME280.hum = (uint32_t) (var >> 12) / 1024.0f;
	}
}

void BME280_read_data()
{
	HAL_I2C_Mem_Read(BME280.hi2c, BME280_I2C_ADDR, BME280_REG_PRESS_MSB, I2C_MEMADD_SIZE_8BIT, BME280.buf, 8, 3000);
	BME280_convert_from_raw();
}
