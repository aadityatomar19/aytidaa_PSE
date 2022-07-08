/*
 * BMP280.c
 *
 *  Created on: 11-Mar-2022
 *      Author: root
 */
#include "BMP180.h"

static const char *TAG_BMP ="BMP180";

esp_err_t i2c_dev_read(i2c_port_t i2c_num, uint8_t addr, const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
	esp_err_t err;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	if (out_data && out_size)
	{
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, addr << 1, true);
		i2c_master_write(cmd, (void *)out_data, out_size, true);
	}
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (addr << 1) | 1, true);
	i2c_master_read(cmd, in_data, in_size, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return err;
}


esp_err_t i2c_dev_write(i2c_port_t i2c_num,uint8_t dev_addr, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)
{
	esp_err_t err;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, dev_addr << 1, true);
	if (out_reg && out_reg_size)
		i2c_master_write(cmd, (void *)out_reg, out_reg_size, true);
	i2c_master_write(cmd, (void *)out_data, out_size, true);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return err;
}

inline esp_err_t i2c_dev_write_reg(i2c_port_t i2c_num,uint8_t dev_addr, uint8_t reg,const void *out_data, size_t out_size)
{
	return i2c_dev_write(i2c_num, dev_addr, &reg, 1, out_data, out_size);
}

inline esp_err_t i2c_read_reg(i2c_port_t i2c_num, uint8_t addr, uint8_t reg,void *in_data, size_t in_size)
{
	return i2c_dev_read(i2c_num, addr, &reg, 1, in_data, in_size);
}

inline static esp_err_t read_register16(i2c_port_t i2c_num, uint8_t dev_addr, uint8_t reg, int16_t *r)
{
	uint8_t d[] = { 0, 0 };

	i2c_read_reg(i2c_num, dev_addr, reg, d, 2);
	*r = ((int16_t)d[0] << 8) | d[1] ;

	return ESP_OK;
}


void bmp_read_calibration_data(i2c_port_t i2c_num, bmp180_t *dev)
{
	read_register16(i2c_num, BMP180_I2C_ADDRESS_0, BMP180_CALIBRATION_REG + 0, &dev->AC1);
	read_register16(i2c_num, BMP180_I2C_ADDRESS_0, BMP180_CALIBRATION_REG + 2,&dev->AC2);
	read_register16(i2c_num, BMP180_I2C_ADDRESS_0, BMP180_CALIBRATION_REG + 4,&dev->AC3);
	read_register16(i2c_num, BMP180_I2C_ADDRESS_0, BMP180_CALIBRATION_REG + 6,(int16_t *) &dev->AC4);
	read_register16(i2c_num, BMP180_I2C_ADDRESS_0, BMP180_CALIBRATION_REG + 8,(int16_t *)&dev->AC5);
	read_register16(i2c_num, BMP180_I2C_ADDRESS_0, BMP180_CALIBRATION_REG + 10,(int16_t *)&dev->AC6);
	read_register16(i2c_num, BMP180_I2C_ADDRESS_0, BMP180_CALIBRATION_REG + 12,&dev->B1);
	read_register16(i2c_num, BMP180_I2C_ADDRESS_0, BMP180_CALIBRATION_REG + 14,&dev->B2);
	read_register16(i2c_num, BMP180_I2C_ADDRESS_0, BMP180_CALIBRATION_REG + 16,&dev->MB);
	read_register16(i2c_num, BMP180_I2C_ADDRESS_0, BMP180_CALIBRATION_REG + 18,&dev->MC);
	read_register16(i2c_num, BMP180_I2C_ADDRESS_0, BMP180_CALIBRATION_REG + 20,&dev->MD);

	ESP_LOGI(TAG_BMP, "Calibration data received:");
	ESP_LOGI(TAG_BMP, "dig_T1=%d", dev->AC1);
	ESP_LOGI(TAG_BMP, "dig_T2=%d", dev->AC2);
	ESP_LOGI(TAG_BMP, "dig_T3=%d", dev->AC3);
	ESP_LOGI(TAG_BMP, "dig_P1=%d", dev->AC4);
	ESP_LOGI(TAG_BMP, "dig_P2=%d", dev->AC5);
	ESP_LOGI(TAG_BMP, "dig_P3=%d", dev->AC6);
	ESP_LOGI(TAG_BMP, "dig_P4=%d", dev->B1);
	ESP_LOGI(TAG_BMP, "dig_P5=%d", dev->B2);
	ESP_LOGI(TAG_BMP, "dig_P6=%d", dev->MB);
	ESP_LOGI(TAG_BMP, "dig_P7=%d", dev->MC);
	ESP_LOGI(TAG_BMP, "dig_P8=%d", dev->MD);
}

esp_err_t bmp180_start_measurement(i2c_port_t i2c_num, uint8_t cmd)
{
	return i2c_dev_write_reg(i2c_num,BMP180_I2C_ADDRESS_0, BMP180_CONTROL_REG, &cmd, 1);
}
esp_err_t bmp180_get_uncompensated_temperature(i2c_port_t i2c_num, int32_t *temperature){
	if (bmp180_start_measurement(i2c_num, BMP180_MEASURE_TEMP) != ESP_OK){
		ESP_LOGI(TAG_BMP, "Write register failed in temp function");
		return ESP_FAIL;
	}
	ets_delay_us(5000);
	int16_t v;
	if((read_register16( i2c_num, BMP180_I2C_ADDRESS_0, BMP180_OUT_MSB_REG, &v)) != ESP_OK){
		ESP_LOGI(TAG_BMP, "Read Data failed in temp function");
		return ESP_FAIL;
	}
	*temperature = v;
	return ESP_OK;
}

static esp_err_t bmp180_get_uncompensated_pressure(i2c_port_t i2c_num, uint32_t *pressure)
{
	uint16_t us;
	us = 8000;

	// Write Start Code into reg 0xF4
	if(bmp180_start_measurement(i2c_num, BMP180_MEASURE_PRESS | (1 << 6)) != ESP_OK){
		ESP_LOGI(TAG_BMP, "Write register failed in pressure function");
		return ESP_FAIL;
	}
	ets_delay_us(us);
	uint8_t d[] = { 0, 0, 0 };
	uint8_t reg = BMP180_OUT_MSB_REG;
	if(i2c_read_reg(i2c_num, BMP180_I2C_ADDRESS_0, reg, d, 3) != ESP_OK){
		ESP_LOGI(TAG_BMP, "Read Data failed in pressure");
		return ESP_FAIL;
	}
	uint32_t r = ((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2];
	r >>= 8 - 1;
	*pressure = r;
	return ESP_OK;
}

esp_err_t bmp180_read_float(i2c_port_t i2c_num, bmp180_t *dev, float *temperature, float *pressure)
{
	int32_t T, P;
	int32_t UT, X1, X2, B5;
	UT = 0;
	if (bmp180_get_uncompensated_temperature(i2c_num, &UT) != ESP_OK){
		return ESP_FAIL;
	}
	X1 = ((UT - (int32_t)dev->AC6) * (int32_t)dev->AC5) >> 15;
	X2 = ((int32_t)dev->MC << 11) / (X1 + (int32_t)dev->MD);
	B5 = X1 + X2;
	T = (B5 + 8) >> 4;
	*temperature = T / 10.0;

	int32_t X3, B3, B6;
	uint32_t B4, B7, UP = 0;
	if (bmp180_get_uncompensated_pressure(i2c_num, &UP) != ESP_OK){
		return ESP_FAIL;
	}

	B6 = B5 - 4000;
	X1 = ((int32_t)dev->B2 * ((B6 * B6) >> 12)) >> 11;
	X2 = ((int32_t)dev->AC2 * B6) >> 11;
	X3 = X1 + X2;

	B3 = ((((int32_t)dev->AC1 * 4 + X3) << 1) + 2) >> 2;
	X1 = ((int32_t)dev->AC3 * B6) >> 13;
	X2 = ((int32_t)dev->B1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	B4 = ((uint32_t)dev->AC4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (uint32_t)(50000UL >> 1);

	if (B7 < 0x80000000UL)
		P = (B7 * 2) / B4;
	else
		P = (B7 / B4) * 2;

	X1 = (P >> 8) * (P >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * P) >> 16;
	P = P + ((X1 + X2 + (int32_t)3791) >> 4);

	*pressure = (float)P/100;
	return ESP_OK;

}
esp_err_t bmp180_init(i2c_port_t i2c_num, bmp180_t *dev)
{
	i2c_read_reg(i2c_num, BMP180_I2C_ADDRESS_0, BMP180_VERSION_REG, &dev->id, 1);
	ESP_LOGI(TAG_BMP,"chipid=%x",dev->id);
	if (dev->id != BMP180_CHIP_ID)
	{
		ESP_LOGI(TAG_BMP, "Invalid chip ID: expected: 0x%x (BMP280) got: 0x%x",
				BMP180_CHIP_ID, dev->id);
		return ESP_FAIL;
	}

	bmp_read_calibration_data(i2c_num, dev);
	return ESP_OK;
}


