/*
 * BMP280.c
 *
 *  Created on: 11-Mar-2022
 *      Author: root
 */
#include "BMP280.h"

static const char *TAG_BMP ="BMP280";

esp_err_t bmp280_init_default_params(bmp280_params_t *params)
{
	params->mode = BMP280_MODE_NORMAL;
	params->filter = BMP280_FILTER_OFF;
	params->oversampling_pressure = BMP280_STANDARD;
	params->oversampling_temperature = BMP280_STANDARD;
	//	params->oversampling_humidity = BMP280_STANDARD;
	params->standby = BMP280_STANDBY_250;
	return ESP_OK;
}

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

esp_err_t i2c_dev_write_reg(i2c_port_t i2c_num,uint8_t dev_addr, uint8_t reg,const void *out_data, size_t out_size)
{
	return i2c_dev_write(i2c_num, dev_addr, &reg, 1, out_data, out_size);
}

inline static esp_err_t write_register8(i2c_port_t i2c_num, uint8_t dev_addr, uint8_t addr, uint8_t value)
{
	return i2c_dev_write_reg(i2c_num, dev_addr, addr, &value, 1);
}

esp_err_t i2c_read_reg(i2c_port_t i2c_num, uint8_t addr, uint8_t reg,void *in_data, size_t in_size)
{
	return i2c_dev_read(i2c_num, addr, &reg, 1, in_data, in_size);
}

static esp_err_t read_register16(i2c_port_t i2c_num, uint8_t dev_addr, uint8_t reg, uint16_t *r)
{
	uint8_t d[] = { 0, 0 };

	i2c_read_reg(i2c_num, dev_addr, reg, d, 2);
	*r = d[0] | (d[1] << 8);

	return ESP_OK;
}


void bmp_read_calibration_data(i2c_port_t i2c_num, bmp280_t *dev)
{
	read_register16(i2c_num, BMP280_I2C_ADDRESS_0, 0x88, &dev->dig_T1);
	read_register16(i2c_num, BMP280_I2C_ADDRESS_0, 0x8a, (uint16_t *)&dev->dig_T2);
	read_register16(i2c_num, BMP280_I2C_ADDRESS_0, 0x8c, (uint16_t *)&dev->dig_T3);
	read_register16(i2c_num, BMP280_I2C_ADDRESS_0, 0x8e, &dev->dig_P1);
	read_register16(i2c_num, BMP280_I2C_ADDRESS_0, 0x90, (uint16_t *)&dev->dig_P2);
	read_register16(i2c_num, BMP280_I2C_ADDRESS_0, 0x92, (uint16_t *)&dev->dig_P3);
	read_register16(i2c_num, BMP280_I2C_ADDRESS_0, 0x94, (uint16_t *)&dev->dig_P4);
	read_register16(i2c_num, BMP280_I2C_ADDRESS_0, 0x96, (uint16_t *)&dev->dig_P5);
	read_register16(i2c_num, BMP280_I2C_ADDRESS_0, 0x98, (uint16_t *)&dev->dig_P6);
	read_register16(i2c_num, BMP280_I2C_ADDRESS_0, 0x9a, (uint16_t *)&dev->dig_P7);
	read_register16(i2c_num, BMP280_I2C_ADDRESS_0, 0x9c, (uint16_t *)&dev->dig_P8);
	read_register16(i2c_num, BMP280_I2C_ADDRESS_0, 0x9e, (uint16_t *)&dev->dig_P9);

	ESP_LOGI(TAG_BMP, "Calibration data received:");
	ESP_LOGI(TAG_BMP, "dig_T1=%d", dev->dig_T1);
	ESP_LOGI(TAG_BMP, "dig_T2=%d", dev->dig_T2);
	ESP_LOGI(TAG_BMP, "dig_T3=%d", dev->dig_T3);
	ESP_LOGI(TAG_BMP, "dig_P1=%d", dev->dig_P1);
	ESP_LOGI(TAG_BMP, "dig_P2=%d", dev->dig_P2);
	ESP_LOGI(TAG_BMP, "dig_P3=%d", dev->dig_P3);
	ESP_LOGI(TAG_BMP, "dig_P4=%d", dev->dig_P4);
	ESP_LOGI(TAG_BMP, "dig_P5=%d", dev->dig_P5);
	ESP_LOGI(TAG_BMP, "dig_P6=%d", dev->dig_P6);
	ESP_LOGI(TAG_BMP, "dig_P7=%d", dev->dig_P7);
	ESP_LOGI(TAG_BMP, "dig_P8=%d", dev->dig_P8);
	ESP_LOGI(TAG_BMP, "dig_P9=%d", dev->dig_P9);
}

static inline int32_t compensate_temperature(bmp280_t *dev, int32_t adc_temp, int32_t *fine_temp)
{
	int32_t var1, var2;
	var1 = ((((adc_temp >> 3) - ((int32_t)dev->dig_T1 << 1))) * (int32_t)dev->dig_T2) >> 11;
	var2 = (((((adc_temp >> 4) - (int32_t)dev->dig_T1) * ((adc_temp >> 4) - (int32_t)dev->dig_T1)) >> 12) * (int32_t)dev->dig_T3) >> 14;
	*fine_temp = var1 + var2;
	return (*fine_temp * 5 + 128) >> 8;
}

/*
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_pressure(bmp280_t *dev, int32_t adc_press, int32_t fine_temp)
{
	int64_t var1, var2, p;

	var1 = (int64_t)fine_temp - 128000;
	var2 = var1 * var1 * (int64_t)dev->dig_P6;
	var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
	var2 = var2 + (((int64_t)dev->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12);
	var1 = (((int64_t)1 << 47) + var1) * ((int64_t)dev->dig_P1) >> 33;

	if (var1 == 0)
	{
		return 0;  // avoid exception caused by division by zero
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t)dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t)dev->dig_P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + ((int64_t)dev->dig_P7 << 4);
	return p;
}

esp_err_t bmp280_read_fixed(i2c_port_t i2c_num, bmp280_t *dev, int32_t *temperature, uint32_t *pressure)
{
	int32_t adc_pressure;
	int32_t adc_temp;
	uint8_t data[8];
	// Need to read in one sequence to ensure they match.
	size_t size = 6;
	if((i2c_read_reg(i2c_num, BMP280_I2C_ADDRESS_0, 0xf7, data, size)) != ESP_OK){
		ESP_LOGI(TAG_BMP,"Failed to read data");
		return ESP_FAIL;
	}
	adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	adc_temp = data[3] << 12 | data[4] << 4 | data[5] >> 4;
	//    ESP_LOGI(TAG, "ADC temperature: %d", adc_temp);
	//    ESP_LOGI(TAG, "ADC pressure: %d", adc_pressure);
	int32_t fine_temp;
	*temperature = compensate_temperature(dev, adc_temp, &fine_temp);
	*pressure = compensate_pressure(dev, adc_pressure, fine_temp);
	return ESP_OK;

}

esp_err_t bmp280_read_float(i2c_port_t i2c_num, bmp280_t *dev, float *temperature, float *pressure)
{
	int32_t fixed_temperature;
	uint32_t fixed_pressure;
	if (bmp280_read_fixed(i2c_num, dev, &fixed_temperature, &fixed_pressure) != ESP_OK){
		return ESP_FAIL;
	}
	*temperature = (float)fixed_temperature / 100;
	*pressure = (float)fixed_pressure / 256/100;
	return ESP_OK;

}
esp_err_t bmp280_init(i2c_port_t i2c_num, bmp280_params_t *params, bmp280_t *dev)
{
	i2c_read_reg(i2c_num, BMP280_I2C_ADDRESS_0, BMP280_REG_ID, &dev->id, 1);
	ESP_LOGI(TAG_BMP,"chipid=%x",dev->id);
	if (dev->id != BMP280_CHIP_ID)
	{
		ESP_LOGI(TAG_BMP, "Invalid chip ID: expected: 0x%x (BMP280) got: 0x%x",
				BMP280_CHIP_ID, dev->id);
		return ESP_FAIL;
	}
	write_register8(i2c_num, BMP280_I2C_ADDRESS_0, BMP280_REG_RESET, BMP280_RESET_VALUE);
	uint8_t status=0;
//	unsigned long previous=0;
//	previous = millis();
	while (!status)
	{
		i2c_read_reg(i2c_num, BMP280_I2C_ADDRESS_0, BMP280_REG_STATUS, &status, 1);
		if(!status)
			ESP_LOGI(TAG_BMP,"failed to read status\n");
	}
	vTaskDelay(500/portTICK_RATE_MS);
	bmp_read_calibration_data(i2c_num, dev);

	uint8_t config = (params->standby << 5) | (params->filter << 2);
	ESP_LOGI(TAG_BMP, "Writing config reg=%x", config);
	write_register8(i2c_num, BMP280_I2C_ADDRESS_0, BMP280_REG_CONFIG, config);
	if (params->mode == BMP280_MODE_FORCED)
	{
		params->mode = BMP280_MODE_SLEEP;  // initial mode for forced is sleep
	}
	uint8_t ctrl = (params->oversampling_temperature << 5) | (params->oversampling_pressure << 2) | (params->mode);
	ESP_LOGI(TAG_BMP, "Writing ctrl reg=%x", ctrl);
	if(write_register8(i2c_num, BMP280_I2C_ADDRESS_0, BMP280_REG_CTRL, ctrl) != ESP_OK){
		ESP_LOGI(TAG_BMP,"Failed to control sensor");
		return ESP_FAIL;
	}
	return ESP_OK;
}


