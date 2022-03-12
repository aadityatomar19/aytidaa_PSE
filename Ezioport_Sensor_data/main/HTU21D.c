/*
 * HTU21D.C
 *
 *  Created on: 11-Mar-2022
 *      Author: root
 */

#include "HTU21D.h"
/*
 * @brief This function crc algorithm according to HTU21 datasheet to check received data validity
 * @param value temp or humidity value receievd from htu
 * @param crc CRC value received from htu
 * @return bool
 *              1 if valid value
 *              0 if invalid value
 */
bool is_crc_valid(uint16_t value, uint8_t crc) {
	uint32_t row = (uint32_t)value << 8;
	row |= crc;
	uint32_t divisor = (uint32_t)0x988000;

	for (int i = 0 ; i < 16 ; i++) {
		if (row & (uint32_t)1 << (23 - i))
			row ^= divisor;
		divisor >>= 1;
	}
	return (row == 0);
}

/*
 * @brief read value from htu21 register using i2c
 * @param command Register from which data need to be read
 *                (TRIG_TEMP_MEASUREMENT_NOHOLD or TRIG_HUMD_MEASUREMENT_NOHOLD)
 * @return uint16_t
 *         0 if read fails
 *         raw_value  if read success
 */
uint16_t read_value(i2c_port_t i2c_num, uint16_t *raw_value, uint8_t command) {
	esp_err_t ret;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd,
			(HTU21D_ADDR << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, command, true);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if ( ret != ESP_OK ) {
		*raw_value = 0;
		return ret;
	}

	vTaskDelay(50 / portTICK_RATE_MS);

	uint8_t msb, lsb, crc;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd,
			(HTU21D_ADDR << 1) | I2C_MASTER_READ, true);
	i2c_master_read_byte(cmd, &msb, I2C_MASTER_ACK);
	i2c_master_read_byte(cmd, &lsb, I2C_MASTER_ACK);
	i2c_master_read_byte(cmd, &crc, I2C_MASTER_NACK);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	*raw_value = ((uint16_t) msb << 8) | ( (uint16_t) lsb);

	if ( !is_crc_valid(*raw_value, crc) ) {
		*raw_value = 0;
		ret = ESP_FAIL;
	} else {
		*raw_value = *raw_value & 0xFFFC;
	}
	return ret;
}

/*
 * @brief convert raw temperatue value to *C if value is valid otherwise returns 0
 * @return temp_data_t (temp value and crc validity)
 */
esp_err_t ht21d_read_temperature(i2c_port_t i2c_num, float *temperature) {
	esp_err_t ret;
	uint16_t raw_temperature;
	ret = read_value(i2c_num,&raw_temperature, TRIG_TEMP_MEASUREMENT_NOHOLD);
	float temp = (raw_temperature *(175.72 / 65536.0)) - 46.85;

	*temperature = temp;
	return ret;
}

/*
 * @brief convert raw humidity value to %rh if value is valid otherwise returns 0
 * @return hum_data_t (humidity value and crc validity)
 */
esp_err_t ht21d_read_humidity(i2c_port_t i2c_num,float *humidity) {
	esp_err_t ret;
	uint16_t raw_humidity;
	ret = read_value(i2c_num,&raw_humidity, TRIG_HUMD_MEASUREMENT_NOHOLD);
	float hum = ((raw_humidity * 125.0 / 65536.0) - 6.0);

	*humidity = hum;
	return ret;
}

