/*
 * DS1307.c
 *
 *  Created on: 11-Mar-2022
 *      Author: root
 */
#include "DS1307.h"

static const char *TAG_RTC ="RTC";

/*
 * @brief Convert integer into binary coded decimal
 * @return Binary coded decimal
 */
uint8_t intToBCD(uint8_t num) {
	return ((num / 10) << 4) | (num%10);
}

/*
 * @brief Convert binary coded decimal into integer
 * @return Integer
 */
uint8_t bcdToInt(uint8_t bcd) {
	return ((bcd >> 4) * 10) + (bcd & 0x0f);
}

/*
 * @brief Read time from rtc and convert it from bcd format to int
 * @return time_t
 *        0 if read fails
 *        unix time if read success
 */
esp_err_t read_time(i2c_port_t i2c_num,struct tm *currentTime) {
	esp_err_t err;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (DS1307_ADDRESS << 1) | I2C_MASTER_WRITE, true /* expect ack */);
	i2c_master_write_byte(cmd, 0x0, true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (DS1307_ADDRESS << 1) | I2C_MASTER_READ, true /* expect ack */);
	uint8_t data[7] = {0};
	i2c_master_read_byte(cmd, &data[0],I2C_MASTER_ACK);
	i2c_master_read_byte(cmd, &data[1],I2C_MASTER_ACK);
	i2c_master_read_byte(cmd, &data[2],I2C_MASTER_ACK);
	i2c_master_read_byte(cmd, &data[3],I2C_MASTER_ACK);
	i2c_master_read_byte(cmd, &data[4],I2C_MASTER_ACK);
	i2c_master_read_byte(cmd, &data[5],I2C_MASTER_ACK);
	i2c_master_read_byte(cmd, &data[6],I2C_MASTER_NACK);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	currentTime->tm_sec  = bcdToInt(data[0]);
	currentTime->tm_min  = bcdToInt(data[1]);
	currentTime->tm_hour = bcdToInt(data[2]);
	currentTime->tm_wday = bcdToInt(data[3])-1;
	currentTime->tm_mday = bcdToInt(data[4]);
	currentTime->tm_mon  = bcdToInt(data[5])-1;
	currentTime->tm_year = bcdToInt(data[6])+100;
	return err;
}



/*
 * @brief update time in rtc
 * @param newTime time to write in ds1307 of time_t type
 *
 */
esp_err_t writetime(i2c_port_t i2c_num, struct tm *newTime) {
	ESP_LOGI(TAG_RTC,"\n  >> writeValue:") ;
	esp_err_t errRc;
	ESP_LOGI(TAG_RTC,"\n Seconds %d", newTime->tm_sec);
	ESP_LOGI(TAG_RTC,"\n min %d", newTime->tm_min);
	ESP_LOGI(TAG_RTC,"\n hours %d", newTime->tm_hour);
	ESP_LOGI(TAG_RTC,"\n day %d", newTime->tm_mday);
	ESP_LOGI(TAG_RTC,"\n mon  %d", newTime->tm_mon+1);
	ESP_LOGI(TAG_RTC,"\n year %d", newTime->tm_year-100);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (DS1307_ADDRESS << 1) | I2C_MASTER_WRITE, true /* expect ack */);
	i2c_master_write_byte(cmd,0x0, true);

	i2c_master_write_byte(cmd, intToBCD(newTime->tm_sec), true);      // seconds
	i2c_master_write_byte(cmd, intToBCD(newTime->tm_min), true);     // minutes
	i2c_master_write_byte(cmd, intToBCD(newTime->tm_hour), true);    // hours
	i2c_master_write_byte(cmd, intToBCD(newTime->tm_wday+1),true);  // week day
	i2c_master_write_byte(cmd, intToBCD(newTime->tm_mday),true);     // date of month
	i2c_master_write_byte(cmd, intToBCD(newTime->tm_mon+1),true);    // month
	i2c_master_write_byte(cmd, intToBCD(newTime->tm_year-100),true); // year
	i2c_master_stop(cmd);
	errRc = i2c_master_cmd_begin(i2c_num, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	return errRc;
}
