#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_err.h"

#define SDA_PIN                             21
#define SCL_PIN                             22
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0

#define HTU21D_ADDR                              0x40
#define DS1307_ADDRESS                           0x68
#define TRIG_TEMP_MEASUREMENT_HOLD               0xE3
#define TRIG_HUMD_MEASUREMENT_HOLD               0xE5
#define TRIG_TEMP_MEASUREMENT_NOHOLD             0xF3
#define TRIG_HUMD_MEASUREMENT_NOHOLD             0xF5
#define WRITE_USER_REG                           0xE6
#define READ_USER_REG                            0xE7
#define SOFT_RESET                               0xFE

float previous_temp_value = 0;
float previous_hum_value = 0;
bool crc_flag = true;
char rcv_buffer[200];

typedef struct {
	float temp_data;
	bool  temp_flag;
}temp_data_t;

typedef struct {
	float hum_data;
	bool  hum_flag;
}hum_data_t;

static const char *TAG_HTU = "HTU";
static const char *TAG_I2C = "I2C";
void initI2C();
hum_data_t ht21d_read_humidity();
temp_data_t ht21d_read_temperature();
uint16_t read_value(uint8_t command);
bool is_crc_valid(uint16_t value, uint8_t crc);

/*
 * @brief Initialise I2C Driver
 * @param i2c_mode I2C mode
 * @param port_no I2C port number
 * @param sda_pin GPIO number for I2C sda signal
 * @param scl_pin GPIO number for I2C scl signal
 * @param clock_freq I2C clock frequency for master mode
 */
void initI2C(i2c_mode_t i2c_mode, i2c_port_t port_no, int sda_pin, int scl_pin, uint32_t clock_freq) {
	esp_err_t ret;
	i2c_config_t conf;
	conf.mode = i2c_mode;
	conf.sda_io_num = sda_pin;
	conf.scl_io_num = scl_pin;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = clock_freq;
	conf.clk_flags = 0;
	ret = i2c_param_config(port_no, &conf);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG_I2C, "Failed to initialize I2C parameters.");
		return;
	}
	ret = i2c_driver_install(port_no, i2c_mode, 0, 0, 0);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG_I2C, "Failed to install I2C drivers");
		return;
	}
}

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
uint16_t read_value(uint8_t command) {
	i2c_port_t i2c_num = I2C_EXAMPLE_MASTER_NUM;
	esp_err_t ret;
	crc_flag = true;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd,
			(HTU21D_ADDR << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, command, true);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if ( ret != ESP_OK ) {
		crc_flag = false;
		return 0;
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
	if ( ret != ESP_OK ) {
		crc_flag = false;
		return 0;
	}
	uint16_t raw_value = ((uint16_t) msb << 8) | ( (uint16_t) lsb);

	if ( !is_crc_valid(raw_value, crc) ) {
		crc_flag = false;
	} else {
		crc_flag = true;
	}
	return raw_value & 0xFFFC;
}

/*
 * @brief convert raw temperatue value to *C if value is valid otherwise returns 0
 * @return temp_data_t (temp value and crc validity)
 */
temp_data_t ht21d_read_temperature() {
	temp_data_t temp_value ={0};
	uint16_t raw_temperature = read_value(TRIG_TEMP_MEASUREMENT_NOHOLD);
	float temp = (raw_temperature *(175.72 / 65536.0)) - 46.85;

	if ( crc_flag ) {
		temp_value.temp_data = temp;
		temp_value.temp_flag = true;

	} else {
		temp_value.temp_data = 0;
		temp_value.temp_flag = false;
	}
	return temp_value;
}

/*
 * @brief convert raw humidity value to %rh if value is valid otherwise returns 0
 * @return hum_data_t (humidity value and crc validity)
 */
hum_data_t ht21d_read_humidity() {
	hum_data_t hum_value = {0};
	uint16_t raw_humidity = read_value(TRIG_HUMD_MEASUREMENT_NOHOLD);
	float hum = ((raw_humidity * 125.0 / 65536.0) - 6.0);
	if ( crc_flag ) {
		hum_value.hum_data = hum;
		hum_value.hum_flag = true;

	} else {
		hum_value.hum_data = 0;
		hum_value.hum_flag = false;
	}
	return hum_value;
}


void app_main(void)
{
	esp_err_t ret;
	// Initialise flash in esp32
	ret = nvs_flash_init();
	if ( ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND ) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	initI2C(I2C_MODE_MASTER, I2C_NUM_0, SDA_PIN, SCL_PIN, 100000);
	while(1){
		float temp = 0 , humidity = 0;
		temp_data_t temperat = {0};
		hum_data_t humid = {0};
		temperat = ht21d_read_temperature();
		temp = temperat.temp_data;
		//		if ( temperat.temp_flag ) iteration_temp++;
		humid = ht21d_read_humidity();
		humidity = humid.hum_data;
		//		if ( humid.hum_flag ) iteration_hum++;
		ESP_LOGI(TAG_HTU,"Humidity=%f Temp=%f",humidity,temp);
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

