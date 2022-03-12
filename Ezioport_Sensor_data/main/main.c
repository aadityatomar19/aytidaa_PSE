#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "errno.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include <time.h>
#include "esp_err.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include <main.h>
#include "sdkconfig.h"


#define SDA_PIN                             21
#define SCL_PIN                             22
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0
#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)
#define TXD2_PIN (GPIO_NUM_16)
#define RXD2_PIN (GPIO_NUM_32)

#define pms0_uart UART_NUM_0
#define pms1_uart UART_NUM_1
static const char *TAG_PMS = "PMS";
static const char *TAG_PMS1 = "PMS1";
static const char *TAG_RX = "RX";
bool bmp280_flag = true;
bmp280_t dev;
bmp280_params_t params;


static const char *TAG_FLASH = "FLASH";
static const char *TAG_I2C = "I2C";
static QueueHandle_t sensor_data_queue;

uint32_t IRAM_ATTR millis() {
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}
/*
 * @brief Initialise UART driver
 * @param baud UART BAUD rate
 * @param uart_no UART port number
 * @param tx_pin UART TX pin GPIO num
 * @param rx_pin UART RX pin GPIO num
 * @param rx_buffer UART RX ring buffer size
 */
void inituart(int baud, uart_port_t uart_no, int tx_pin, int rx_pin, int rx_buffer) {
	const uart_config_t uart_config = {
			.baud_rate = baud,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	uart_param_config(uart_no, &uart_config);
	uart_set_pin(uart_no, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(uart_no, rx_buffer, 0, 0, NULL, 0);
}
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
void getSensorData() {
	while(1){
		pm_data_t pms0Value = {0};
		pm_data_t pms1Value = {0};
		float temp = 0 , humidity = 0;
		float press = 0;
		int iteration = 0;
		int iteration_pms0 = 0;
		int iteration_pms1 = 0;
		int  iteration_temp = 0;
		int iteration_hum = 0;
		int iteration_bmp = 0;
		esp_err_t err;
		unsigned long previousmillis = millis();
		sensor_data_t sensorvalue = {0};
		while ( millis() - previousmillis <= 10000 ) {
			float temperat = 0;
			float humid = 0;
			pm_data_t value0 = {0};
			pm_data_t value1 = {0};
			float bmpTemp= 0 , bmpPress = 0;
			iteration += 1;
			err = read_pms(pms0_uart, &value0);
			if (err == ESP_OK ){
				iteration_pms0 += 1;
				pms0Value.pm10 += value0.pm10;
				pms0Value.pm1_0 += value0.pm1_0;
				pms0Value.pm2_5 += value0.pm2_5;
			}

			err = read_pms(pms1_uart, &value1);
			if (err == ESP_OK ){
				iteration_pms1 += 1;
				pms1Value.pm10 += value1.pm10;
				pms1Value.pm1_0 += value1.pm1_0;
				pms1Value.pm2_5 += value1.pm2_5;
			}
			if (ht21d_read_temperature(I2C_NUM_0, &temperat) == ESP_OK){
				temp += temperat;
				iteration_temp++;
			}
			if (ht21d_read_humidity(I2C_NUM_0, &humid) == ESP_OK){
				humidity += humid;
				iteration_hum++;
			}
			if (bmp280_flag){
				if (bmp280_read_float(I2C_NUM_0, &dev, &bmpTemp, &bmpPress) == ESP_OK){
					iteration_bmp +=1;
					press += bmpPress;
				}
			}
		}
		if(!iteration_bmp){
			sensorvalue.pressure = 0;
		} else {
			sensorvalue.pressure = press / iteration_bmp;
		}
		if ( !iteration_temp){
			sensorvalue.temperature = 0;
		} else {
			sensorvalue.temperature = temp / iteration_temp;
		}
		if ( !iteration_hum){
			sensorvalue.humidity = 0;
		} else {
			sensorvalue.humidity = humidity / iteration_hum;
		}
//		EventBits_t pms_event_bits = xEventGroupGetBits(pms_switch_event_group);
//		if ((pms_event_bits & PMS0_SET_BIT)!= 0){
			if (!iteration_pms0){
				iteration_pms0 = 1;
			}
			sensorvalue.pms_num[0].pm1_0 = pms0Value.pm1_0 / iteration_pms0;
			sensorvalue.pms_num[0].pm2_5 = pms0Value.pm2_5 / iteration_pms0;
			sensorvalue.pms_num[0].pm10 = pms0Value.pm10 / iteration_pms0;
//		} else {
//			sensorvalue.pms_num[0].pm1_0 = 0;
//			sensorvalue.pms_num[0].pm2_5 = 0;
//			sensorvalue.pms_num[0].pm10 = 0;
//		}
//		if ((pms_event_bits & PMS1_SET_BIT)!= 0){
			if (!iteration_pms1){
				iteration_pms1 = 1;
			}
			sensorvalue.pms_num[1].pm1_0 = pms1Value.pm1_0 / iteration_pms1;
			sensorvalue.pms_num[1].pm2_5 = pms1Value.pm2_5 / iteration_pms1;
			sensorvalue.pms_num[1].pm10 = pms1Value.pm10 / iteration_pms1;
//		} else {
//			sensorvalue.pms_num[1].pm1_0 = 0;
//			sensorvalue.pms_num[1].pm2_5 = 0;
//			sensorvalue.pms_num[1].pm10 = 0;
//		}
		ESP_LOGI(TAG_RX, "%d , %d , %d , %d , %d , %d , %f , %f ,  %f",
				sensorvalue.pms_num[0].pm1_0, sensorvalue.pms_num[0].pm2_5, sensorvalue.pms_num[0].pm10,
				sensorvalue.pms_num[1].pm1_0, sensorvalue.pms_num[1].pm2_5, sensorvalue.pms_num[1].pm10,
				sensorvalue.temperature, sensorvalue.humidity,sensorvalue.pressure);
		xQueueOverwrite(sensor_data_queue,&sensorvalue);
	}
}

void app_main() {
	esp_err_t ret;

	// Initialise flash in esp32
	ret = nvs_flash_init();
	if ( ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND ) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	inituart(9600, UART_NUM_0, TXD_PIN, RXD_PIN, RX_BUF_SIZE);
	inituart(9600, UART_NUM_1, TXD2_PIN, RXD2_PIN, RX_BUF2_SIZE);
	sdCard_pins_config_t  pin = {
			.miso_pin = 19,
			.mosi_pin = 23,
			.sclk_pin = 18,
			.cs_pin = 5,
	};
	initI2C(I2C_MODE_MASTER, I2C_NUM_0, SDA_PIN, SCL_PIN, 100000);
	ret = init_sd(&pin);
	if (ret != ESP_OK){
		ESP_LOGI(TAG_FLASH,"sdcard init failed");
	}
	ret = bmp280_init_default_params(&params);
	if (ret != ESP_OK){
		bmp280_flag = false;
	}
	else {
		ret = bmp280_init(I2C_NUM_0, &params, &dev);
		if (ret != ESP_OK){
			bmp280_flag = false;
		}
	}
	sensor_data_queue = xQueueCreate(1,sizeof(sensor_data_t));
//	char data[] = "bc";
//	char filename[] = "abc";
//
//	while(1){
//		float temp =0,hum=0;
//		float bmpTemp = 0,bmpPress = 0;
//		time_t timeVal =0;
//		ret = read_time(I2C_NUM_0, &timeVal);
//		ESP_LOGI(TAG_FLASH,"current time is %ld %s", timeVal,esp_err_to_name(ret));
//		ret = ht21d_read_temperature(I2C_NUM_0, &temp);
//		ESP_LOGI(TAG_FLASH,"current temp is %f %s", temp,esp_err_to_name(ret));
//		ret = ht21d_read_humidity(I2C_NUM_0, &hum);
//		ESP_LOGI(TAG_FLASH,"current humidity is %f %s", hum,esp_err_to_name(ret));
//		if (bmp280_flag){
//			ret = bmp280_read_float(I2C_NUM_0, &dev, &bmpTemp, &bmpPress);
//			ESP_LOGI(TAG_FLASH,"current bmp temp is %f and pressure is %f %s", bmpTemp, bmpPress,esp_err_to_name(ret));
//		}
//		pm_data_t value0 = {0};
//		pm_data_t value1 = {0};
//		pm_data_t pms0Value = {0};
//		pm_data_t pms1Value = {0};
//		ret = read_pms(pms0_uart, &value0);
//		if (ret == ESP_OK ){
//			pms0Value.pm10 = value0.pm10;
//			pms0Value.pm1_0 = value0.pm1_0;
//			pms0Value.pm2_5 = value0.pm2_5;
//		}
//		ret = read_pms(pms1_uart, &value1);
//		if (ret == ESP_OK ){
//			pms1Value.pm10 = value1.pm10;
//			pms1Value.pm1_0 = value1.pm1_0;
//			pms1Value.pm2_5 = value1.pm2_5;
//		}
//		ESP_LOGI(TAG_PMS,"PM1.0=%d PM2.5=%d PM10=%d",pms0Value.pm1_0,pms0Value.pm2_5,pms0Value.pm10);
//		ESP_LOGI(TAG_PMS1,"PM1.0=%d PM2.5=%d PM10=%d",pms1Value.pm1_0,pms1Value.pm2_5,pms1Value.pm10);
//
//
//		ret = write_sd (data, filename);
//		ESP_LOGI(TAG_FLASH, "%s ",esp_err_to_name(ret));
//		vTaskDelay(10000/portTICK_RATE_MS);
//	}
	while(1){
		 getSensorData();
	}
}
