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
#include "stdio.h"
#include "common.h"

#define SDA_PIN                             21
#define SCL_PIN                             22
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0


static const int PMS_BUF_SIZE = 256;
static const int SIM7600_BUF_SIZE = 1024;

#define RESET_PIN 			(GPIO_NUM_19 )
#define TXD0_PIN (GPIO_NUM_1)
#define RXD0_PIN (GPIO_NUM_3)

#define TXD1_PIN (GPIO_NUM_25)
#define RXD1_PIN (GPIO_NUM_26)

#define TXD2_PIN (GPIO_NUM_16)
#define RXD2_PIN (GPIO_NUM_32)

#define pms0_uart UART_NUM_0
#define SIM7600_uart UART_NUM_1
#define pms1_uart UART_NUM_2

//uint32_t IRAM_ATTR millis() {
//	return xTaskGetTickCount() * portTICK_PERIOD_MS;
//}
int Counter = 0;                //integer to store counter values to update RTC time
bool Rtc = true;                     //bool to update RTC time
static const char *TAG_SIM7600 = "SIM7600";
static const char *TAG_PMS = "PMS";
static const char *TAG_PMS1 = "PMS1";
static const char *TAG_RX = "RX";
static const char *TAG_RTC = "RTC";
bool bmp280_flag = true;
bmp280_t dev;
bmp280_params_t params;


char APN[] = "www";                           //APN
char Client_Id[] = "ez4g07";                  //Client ID
char Username[] = "ez4g01";                   //Username
char Password[] = "ez4g01xxx";                //Password
char MQTT_Server[] = "104.196.168.114:1883";  //MQTT broker with port
char *Pub_Topic = "ezioport/test";       //Topic to publish data
char Payload[300] = "";
char mac_id[50] = "";
#define mqtt_data "{\"PM1\":%d,\"PM2_5\":%d,\"PM10\":%d,\"PM1_x\":%d,\"PM2.5_x\":%d,\"PM10_x\":%d,\"RH\":%.02f,\"T\":%.02f,\"P\":%.02f,\"LAT\":%.02f,\"LON\":%.02f,\"MAC\":\"%s\"}"

static const char *TAG_FLASH = "FLASH";
static const char *TAG_I2C = "I2C";
static QueueHandle_t sensor_data_queue;
static QueueHandle_t time_queue;     //Queue to exchange RTC time between two tasks
//static QueueHandle_t data_queue;     //Queue to exchange Sensors data between two tasks

/*structure to exchange data between two tasks*/
typedef struct
{
	uint32_t PM1_0_PMS1;
	uint32_t PM2_5_PMS1;
	uint32_t PM10_0_PMS1;
	uint32_t PM1_0_PMS2;
	uint32_t PM2_5_PMS2;
	uint32_t PM10_0_PMS2;
	float Humidity;
	float Temp;
	float Pressure;
	float latitude;
	float longitude;
}Sensor_t;


//uint32_t IRAM_ATTR millis() {
//	return xTaskGetTickCount() * portTICK_PERIOD_MS;
//}
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
		Counter++;
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
		sensorvalue.latitude = GPSlat();
		sensorvalue.longitude = GPSlog();

		ESP_LOGI(TAG_RX, "%d ,%d ,%d ,%d ,%d ,%d ,%f ,%f ,%f, %f, %f ",
				sensorvalue.pms_num[0].pm1_0, sensorvalue.pms_num[0].pm2_5, sensorvalue.pms_num[0].pm10,
				sensorvalue.pms_num[1].pm1_0, sensorvalue.pms_num[1].pm2_5, sensorvalue.pms_num[1].pm10,
				sensorvalue.temperature, sensorvalue.humidity,sensorvalue.pressure,sensorvalue.latitude,sensorvalue.longitude);
		struct tm currentTime;
		read_time(I2C_NUM_0 ,&currentTime);
		ESP_LOGI(TAG_RTC,"%d,%d,%d,%d,%d,%d,",currentTime.tm_year,currentTime.tm_mon,currentTime.tm_mday,currentTime.tm_hour,currentTime.tm_min,currentTime.tm_sec);
		struct tm received_time;
		/*loop to check availability of updated time to update RTC time*/
		if ( xQueueReceive(time_queue, &received_time, 500) ) {
			//		        rtc.adjust(DateTime(received_time.Year, received_time.Month, received_time.Day, received_time.Hour, received_time.Minute, received_time.Second));
			//		        Serial.println("RTC time is updated");
			//		        Serial.printf("day,month,year,hour,min,sec,%d-%d-%d %d:%d:%d", received_time.Day, received_time.Month, received_time.Year, received_time.Hour, received_time.Minute, received_time.Second);
			err= writetime( I2C_NUM_0, &received_time);
			if(err!=ESP_OK){
				ESP_LOGI(TAG_RTC,"RTC is not updated %s",esp_err_to_name(err));
			}

		}
		xQueueOverwrite(sensor_data_queue,&sensorvalue);
	}
}
void sim7600(){
	while (1) {
		struct tm UpdatedTime;
		bool pubfail = false;
		/*loop to Reset SIM7600 and check network*/
		if (PowerOn()) {
			//	      sync_network_time();
			/*loop to Switch ON internet*/
			if (Internet(APN)) {
				/*loop to connect to MQTT broker and publish data*/
				while (!pubfail) {
					GPSPositioning();
					sensor_data_t received_data_mqtt;
					/*loop to check availability of sensors data from task: Get_Sensors_data*/
					if ( xQueueReceive(sensor_data_queue, &received_data_mqtt, 30000) ) {
						sprintf(Payload, mqtt_data,received_data_mqtt.pms_num[0].pm1_0, received_data_mqtt.pms_num[0].pm2_5, received_data_mqtt.pms_num[0].pm10,
								received_data_mqtt.pms_num[1].pm1_0, received_data_mqtt.pms_num[1].pm2_5, received_data_mqtt.pms_num[1].pm10,
								received_data_mqtt.humidity, received_data_mqtt.temperature,received_data_mqtt.pressure,received_data_mqtt.latitude,received_data_mqtt.longitude,mac_id);
						printf("Payload= %s",Payload );
						/*loop to connect to MQTT broker*/
						if (MQTT_Start(MQTT_Server, Username, Password, Client_Id)) {
							/*loop to publish Sensors data*/
							if (MQTT_Pub_data(Pub_Topic, Payload)) {
								printf("Data published successfully");
							}
							else {
								printf("pub failed ");
								pubfail = true;
							}
							MQTT_Stop();
						}
						else
							pubfail = true;
					}
					else {
						printf("data not found");
					}
					/*loop to get updated time from SIM7600 and overwrite it in a queue*/
					if ((Counter >= 1) || Rtc) {
						getTime(&UpdatedTime);
						printf("%d,%d,%d,%d,%d,%d",UpdatedTime.tm_year,UpdatedTime.tm_mon,UpdatedTime.tm_mday,UpdatedTime.tm_hour,UpdatedTime.tm_min,UpdatedTime.tm_sec);
						xQueueOverwrite(time_queue, &UpdatedTime);
						Rtc = false;
						Counter = 0;
					}
				}
			}
		}
		else
			GPSPositioning();
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
	uint8_t mac_addr[8]  = {0};
	uint8_t mac[6];
	ret = esp_efuse_mac_get_default(mac);
	if (ret != ESP_OK) {
		ESP_LOGI(TAG_SIM7600, "Did not get the mac ");
	} else {
		esp_base_mac_addr_set(mac);
	}
	esp_base_mac_addr_get(mac_addr);
	sprintf(mac_id, "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0],
			mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

	inituart(9600, pms0_uart, TXD0_PIN, RXD0_PIN, PMS_BUF_SIZE);
	inituart(9600, pms1_uart, TXD2_PIN, RXD2_PIN, PMS_BUF_SIZE);
	inituart(115200, SIM7600_uart, TXD1_PIN, RXD1_PIN, SIM7600_BUF_SIZE);
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
	time_queue = xQueueCreate(1,sizeof(struct tm));
	/*  Creates getSensorData with a stack size of 4096 bytes at priority 4 */
	xTaskCreate(getSensorData, "get_sensor_data", 1024*4, NULL, 4, NULL);
	xTaskCreate(sim7600,"SIM7600",8192,NULL,1,NULL);
}
