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
#include "driver/rmt.h"
#include "led_strip.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_wifi.h"
//#include "protocol_examples_common.h"


#if CONFIG_EXAMPLE_CONNECT_WIFI
#include "esp_wifi.h"
#endif

#define SSID "Qwerty"
#define PASSWORD "yesnomaybem8"

#define SDA_PIN                             21   		//SDA
#define SCL_PIN                             22			//SCL
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0

#define MOUNT_POINT "/sdcard"

static const int PMS_BUF_SIZE = 256;
static const int SIM7600_BUF_SIZE = 1024;


#define RESET_PIN (GPIO_NUM_19)  		//Reset pin for SIM7600
#define TXD0_PIN (GPIO_NUM_1)			//Tx for SIM7600
#define RXD0_PIN (GPIO_NUM_3)			//Rx for SIM7600

#define TXD1_PIN (GPIO_NUM_25)			//Tx for PMS5003 1
#define RXD1_PIN (GPIO_NUM_26)			//Rx for PMS5003 1

#define TXD2_PIN (GPIO_NUM_16)			//Tx for PMS5003 2
#define RXD2_PIN (GPIO_NUM_32)			//Rx for PMS5003 2

#define pms0_uart UART_NUM_0			//UART 0
#define SIM7600_uart UART_NUM_1			//UART 1
#define pms1_uart UART_NUM_2			//UART 2

#define DEFAULT_TIME_0 0
#define DEFAULT_TIME 3600				// Default time for PMS5003 toggle
long pms0_time = DEFAULT_TIME_0;			//Default time PMS5003_1 time
long pms1_time = DEFAULT_TIME_0;			//Default time PMS5003_2 time
long b_pms_time = DEFAULT_TIME;			//Default time PMS5003_1 and PMS5003_2 time
long fota=0;

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define STRIP_LED_NUMBER 1				//LED Number of Strip
#define RGB_GPIO GPIO_NUM_12			//LED GPIO number
#define PMS0_SET_PIN GPIO_NUM_27		//GPIO to toggle PMS5003_1
#define PMS1_SET_PIN GPIO_NUM_33		//GPIO to toggle PMS5003_2

#define GPIO_OUTPUT_PIN_SEL1 (1ULL << PMS0_SET_PIN)
#define GPIO_OUTPUT_PIN_SEL2 (1ULL << PMS1_SET_PIN)

#define OTA_URL_SIZE 256

static const char *TAG_SIM7600 = "SIM7600";
static const char *TAG_RX = "RX";
static const char *TAG_RTC = "RTC";
static const char *TAG_SD = "SD";
static const char *TAG_FLASH = "FLASH";
static const char *TAG_I2C = "I2C";
static const char *TAG_CONFIG = "CONFIG";
static const char *TAG_ota = "simple_ota_example";
static const char *TAG_WiFi = "wifi_conection";

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

bool SD_init= true;
bool bmp280_flag = true;
bmp180_t dev;
bool flag=true;
int interval = 9500;							//Interval to Average data of all sensors
unsigned long Counter = 0;                              	//integer to store counter values to update RTC time
bool Rtc = true;                              	//bool to update RTC time
bool Start=true;							  	//bool to write legendsin file
int dd=0;										//Integer to store date
char APN[] = "www";                           	//APN
char Client_Id[] = "ez4g10";                   	//Client ID
char Username[] = "ez4g01";                   	//Username
char Password[] = "ez4g01xxx";                	//Password
char MQTT_Server[] = "104.196.168.114:1883";  	//MQTT broker with port
char *Pub_Topic = "ezioport/drnaveen";        	//Topic to publish data
char Payload[300] = "";							//Array to store payload
char mac_id[50] = "";							//Array to store MAC ID
#define mqtt_data "{\"PM1\":%d,\"PM2.5\":%d,\"PM10\":%d,\"PM1_x\":%d,\"PM2.5_x\":%d,\"PM10_x\":%d,\"RH\":%.02f,\"T\":%.02f,\"P\":%.02f,\"LAT\":%.02f,\"LON\":%.02f,\"MAC\":\"%s\"}"
char *datalegend =   "PM1,PM2.5,PM10,PM1X,PM2.5X,PM10X,Humidity,Temperature,Pressure,LAT,LONG,ts,MAC\n";   // Legends to write into file

static QueueHandle_t sensor_data_queue;				//Queue to exchange Sensor data between two tasks
static QueueHandle_t SD_data_queue;					//Queue to exchange SD data between two tasks
static QueueHandle_t time_queue;     				//Queue to exchange RTC time between two tasks
static const int PMS0_SET_BIT = BIT0;
static const int PMS1_SET_BIT = BIT1;
static const int NETWORK_SET_BIT = BIT0;
static const int SD_WRITE_SET_BIT = BIT1;
static const int CONNECTED_BIT = BIT0;
static const int FOTA_BIT = BIT1;
static const int FOTA_SUCCESS = BIT2;

static EventGroupHandle_t pms_switch_event_group;
static EventGroupHandle_t led_event_group;
static EventGroupHandle_t wifi_event_group;

/*structure to exchange sensor data between two tasks*/
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
void read_config(long *pms0_t, long *pms1_t, long *b_Pms_t, long *fota);

void wifi_connect(){
	wifi_config_t cfg = {
			.sta = {
					.ssid = SSID,
					.password = PASSWORD,
			},
	};
	ESP_ERROR_CHECK( esp_wifi_disconnect() );
	ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg) );
	ESP_ERROR_CHECK( esp_wifi_connect() );
}
/*
 * @brief initialise gpio pins for PMS5003_1 & PMS5003_2 toggle
 */
void init_gpio() {
	gpio_config_t io_conf2;
	//disable interrupt
	io_conf2.intr_type = GPIO_INTR_DISABLE;
	//set as output mode
	io_conf2.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf2.pin_bit_mask = GPIO_OUTPUT_PIN_SEL1;
	//disable pull-down mode
	io_conf2.pull_down_en = 0;
	//disable pull-up mode
	io_conf2.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf2);

	gpio_config_t io_conf3;
	//disable interrupt
	io_conf3.intr_type = GPIO_INTR_DISABLE;
	//set as output mode
	io_conf3.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf3.pin_bit_mask = GPIO_OUTPUT_PIN_SEL2;
	//disable pull-down mode
	io_conf3.pull_down_en = 0;
	//disable pull-up mode
	io_conf3.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf3);
}
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch(event->event_id) {
	case SYSTEM_EVENT_STA_START:
		wifi_connect();
		ESP_LOGI(TAG_WiFi, "Connecting.......");
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		ESP_LOGI(TAG_WiFi, "connected");
		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		ESP_LOGI(TAG_WiFi, "Retrying to connect");
		esp_wifi_connect();
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
		break;
	default:
		break;
	}
	return ESP_OK;
}
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
	switch (evt->event_id) {
	case HTTP_EVENT_ERROR:
		ESP_LOGD(TAG_ota, "HTTP_EVENT_ERROR");
		break;
	case HTTP_EVENT_ON_CONNECTED:
		ESP_LOGD(TAG_ota, "HTTP_EVENT_ON_CONNECTED");
		break;
	case HTTP_EVENT_HEADER_SENT:
		ESP_LOGD(TAG_ota, "HTTP_EVENT_HEADER_SENT");
		break;
	case HTTP_EVENT_ON_HEADER:
		ESP_LOGD(TAG_ota, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
		break;
	case HTTP_EVENT_ON_DATA:
		ESP_LOGD(TAG_ota, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
		break;
	case HTTP_EVENT_ON_FINISH:
		ESP_LOGD(TAG_ota, "HTTP_EVENT_ON_FINISH");
		break;
	case HTTP_EVENT_DISCONNECTED:
		ESP_LOGD(TAG_ota, "HTTP_EVENT_DISCONNECTED");
		break;
	}
	return ESP_OK;
}
static void initialise_wifi(void)
{
	esp_log_level_set("wifi", ESP_LOG_NONE); // disable wifi driver logging
	tcpip_adapter_init();
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
	ESP_ERROR_CHECK( esp_wifi_start() );
	esp_err_t ret = tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA ,"icircuit");
	if(ret != ESP_OK ){
		ESP_LOGE(TAG_WiFi,"failed to set hostname:%d",ret);
	}
}


void FOTA_Update()
{
	ESP_LOGI(TAG_ota, "Starting OTA example");

	esp_http_client_config_t config = {
			.url = "https://gitlab.com/aditomar19/fota/-/raw/main/Ezioport_Sensor_data.bin?inline=false",
			.cert_pem = (char *)server_cert_pem_start,
			.event_handler = _http_event_handler,
			.keep_alive_enable = true,
	};
	esp_err_t ret = esp_https_ota(&config);
	if (ret == ESP_OK) {
		xEventGroupSetBits(wifi_event_group, FOTA_SUCCESS);
		vTaskDelay(2000/portTICK_RATE_MS);
		esp_restart();
	} else {
		ESP_LOGE(TAG_ota, "Firmware upgrade failed");
		xEventGroupClearBits(wifi_event_group, FOTA_SUCCESS);
	}
}
void FOTA(void *pvParam){
	while(1){
		printf("FOTA task started now \n");
		xEventGroupWaitBits(wifi_event_group,CONNECTED_BIT,false,true,portMAX_DELAY);
		tcpip_adapter_ip_info_t ip_info;
		ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
		printf("IP :  %s\n", ip4addr_ntoa(&ip_info.ip));
		FOTA_Update();
	}
}
/*
 * @brief Switching PMS5003 on/off by pulling up/down set pin of pms
 * */
void pms_toggle(){
	int i = 0;
	ESP_LOGI(TAG_CONFIG,"Toggle time for PMS are %ld,  %ld,  %ld", pms0_time, pms1_time, b_pms_time);
	while (1){
		if (i == 0){
			i= 1;
			gpio_set_level(PMS1_SET_PIN,0);
			gpio_set_level(PMS0_SET_PIN,1);
			xEventGroupClearBits(pms_switch_event_group,PMS1_SET_BIT);
			xEventGroupSetBits(pms_switch_event_group,PMS0_SET_BIT);
			//			xEventGroupClearBits(pms_switch_event_group,BOTH_PMS_SET_BIT);
			vTaskDelay(pms0_time*1000 / portTICK_RATE_MS);

		}
		else if ( i ==1){
			i= 2;
			gpio_set_level(PMS0_SET_PIN,0);
			gpio_set_level(PMS1_SET_PIN,1);
			xEventGroupClearBits(pms_switch_event_group,PMS0_SET_BIT);
			xEventGroupSetBits(pms_switch_event_group,PMS1_SET_BIT);
			vTaskDelay(pms1_time*1000 / portTICK_RATE_MS);

			//			xEventGroupClearBits(pms_switch_event_group,BOTH_PMS_SET_BIT);
		}
		else {
			i = 0 ;
			gpio_set_level(PMS0_SET_PIN,1);
			gpio_set_level(PMS1_SET_PIN,1);
			//			xEventGroupSetBits(pms_switch_event_group,BOTH_PMS_SET_BIT);
			xEventGroupSetBits(pms_switch_event_group,PMS0_SET_BIT);
			xEventGroupSetBits(pms_switch_event_group,PMS1_SET_BIT);
			vTaskDelay(b_pms_time*1000 / portTICK_RATE_MS);

		}
		vTaskDelay(50 / portTICK_RATE_MS);

	}
}
/*
 * @brief Function to read both pms5003's run time in 3 different configuration
 *        i.e. Only PMS1 run time , PMS2 run time and Both PMS simultaneous run time
 *        If config file doesn't exist or time is invalid then pms configurationwwill be set to
 *        default time i.e 3600 sec
 * @param    *pms0_time,*pms1_time,*both_pms_time  pointer to save configuration time
 */
void read_config(long *pms0_time, long *pms1_time, long *both_pms_time, long *fota ){
	char *pms0_time_string = (char*)malloc(sizeof(char));
	char *pms1_time_string = (char*)malloc(sizeof(char));
	char *both_pms_time_string = (char*)malloc(sizeof(char));
	char *fota_string = (char*)malloc(sizeof(char));

	FILE* config_file = fopen(MOUNT_POINT"/config.txt","r");
	if (config_file == NULL) {
		ESP_LOGE(TAG_SD, "Failed to open config file for reading");
		return;
	}
	int i = 0;
	int tot = 0;
	char line[5][100];
	while(fgets(line[i],sizeof(line[i]),config_file) != NULL){
		char* pos = strchr(line[i],'\r');
		if (pos){
			*pos = '\0';
		}
		i++;
	}
	fclose(config_file);
	tot = i;
	for(i=0;i<tot;i++){
		char* token;
		char* token1;
		char* temp_var;
		char pos[200] ;
		char *eptr;
		strcpy(pos,line[i]);

		if( strncmp("PMS1_ON_TIME :",line[i],14)==0)
		{

			token = strtok(pos,";");
			temp_var = token;
			token1 = strtok(temp_var,":");

			while(token1 !=0){
				temp_var = token1;
				token1 = strtok(0,":");
			}
			strcpy(pms0_time_string ,temp_var);
			*pms0_time = strtol(pms0_time_string, &eptr, 10);
			if (*pms0_time == 0){
				if (strcmp(eptr, pms0_time_string) == 0){
					ESP_LOGI(TAG_SD,"HELLO HELLO HELLO !!!!! error occurred");
					*pms0_time = DEFAULT_TIME_0;
				}
				if (errno ==EINVAL){
					ESP_LOGI(TAG_SD,"Conversion error occurred: %d", errno);
					*pms0_time = DEFAULT_TIME_0;
				}

			}
			if ((*pms0_time == LONG_MIN) || (*pms0_time == LONG_MAX)){
				if (errno == ERANGE){
					ESP_LOGI(TAG_SD,"The value provided was out of range");
					*pms0_time = DEFAULT_TIME_0;
				}
			}
			ESP_LOGI(TAG_SD,"PMS1_ON_TIME variable %s  >> %s", temp_var , pms0_time_string);

		}
		else if (strncmp("PMS2_ON_TIME :",line[i],14)==0){
			token = strtok(pos,";");
			temp_var = token;
			token1 = strtok(temp_var,":");

			while(token1 !=0){
				temp_var = token1;
				token1 = strtok(0,":");
			}
			strcpy(pms1_time_string,temp_var);
			*pms1_time = strtol(pms1_time_string, &eptr, 10);
			if (*pms1_time == 0){
				if (strcmp(eptr, pms1_time_string) == 0){
					ESP_LOGI(TAG_SD,"HELLO HELLO HELLO !!!!! error occurred");
					*pms1_time = DEFAULT_TIME_0;
				}
				else if (errno ==EINVAL){
					ESP_LOGI(TAG_SD,"Conversion error occurred: %d", errno);
					*pms1_time = DEFAULT_TIME_0;
				}
			}
			if ((*pms1_time == LONG_MIN) || (*pms1_time == LONG_MAX)){
				if (errno == ERANGE){
					ESP_LOGI(TAG_SD,"The value provided was out of range");
					*pms1_time = DEFAULT_TIME_0;
				}
			}

			ESP_LOGI(TAG_SD,"PMS2_ON_TIME variable %s >> %s ", temp_var,pms1_time_string);
		}
		else if (strncmp("BOTH_PMS_ON_TIME :",line[i],18)==0){
			token = strtok(pos,";");
			temp_var = token;
			token1 = strtok(temp_var,":");

			while(token1 !=0){
				temp_var = token1;
				token1 = strtok(0,":");
			}
			strcpy(both_pms_time_string,temp_var) ;
			*both_pms_time = strtol(both_pms_time_string, &eptr, 10);
			if (*both_pms_time == 0){
				ESP_LOGI(TAG_SD,"Conversion error occurred: %s", strerror(errno));
				if (strcmp(eptr, both_pms_time_string) == 0){
					ESP_LOGI(TAG_SD,"HELLO HELLO HELLO !!!!! error occurred");
					*both_pms_time = DEFAULT_TIME;
				}
				else if (errno ==EINVAL){
					ESP_LOGI(TAG_SD,"Conversion error occurred: %d", errno);
					*both_pms_time = DEFAULT_TIME;
				}
			}
			if ((*both_pms_time == LONG_MIN) || (*both_pms_time == LONG_MAX)){
				if (errno == ERANGE){
					ESP_LOGI(TAG_SD,"The value provided was out of range");
					*both_pms_time = DEFAULT_TIME;
				}
			}
			ESP_LOGI(TAG_SD,"BOTH_PMS_ON_TIME %s >> %s ", temp_var,both_pms_time_string);
		}
		else if( strncmp("FOTA :",line[i],06)==0)
		{

			token = strtok(pos,";");
			temp_var = token;
			token1 = strtok(temp_var,":");

			while(token1 !=0){
				temp_var = token1;
				token1 = strtok(0,":");
			}
			strcpy(fota_string ,temp_var);
			*fota = strtol(fota_string, &eptr, 10);
			if (*fota == 0){
				if (strcmp(eptr, fota_string) == 0){
					ESP_LOGI(TAG_SD,"HELLO HELLO HELLO !!!!! error occurred");
					*fota = DEFAULT_TIME_0;
				}
				if (errno ==EINVAL){
					ESP_LOGI(TAG_SD,"Conversion error occurred: %d", errno);
					*fota = DEFAULT_TIME_0;
				}

			}
			if ((*fota == LONG_MIN) || (*fota == LONG_MAX)){
				if (errno == ERANGE){
					ESP_LOGI(TAG_SD,"The value provided was out of range");
					*fota = DEFAULT_TIME_0;
				}
			}
			ESP_LOGI(TAG_SD,"fota variable %s  >> %s", temp_var , fota_string);

		}
	}
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
/*
 * @brief Collecting Sensor data for defined interval, doing average
 *        and sending average data in sensor_data_queue
 */
void getSensorData() {
	while(1){
		ESP_LOGI(TAG_RX, "FREE INTERNAL HEAP MQTT destroy end %d", esp_get_free_internal_heap_size());
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
		ESP_LOGI(TAG_RX,"TIME MIllis %d",millis());
		while ( millis() - previousmillis <=interval ) {
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
				if (bmp180_read_float(I2C_NUM_0, &dev, &bmpTemp, &bmpPress) == ESP_OK){
					iteration_bmp +=1;
					press += bmpPress;
				}
			}
			sensorvalue.latitude = GPSlat();
			sensorvalue.longitude = GPSlog();
			struct tm received_time;
			/*loop to check availability of updated time to update RTC time*/
			if ( xQueueReceive(time_queue, &received_time, 100) ) {
				received_time.tm_year -=1900;
				received_time.tm_mon -= 1;
				err= writetime( I2C_NUM_0, &received_time);
				if(err!=ESP_OK){
					ESP_LOGI(TAG_RTC,"RTC is not updated %s",esp_err_to_name(err));
				}
				else{
					ESP_LOGI(TAG_RTC,"RTC is updated");
				}
			}
			vTaskDelay(112/portTICK_RATE_MS);
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
		EventBits_t pms_event_bits = xEventGroupGetBits(pms_switch_event_group);
		if ((pms_event_bits & PMS0_SET_BIT)!= 0){
			if (!iteration_pms0){
				iteration_pms0 = 1;
			}
			sensorvalue.pms_num[0].pm1_0 = pms0Value.pm1_0 / iteration_pms0;
			sensorvalue.pms_num[0].pm2_5 = pms0Value.pm2_5 / iteration_pms0;
			sensorvalue.pms_num[0].pm10 = pms0Value.pm10 / iteration_pms0;
		} else {
			sensorvalue.pms_num[0].pm1_0 = 0;
			sensorvalue.pms_num[0].pm2_5 = 0;
			sensorvalue.pms_num[0].pm10 = 0;
		}
		if ((pms_event_bits & PMS1_SET_BIT)!= 0){
			if (!iteration_pms1){
				iteration_pms1 = 1;
			}
			sensorvalue.pms_num[1].pm1_0 = pms1Value.pm1_0 / iteration_pms1;
			sensorvalue.pms_num[1].pm2_5 = pms1Value.pm2_5 / iteration_pms1;
			sensorvalue.pms_num[1].pm10 = pms1Value.pm10 / iteration_pms1;
		} else {
			sensorvalue.pms_num[1].pm1_0 = 0;
			sensorvalue.pms_num[1].pm2_5 = 0;
			sensorvalue.pms_num[1].pm10 = 0;
		}
		xQueueOverwrite(sensor_data_queue,&sensorvalue);
		ESP_LOGI(TAG_RX,"TIME millis of data write%d",millis());
		struct tm currentTime={0};
		SD_sensor_t SD_Data={0};
		read_time(I2C_NUM_0 ,&currentTime);
		time_t systime = mktime(&currentTime);
		ESP_LOGI(TAG_RTC,"Current time is %ld",  systime);
		currentTime.tm_year += 1900;
		currentTime.tm_mon += 1;
		struct timeval Esptime;
		Esptime.tv_sec = systime;
		Esptime.tv_usec = 0;
		sntp_sync_time(&Esptime);
		SD_Data.pms[0].pm1_0=sensorvalue.pms_num[0].pm1_0;
		SD_Data.pms[0].pm2_5=sensorvalue.pms_num[0].pm2_5;
		SD_Data.pms[0].pm10=sensorvalue.pms_num[0].pm10;
		SD_Data.pms[1].pm1_0=sensorvalue.pms_num[1].pm1_0;
		SD_Data.pms[1].pm2_5=sensorvalue.pms_num[1].pm2_5;
		SD_Data.pms[1].pm10=sensorvalue.pms_num[1].pm10;
		SD_Data.humidity=sensorvalue.humidity;
		SD_Data.temperature=sensorvalue.temperature;
		SD_Data.pressure=sensorvalue.pressure;
		SD_Data.latitude=sensorvalue.latitude;
		SD_Data.longitude=sensorvalue.longitude;
		SD_Data.Day=currentTime.tm_mday;
		SD_Data.Month=currentTime.tm_mon;
		SD_Data.Year=currentTime.tm_year;
		SD_Data.Hour=currentTime.tm_hour;
		SD_Data.Minute=currentTime.tm_min;
		SD_Data.Second=currentTime.tm_sec;
		SD_Data.MAC=mac_id;
		xQueueOverwrite(SD_data_queue,&SD_Data);
		ESP_LOGI(TAG_RX,"TIME MIllis after loop %d",millis());
	}
}
/*
 * @brief  Writes Sensors data into sd card
 */
void Data_Logger(){
	esp_err_t err;
	SD_sensor_t Data;
	while(1){
		if (xQueueReceive(SD_data_queue, &Data, interval)) {
			//			ESP_LOGI(TAG_RTC,"%d,%d,%d,%d,%d,%d,",Data.Year,Data.Month,Data.Day,Data.Hour,Data.Minute,Data.Second);
			char dtfilename[50] = "";
			char filename[50] = "";
			char data[200] = "";
			sprintf(data,"%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%d-%d-%d %d:%d:%d,%s",
					Data.pms[0].pm1_0, Data.pms[0].pm2_5, Data.pms[0].pm10,
					Data.pms[1].pm1_0, Data.pms[1].pm2_5, Data.pms[1].pm10,
					Data.temperature, Data.humidity,Data.pressure,Data.latitude,
					Data.longitude,Data.Day,Data.Month,Data.Year,
					Data.Hour,Data.Minute,Data.Second,mac_id);
			ESP_LOGI(TAG_RX,"data: %s",data);
			sprintf(dtfilename,"%02d%02d%04d",Data.Day,Data.Month,Data.Year);
			sprintf(filename,"%s/%02d%02d%04d.csv",MOUNT_POINT,Data.Day,Data.Month,Data.Year);
			if (SD_init){
				if ((Data.Day != dd) || Start) {
					dd=Data.Day;
					Start=false;
					FILE* f = fopen(filename,"r");
					fclose(f);
					if (f == NULL) {
						ESP_LOGI(TAG_SD, "Failed to open file for writing legends");
						ESP_LOGI(TAG_SD, "writing legends");
						err = write_sd (datalegend, dtfilename);
						if(err==ESP_OK){
							ESP_LOGE(TAG_SD, "legends updated");
						}
						ESP_LOGI(TAG_SD, "%s ",esp_err_to_name(err));
					}else {
						err = write_sd (data, dtfilename);
						if(err==ESP_OK){
							ESP_LOGE(TAG_SD, "data updated 1");
							xEventGroupSetBits(led_event_group,SD_WRITE_SET_BIT);
						}
						ESP_LOGI(TAG_FLASH, "%s ",esp_err_to_name(err));
					}
				}else {
					FILE* f = fopen(filename,"r");
					fclose(f);
					if (f != NULL) {
						err = write_sd (data, dtfilename);
						if(err==ESP_OK){
							ESP_LOGI(TAG_SD, "data updated");
							xEventGroupSetBits(led_event_group,SD_WRITE_SET_BIT);
						} else {
							xEventGroupClearBits(led_event_group,SD_WRITE_SET_BIT);
						}
						ESP_LOGI(TAG_FLASH, "%s ",esp_err_to_name(err));
					}
					else{
						ESP_LOGE(TAG_SD, "data  not updated");
						xEventGroupClearBits(led_event_group,SD_WRITE_SET_BIT);
					}
				}
			}
		}
	}
}
/*
 * @brief This function shows status by RGB LED
 */
void status_led(){
	rmt_config_t config = RMT_DEFAULT_CONFIG_TX(RGB_GPIO, RMT_TX_CHANNEL);
	// set counter clock to 40MHz
	config.clk_div = 2;

	ESP_ERROR_CHECK(rmt_config(&config));
	ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

	// install ws2812 driver
	led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
	led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
	if (!strip) {
		ESP_LOGE(TAG_FLASH, "install WS2812 driver failed");
	}
	int led_no = 0;
	ESP_ERROR_CHECK(strip->clear(strip, 100));
	while(1){
		EventBits_t ledBits = xEventGroupGetBits(led_event_group);
		if (((ledBits&NETWORK_SET_BIT) == NETWORK_SET_BIT) &&((ledBits&SD_WRITE_SET_BIT) == SD_WRITE_SET_BIT)){
			ESP_ERROR_CHECK(strip->set_pixel(strip, led_no, 0,255,0));
			ESP_ERROR_CHECK(strip->refresh(strip, 100));
		} else if (((ledBits&NETWORK_SET_BIT) == NETWORK_SET_BIT) && ((ledBits&SD_WRITE_SET_BIT) != SD_WRITE_SET_BIT)){
			ESP_ERROR_CHECK(strip->set_pixel(strip, led_no, 255,255,0));
			ESP_ERROR_CHECK(strip->refresh(strip, 100));
		} else if(((ledBits&NETWORK_SET_BIT) != NETWORK_SET_BIT) && ((ledBits&SD_WRITE_SET_BIT) == SD_WRITE_SET_BIT)){
			ESP_ERROR_CHECK(strip->set_pixel(strip, led_no, 0,0,255));
			ESP_ERROR_CHECK(strip->refresh(strip, 100));
		} else {
			ESP_ERROR_CHECK(strip->set_pixel(strip, led_no, 255,0,0));
			ESP_ERROR_CHECK(strip->refresh(strip, 100));
		}
		vTaskDelay(10/portTICK_RATE_MS);
	}
}
/*
 * @brief This function shows status by RGB LED
 */
void fota_status_led(){
	rmt_config_t config = RMT_DEFAULT_CONFIG_TX(RGB_GPIO, RMT_TX_CHANNEL);
	// set counter clock to 40MHz
	config.clk_div = 2;

	ESP_ERROR_CHECK(rmt_config(&config));
	ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

	// install ws2812 driver
	led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(STRIP_LED_NUMBER, (led_strip_dev_t)config.channel);
	led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);
	if (!strip) {
		ESP_LOGE(TAG_FLASH, "install WS2812 driver failed");
	}
	int led_no = 0;
	ESP_ERROR_CHECK(strip->clear(strip, 100));
	while(1){
		EventBits_t ledBits = xEventGroupGetBits(wifi_event_group);
		if (((ledBits&FOTA_BIT) == FOTA_BIT) &&((ledBits&CONNECTED_BIT) == CONNECTED_BIT)&& ((ledBits&FOTA_SUCCESS) == FOTA_SUCCESS)){
			ESP_ERROR_CHECK(strip->set_pixel(strip, led_no, 0,255,0));
			ESP_ERROR_CHECK(strip->refresh(strip, 100));
		} else if (((ledBits&FOTA_BIT) == FOTA_BIT) && ((ledBits&CONNECTED_BIT) != CONNECTED_BIT)){
			ESP_ERROR_CHECK(strip->set_pixel(strip, led_no, 20, 44, 225));
			ESP_ERROR_CHECK(strip->refresh(strip, 100));
		} else if(((ledBits&FOTA_BIT) == FOTA_BIT) && ((ledBits&CONNECTED_BIT) == CONNECTED_BIT) && ((ledBits&FOTA_SUCCESS) != FOTA_SUCCESS)){
			ESP_ERROR_CHECK(strip->set_pixel(strip, led_no, 20, 44, 225));
			ESP_ERROR_CHECK(strip->refresh(strip, 100));
			vTaskDelay(200/portTICK_RATE_MS);
			ESP_ERROR_CHECK(strip->set_pixel(strip, led_no, 0,0,0));
			ESP_ERROR_CHECK(strip->refresh(strip, 100));
			vTaskDelay(200/portTICK_RATE_MS);
		}
		vTaskDelay(10/portTICK_RATE_MS);
	}
}
/*
 * @brief This Function checks Network of SIM7600 and connect to MQTT server
 * & receives sensor data from their respective queues,
 *        publish data to the pre-defined topic
 */
void sim7600(){
	while (1) {
		struct tm UpdatedTime;
		bool pubfail = false;
		xEventGroupClearBits(led_event_group,NETWORK_SET_BIT);
		/*loop to Reset SIM7600 and check network*/
		if (PowerOn()) {
			xEventGroupSetBits(led_event_group,NETWORK_SET_BIT);
			/*loop to Switch ON internet*/
			if (Internet(APN)) {
				/*loop to connect to MQTT broker and publish data*/
				while (!pubfail) {
					ESP_LOGI(TAG_SIM7600, "FREE INTERNAL HEAP MQTT destroy end %d", esp_get_free_internal_heap_size());
					GPSPositioning();
					sensor_data_t received_data_mqtt;
					/*loop to check availability of sensors data from task: Get_Sensors_data*/
					if ( xQueueReceive(sensor_data_queue, &received_data_mqtt, interval) ) {
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
					if ((Counter >= 4320) || Rtc) {
						getTime(&UpdatedTime);
						printf("%d,%d,%d,%d,%d,%d",(UpdatedTime.tm_year),(UpdatedTime.tm_mon),UpdatedTime.tm_mday,UpdatedTime.tm_hour,UpdatedTime.tm_min,UpdatedTime.tm_sec);
						xQueueOverwrite(time_queue, &UpdatedTime);
						Rtc = false;
						Counter = 0;
					}
				}
			}
		}
		else{
			xEventGroupClearBits(led_event_group,NETWORK_SET_BIT);
			GPSPositioning();
		}
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
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	wifi_event_group = xEventGroupCreate();
	pms_switch_event_group = xEventGroupCreate();
	led_event_group = xEventGroupCreate();
	inituart(9600, pms0_uart, TXD0_PIN, RXD0_PIN, PMS_BUF_SIZE);
	sdCard_pins_config_t  pin = {
			.miso_pin = 19,
			.mosi_pin = 23,
			.sclk_pin = 18,
			.cs_pin = 5,
	};
	ret = init_sd(&pin);
	if (ret != ESP_OK){
		ESP_LOGI(TAG_FLASH,"sd card init failed");
		SD_init=false;
		xEventGroupClearBits(led_event_group,SD_WRITE_SET_BIT);
	} else {
		xEventGroupSetBits(led_event_group,SD_WRITE_SET_BIT);
		SD_init=true;
	}
	ESP_LOGI(TAG_SIM7600,"hello aaditya blue");
	read_config(&pms0_time, &pms1_time, &b_pms_time, &fota);
	xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
	xEventGroupClearBits(wifi_event_group, FOTA_BIT);
	xEventGroupClearBits(wifi_event_group, FOTA_SUCCESS);
	if(fota==1){
		xEventGroupSetBits(wifi_event_group, FOTA_BIT);
		xTaskCreate(fota_status_led, "fota_status_led", 1024*3, NULL, 5, NULL);
		initialise_wifi();//
		xTaskCreate(&FOTA,"FOTA",10000,NULL,5,NULL);
	}
	else{
		inituart(9600, pms1_uart, TXD2_PIN, RXD2_PIN, PMS_BUF_SIZE);
		inituart(115200, SIM7600_uart, TXD1_PIN, RXD1_PIN, SIM7600_BUF_SIZE);
		// Initialise GPIO for PMS toggle
		init_gpio();
		//Get MAC Id of ESP32
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
		initI2C(I2C_MODE_MASTER, I2C_NUM_0, SDA_PIN, SCL_PIN, 100000);
		ret = bmp180_init(I2C_NUM_0, &dev);
		if (ret != ESP_OK){
			bmp280_flag = false;
		}

		sensor_data_queue = xQueueCreate(1,sizeof(sensor_data_t));
		time_queue = xQueueCreate(1,sizeof(struct tm));
		SD_data_queue=xQueueCreate(1,sizeof(SD_sensor_t));

		/*  Creates getSensorData with a stack size of 4096 bytes at priority 4 */
		xTaskCreate(getSensorData, "get_sensor_data", 1024*4, NULL, 4, NULL);
		xTaskCreate(sim7600,"SIM7600",8192,NULL,1,NULL);
		xTaskCreate(pms_toggle,"pms_toggle", 1024*2,NULL,5,NULL);
		xTaskCreate(status_led, "Status_led", 1024*3, NULL, 2, NULL);
		xTaskCreate(Data_Logger, "Data_Logger", 1024*8, NULL, 2, NULL);
	}
}
