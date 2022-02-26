#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)

#define TXD2_PIN (GPIO_NUM_26)
#define RXD2_PIN (GPIO_NUM_25)

#define pms0_uart UART_NUM_0
#define pms1_uart UART_NUM_1
static const int RX_BUF_SIZE = 256;
static const int RX_BUF2_SIZE = 1024;
static const char *TAG_PMS = "PMS";
typedef struct {
	uint16_t pm1_0;
	uint16_t pm2_5;
	uint16_t pm10;
} pm_data_t;
void inituart();
esp_err_t read_uart(uart_port_t uart_num , pm_data_t *pm);
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
 * @brief read PMS data from uart buffer and parse its value according to PMS datasheet
 *
 * @param uart_num uart port number
 * @param pm pointer of type pm_data_t to save parsed data
 * @return error_code ESP_OK If parsing is successful
 *                    ESP_FAIL IF any error occur
 */


void app_main(void)
{

	printf("init device\n");

	esp_err_t err;
	inituart(9600, UART_NUM_0, TXD_PIN, RXD_PIN, RX_BUF_SIZE);
	inituart(9600, UART_NUM_1, TXD2_PIN, RXD2_PIN, RX_BUF2_SIZE);
	while(1){
		pm_data_t value0 = {0};
//		pm_data_t value1 = {0};
		pm_data_t pms0Value = {0};
//		pm_data_t pms1Value = {0};
		err = read_uart(pms0_uart, &value0);
		if (err == ESP_OK ){
			//    				iteration_pms0 += 1;
			pms0Value.pm10 += value0.pm10;
			pms0Value.pm1_0 += value0.pm1_0;
			pms0Value.pm2_5 += value0.pm2_5;
		}
//		printf("pms data:");
		ESP_LOGI(TAG_PMS,"pm1.0=%d",pms0Value.pm1_0);
		ESP_LOGI(TAG_PMS,"pm2.5=%d",pms0Value.pm2_5);
		ESP_LOGI(TAG_PMS,"pm10=%d",pms0Value.pm10);

	}
}
esp_err_t read_uart(uart_port_t uart_num , pm_data_t *pm) {
	esp_err_t error_code;
	//	pm_data_t pm ={0};
	uint8_t *data = (uint8_t *) malloc(RX_BUF_SIZE + 1);
	const int rxBytes = uart_read_bytes(uart_num,
			data, 32, 1000 / portTICK_RATE_MS);
	if (rxBytes != 32){
		error_code = ESP_FAIL;
	} else if ( (rxBytes == 32) && (data[0] == 0x42) && (data[1] == 0x4d) ) {
		pm->pm1_0 = ((data[10] << 8) + data[10+1]);
		pm->pm2_5 = ((data[10+2] << 8) + data[10+3]);
		pm->pm10 = ((data[10+4] << 8) + data[10+5]);
		error_code = ESP_OK;
	} else {
		error_code = ESP_FAIL;
	}
	uart_flush(uart_num);
	free(data);
	return error_code;
}
