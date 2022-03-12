/*
 * pms7003.c
 *
 *  Created on: Mar 11, 2022
 *      Author: Admin
 */
#include"pms7003.h"
//#include "driver/uart.h"
//#include "esp_err.h"


/*
 * @brief read PMS data from uart buffer and parse its value according to PMS datasheet
 *
 * @param uart_num uart port number
 * @param pm pointer of type pm_data_t to save parsed data
 * @return error_code ESP_OK If parsing is successful
 *                    ESP_FAIL IF any error occur
 */
esp_err_t read_pms(uart_port_t uart_num , pm_data_t *pm) {
	esp_err_t error_code;
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



