/*
 * pms7003.h
 *
 *  Created on: Mar 11, 2022
 *      Author: Admin
 */
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"

static const int RX_BUF_SIZE = 256;
static const int RX_BUF2_SIZE = 1024;
#ifndef MAIN_PMS7003_H_
#define MAIN_PMS7003_H_

typedef struct {
	uint16_t pm1_0;
	uint16_t pm2_5;
	uint16_t pm10;
} pm_data_t;


esp_err_t read_pms(uart_port_t uart_num , pm_data_t *pm);




#endif /* MAIN_PMS7003_H_ */
