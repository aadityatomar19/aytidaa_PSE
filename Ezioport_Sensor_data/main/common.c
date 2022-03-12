/*
 * common.c
 *
 *  Created on: Mar 12, 2022
 *      Author: Admin
 */

#include "common.h"
uint32_t IRAM_ATTR millis() {
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
}
