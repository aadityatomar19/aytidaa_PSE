/*
 * DS1307.h
 *
 *  Created on: 11-Mar-2022
 *      Author: root
 */

#ifndef MAIN_DS1307_H_
#define MAIN_DS1307_H_

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include <time.h>

#define DS1307_ADDRESS                           0x68        // I2C address of DS1307

esp_err_t read_time(i2c_port_t i2c_num,time_t *currentTime);
esp_err_t writetime(i2c_port_t i2c_num,  struct tm *newTime);

#endif /* MAIN_DS1307_H_ */
