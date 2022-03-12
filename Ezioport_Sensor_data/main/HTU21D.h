/*
 * HTU21D.h
 *
 *  Created on: 11-Mar-2022
 *      Author: root
 */

#ifndef MAIN_HTU21D_H_
#define MAIN_HTU21D_H_

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#define HTU21D_ADDR                              0x40        // I2C adddress of HTU21D
#define TRIG_TEMP_MEASUREMENT_HOLD               0xE3        //HTU21D register address to read temp in hold mode
#define TRIG_HUMD_MEASUREMENT_HOLD               0xE5        //HTU21D register address to read humidity in hold mode
#define TRIG_TEMP_MEASUREMENT_NOHOLD             0xF3        //HTU21D register address to read temp in no-hold mode
#define TRIG_HUMD_MEASUREMENT_NOHOLD             0xF5        //HTU21D register address to read humidity in no-hold mode
#define SOFT_RESET                               0xFE        //HTU21D register address for soft reset

esp_err_t ht21d_read_humidity(i2c_port_t i2c_num,float *humidity);
esp_err_t ht21d_read_temperature(i2c_port_t i2c_num, float *temperature);



#endif /* MAIN_HTU21D_H_ */
