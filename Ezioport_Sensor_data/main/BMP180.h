/*
 * BMP180.h
 *
 *  Created on: Jun 1, 2022
 *      Author: Admin
 */

/*
 * BMP280.h
 *
 *  Created on: 11-Mar-2022
 *      Author: root
 */

#ifndef MAIN_BMP180_H_
#define MAIN_BMP180_H_

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#define BMP180_I2C_ADDRESS_0    0x77                      // I2C address when SDO pin is low
#define BMP180_CHIP_ID          0x55                      // BMP280 has chip-id 0x58
/***BMP280 registers*/
#define BMP180_VERSION_REG        0xD0
#define BMP180_CONTROL_REG        0xF4
#define BMP180_RESET_REG          0xE0
#define BMP180_OUT_MSB_REG        0xF6
#define BMP180_OUT_LSB_REG        0xF7
#define BMP180_OUT_XLSB_REG       0xF8
#define BMP180_CALIBRATION_REG    0xAA
// Values for BMP180_CONTROL_REG
#define BMP180_MEASURE_TEMP       0x2E
#define BMP180_MEASURE_PRESS      0x34
// Reset value for BMP180_RESET_REG
#define BMP180_RESET_VALUE        0xB6

/**
 * Device descriptor
 */
typedef struct {
	int16_t  AC1;
	int16_t  AC2;
	int16_t  AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t  B1;
	int16_t  B2;
	int16_t  MB;
	int16_t  MC;
	int16_t  MD;
	uint8_t   id;       // Chip ID
} bmp180_t;


esp_err_t bmp180_init(i2c_port_t i2c_num, bmp180_t *dev);
esp_err_t bmp180_read_float(i2c_port_t i2c_num, bmp180_t *dev, float *temperature, float *pressure);
#endif /* MAIN_BMP180_H_ */
