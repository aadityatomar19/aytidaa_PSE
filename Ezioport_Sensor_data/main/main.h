/*
 * pms.h
 *
 *  Created on: Nov 30, 2020
 *      Author: Sarita Ahlawat
 */

#ifndef MAIN_MAIN_H_
#define MAIN_MAIN_H_
#include "DS1307.h"
#include "HTU21D.h"
#include "BMP180.h"
#include "SDCARD.h"
#include"pms7003.h"
#include "SIM7600.h"


typedef struct {
	pm_data_t pms_num[2];
	float temperature;
	float humidity;
	float pressure;
	float latitude;
	float longitude;
}sensor_data_t;

typedef struct {
	pm_data_t pms[2];
	float temperature;
	float humidity;
	float pressure;
	float latitude;
	float longitude;
	int Year;
	int Month;
	int Day;
	int Hour;
	int Minute;
	int Second;
	char *MAC;
}SD_sensor_t;

typedef struct {
	pm_data_t pms_num[2];
	float temp;
	float hum;
	float press;
	time_t unix_time;
}data_t;

typedef struct {
	float temp_data;
	bool  temp_flag;
}temp_data_t;

typedef struct {
	float hum_data;
	bool  hum_flag;
}hum_data_t;

typedef struct {
    wifi_config_t wifi_values;
    esp_err_t error;
} wifi_data_t;

void initI2C();
void inituart();
#endif /* MAIN_MAIN_H_ */
