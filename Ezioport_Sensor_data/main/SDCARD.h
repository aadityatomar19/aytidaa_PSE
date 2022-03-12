/*
 * SDCARD.h
 *
 *  Created on: 12-Mar-2022
 *      Author: root
 */

#ifndef MAIN_SDCARD_H_
#define MAIN_SDCARD_H_

#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_err.h"


typedef struct {
	int miso_pin;
	int mosi_pin;
	int sclk_pin;
	int cs_pin;
}sdCard_pins_config_t;


esp_err_t init_sd(sdCard_pins_config_t *pins);
esp_err_t write_sd(char *sd_data,char *filename);


#endif /* MAIN_SDCARD_H_ */
