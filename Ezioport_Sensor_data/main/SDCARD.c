/*
 * SDCARD.c
 *
 *  Created on: 12-Mar-2022
 *      Author: root
 */
#include "SDCARD.h"

#define MOUNT_POINT "/sdcard"

static const char *TAG_SD ="SD_Card";

/*
 * @brief Initialise sd card and mount filesystem on it
 */
esp_err_t init_sd(sdCard_pins_config_t *pins) {
	esp_err_t ret;
	esp_vfs_fat_sdmmc_mount_config_t mount_config = {
			.format_if_mount_failed = false,
			.max_files = 5,
			.allocation_unit_size = 16 * 1024
	};
	sdmmc_card_t* card;
	const char mount_point[] = MOUNT_POINT;
	ESP_LOGI(TAG_SD, "Initializing SD card");
	ESP_LOGI(TAG_SD, "Using SPI peripheral");

	sdmmc_host_t host = SDSPI_HOST_DEFAULT();
	spi_bus_config_t bus_cfg = {
			.mosi_io_num = pins->mosi_pin,
			.miso_io_num = pins->miso_pin,
			.sclk_io_num = pins->sclk_pin,
			.quadwp_io_num = -1,
			.quadhd_io_num = -1,
			.max_transfer_sz = 4000,
	};
	spi_dma_chan_t dma_chan = host.slot;
	ret = spi_bus_initialize(host.slot, &bus_cfg, dma_chan);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG_SD, "Failed to initialize bus.");
		return ret;
	}
	sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
	slot_config.gpio_cs = pins->cs_pin;
	slot_config.host_id = host.slot;
	ret = esp_vfs_fat_sdspi_mount(mount_point,
			&host, &slot_config, &mount_config, &card);

	if ( ret != ESP_OK ) {
		if ( ret == ESP_FAIL ) {
			ESP_LOGE(TAG_SD, "Failed to mount filesystem. ");
		} else {
			ESP_LOGE(TAG_SD, "Failed to initialize the card (%s). ",
					esp_err_to_name(ret));
		}
		return ret;
	}
	sdmmc_card_print_info(stdout, card);
	return ret;
}

/*
 * @brief This function writes data to sd card
 * @param sd_data String to write in sd card
 * @param filename file in which data is to be written
 */
esp_err_t write_sd(char *sd_data,char *filename) {
	ESP_LOGI(TAG_SD, "Opening file");
	char *filePath = NULL;
	asprintf(&filePath,"%s/%s.csv",MOUNT_POINT,filename);
	ESP_LOGI(TAG_SD, "filename %s",filePath);
	FILE* f = fopen(filePath,"a");
	if (f == NULL) {
		perror("Error: ");
		ESP_LOGE(TAG_SD, "Failed to open file for writing");
		return ESP_FAIL;
	}
	ESP_LOGI(TAG_SD, "data:- %s ",sd_data);
	int bytesWritten = fprintf(f, "%s \n", sd_data);
	fclose(f);
	free(filePath);
	if (bytesWritten >0)
		return ESP_OK;
	else
		return ESP_FAIL;
}

