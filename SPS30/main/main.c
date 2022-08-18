#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "string.h"
#include "stdio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "SPS30.h"
#include "main.h"

static const int PMS_BUF_SIZE = 1024;
#define TXD0_PIN (GPIO_NUM_1)
#define RXD0_PIN (GPIO_NUM_3)
#define UART0 UART_NUM_0
#define TXD1_PIN (GPIO_NUM_26)
#define RXD1_PIN (GPIO_NUM_25)
#define SPS30_UART1 UART_NUM_1
#define UART0 UART_NUM_0
static const char *TAG_SPS30 = "SPS30";


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
void app_main(void)
{
	esp_err_t ret;
		// Initialise flash in esp32
		ret = nvs_flash_init();
		if ( ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND ) {
			ESP_ERROR_CHECK(nvs_flash_erase());
			ret = nvs_flash_init();
		}
		ESP_ERROR_CHECK(ret);
	  EnableDebugging(DEBUG);

	  inituart(115200, UART0, TXD0_PIN, RXD0_PIN, PMS_BUF_SIZE);
	  inituart(115200, SPS30_UART1, TXD1_PIN, RXD1_PIN, PMS_BUF_SIZE);
	  ESP_LOGI(TAG_SPS30,"Trying to connect");

	  // check for SPS30 connection
	  if (! probe()) Errorloop((char *) "could not probe / connect with SPS30.", 0);
	  else ESP_LOGI(TAG_SPS30,"Detected SPS30.");

	  // reset SPS30 connection
	  if (! reset()) Errorloop((char *) "could not reset.", 0);

	  // start measurement
	  if (start()) ESP_LOGI(TAG_SPS30,"Measurement started");
	  else Errorloop((char *) "Could NOT start measurement", 0);
	  SetAutoCleanInt(20);
	  clean();

	  while(1){
		    read_all();
		    vTaskDelay(1000/portTICK_RATE_MS);
	  }
}
