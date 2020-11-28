#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_wifi.h"

#include <sys/param.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "driver/dac.h"
#include "Sound.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

static void periodic_timer_callback(void* arg);
#define PIN_NUM_MOSI    GPIO_NUM_26
#define PIN_NUM_CLK     GPIO_NUM_27
#define PIN_NUM_CS      GPIO_NUM_32
static spi_device_handle_t dac;
#define DAC_HOST        HSPI_HOST



void DAC_Init()
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num=-1,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t daccfg = {
        .clock_speed_hz = 8000000,  // 1 MHz
        .mode = 1,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .cs_ena_pretrans = 1
    };

    ret = spi_bus_initialize(DAC_HOST, &buscfg, 0);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(DAC_HOST, &daccfg, &dac);
    ESP_ERROR_CHECK(ret);
}

void app_main(void)
{

	//init dac, timer and adc
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
	DAC_Init();
	 const esp_timer_create_args_t periodic_timer_args = {
	            .callback = &periodic_timer_callback,

	            .name = "periodic"
	    };

	esp_timer_handle_t periodic_timer;
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

	gpio_config_t myInput;

	//mic enable
	myInput.pin_bit_mask = 1ULL<<GPIO_NUM_25;
	myInput.intr_type = GPIO_PIN_INTR_DISABLE;
	myInput.pull_down_en = 0;
	myInput.pull_up_en = 0;
	myInput.mode = GPIO_MODE_OUTPUT;
	gpio_config(&myInput);
	gpio_set_level(GPIO_NUM_25, 1);


	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 125));


	//output received data

}


void DAC_Out(uint16_t val)
{
    // Always sending 16 bits of information
    static spi_transaction_t trans = {
        .flags   = SPI_TRANS_USE_TXDATA,
        .length  = 16,
        .tx_data = {0}
    };

    // Update the data being sent
    trans.tx_data[0] = (val >> 8) & 0x0FF;
    trans.tx_data[1] = val & 0x0FF;

    // Send the transaction
    // Note that this blocks. This could be changed
    spi_device_transmit(dac, &trans);
}


uint32_t adc_reading = 0;
static void periodic_timer_callback(void* arg)
{

	DAC_Out(adc_reading>>4);
	adc_reading = adc1_get_raw(ADC1_CHANNEL_0);

}




