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


#define BUFFERSIZE 400
#define NUMBUFFERS 10


//******************************** Swap the ports for the two devices!
//Device 1
#define RPORT 3333
#define SPORT 3334
/*
//device 2
#define RPORT 3334
#define SPORT 3333

*/

/*
 * Setup globals
 */
uint16_t rx_buffer[NUMBUFFERS][BUFFERSIZE];
uint16_t rxWriteTo = 0;
uint16_t rxReadFrom = 0;



uint16_t rxWrite = 0;
uint16_t rxRead = 0;


uint16_t tx_buffer[NUMBUFFERS][BUFFERSIZE];
uint16_t txWriteTo= 0;
uint16_t txReadFrom = 0;

uint16_t txWrite = 0;
uint16_t txRead = 0;
uint8_t txReady = 0;

int packetCount = 0;

int sockRec;
int sockSend;
struct sockaddr_in send_addr;

/*
 * Setup the send socket
 */

/*
void setupSampleWave()
{
	uint8_t sample = 0;
	for(int i = 0; i < sizeof(send_buffer)/sizeof(send_buffer[0]); i++)
	{
		send_buffer[i] = sample;
		sample ^= 0x1;
	}

}
*/


void setupSend()
{
	char addr_str[128];
	int addr_family;
	int ip_protocol;
	char TAG[] = "setupSend";
	//setup socket structure for sending

	send_addr.sin_addr.s_addr = INADDR_BROADCAST;//inet_addr(ipaddr_addr(SENDIP)); //set destination ip
	send_addr.sin_family = AF_INET;
	send_addr.sin_port = htons(SPORT);
	addr_family = AF_INET;
	ip_protocol = IPPROTO_IP;
	inet_ntoa_r(send_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

	sockSend = socket(addr_family, SOCK_DGRAM, ip_protocol);
	if (sockSend < 0) {
		ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
	}
	ESP_LOGI(TAG, "Socket created, sending");

}

/*
 * send the data in the global buffer to the setup connection
 */
void udpSend()
{
	char TAG[] = "udpSend";
	while (1)
	{
		if(txReady)
		{
			txReady = 0; //acknowledge
			int err = sendto(sockSend, tx_buffer[txReadFrom], BUFFERSIZE * 2, 0, (struct sockaddr *)&send_addr, sizeof(send_addr));
			if (err < 0) {
				ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
				break;
			}
			//ESP_LOGI(TAG, "Message sent");
			txReadFrom+=1;
			if(txReadFrom == NUMBUFFERS)
				txReadFrom = 0;
		}

	}



}


/*
 * Set up the receive socket
 */
void setupReceive()
{
	char TAG[] = "Setup Receive";
	char addr_str[128];
	int addr_family;
	int ip_protocol;

	//set up socket structure for receiving
	struct sockaddr_in receiveAddr;
	receiveAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	receiveAddr.sin_family = AF_INET;
	receiveAddr.sin_port = htons(RPORT);
	addr_family = AF_INET;
	ip_protocol = IPPROTO_IP;
	inet_ntoa_r(receiveAddr.sin_addr, addr_str, sizeof(addr_str) - 1);




	//create socket
	sockRec = socket(addr_family, SOCK_DGRAM, ip_protocol);
	if(sockRec < 0)
	{
		ESP_LOGE(TAG, "Unable to create sockeet: errno %d", errno);
	}
	ESP_LOGI(TAG, "Socket created");

	//bind socket
	int err = bind(sockRec, (struct sockaddr *)&receiveAddr, sizeof(receiveAddr));
	if(err==0)
	{
		ESP_LOGE(TAG, "Socket bound");
	}
	else
	{
		ESP_LOGE(TAG, "Socket biding failed %d", err);
	}


}



/*
 * wait for incoming data on the setup connection
 */

int rxReady = 0;
void udpReceive()
{
	char TAG[] = "updReceive";
	while(1)
	{
		//ESP_LOGI(TAG, "Waiting for data");
		struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
		socklen_t socklen = sizeof(source_addr);

		//check for socket packets
		int len = recvfrom(sockRec, rx_buffer[rxWriteTo], BUFFERSIZE * 2, 0, (struct sockaddr *)&source_addr, &socklen);
		//ESP_LOGI(TAG, "Got Data");
		if (len < 0)
		{
			//ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
			//gpio_set_level(GPIO_NUM_26, 1);
		}
		//if packet was received
		else {
			rxWriteTo+=1;
			rxReady = 1;
			if(rxWriteTo == NUMBUFFERS)
			{
				rxWriteTo = 0;
			}
			packetCount+=1;

		}
	}


}


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
	/*
	myInput.pin_bit_mask = 1ULL<<GPIO_NUM_23;
	myInput.intr_type = GPIO_PIN_INTR_DISABLE;
	myInput.pull_down_en = 1;
	myInput.pull_up_en = 0;
	myInput.mode = GPIO_MODE_INPUT;
	//gpio_config(&myInput);


	myInput.pin_bit_mask = 1ULL<<GPIO_NUM_26;
	myInput.intr_type = GPIO_PIN_INTR_DISABLE;
	myInput.pull_down_en = 0;
	myInput.pull_up_en = 0;
	myInput.mode = GPIO_MODE_OUTPUT;
	gpio_config(&myInput);   */


	myInput.pin_bit_mask = 1ULL<<GPIO_NUM_25;
	myInput.intr_type = GPIO_PIN_INTR_DISABLE;
	myInput.pull_down_en = 0;
	myInput.pull_up_en = 0;
	myInput.mode = GPIO_MODE_OUTPUT;
	gpio_config(&myInput);
	gpio_set_level(GPIO_NUM_25, 1);


	//Set up wifi
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_ERROR_CHECK(example_connect());
	//End set up wifi
	//setupSampleWave();






	setupReceive();
	xTaskCreate(udpReceive, "udp_receive", 4096, NULL, 5, NULL);
	setupSend();
	xTaskCreate(udpSend, "udp_send", 4096, NULL, 5, NULL);
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 125));
	//dac_output_enable(DAC_CHANNEL_1);


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


uint16_t musicCounter = 0;
int onePass = 0;
uint32_t adc_reading = 0;
static void periodic_timer_callback(void* arg)
{

	if(packetCount >= 1)
	{	DAC_Out(rx_buffer[rxReadFrom][rxRead] >> 2); //shift to get rid of some noise
		rxRead+=1;
	}
	if(rxRead == BUFFERSIZE-1)
	{
		packetCount -=1;
		rxReadFrom+=1;
		if(rxReadFrom == NUMBUFFERS)
		{rxReadFrom = 0;

		}
		rxRead = 0;
	}


	 adc_reading = adc1_get_raw(ADC1_CHANNEL_0);
	 tx_buffer[txWriteTo][txWrite] = adc_reading;
	//ESP_LOGI("ADC", "VAL: %d",adc_reading);

    txWrite+=1;
    if(txWrite == BUFFERSIZE)
    {
    	txWriteTo+=1;
    	if(txWriteTo == NUMBUFFERS)
    	{
    		txWriteTo = 0;
    	}
    	txWrite = 0;
    	txReady = 1;
    }

}




