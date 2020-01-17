#include "main.h"

ISR(RTC_CNT_vect)
{
	RTC.INTFLAGS = RTC_OVF_bm;
}

int main(void)
{
	// Set all pins to low power mode:
	for (uint8_t i = 0; i < 8; i++) {
		*((uint8_t *)&PORTA + 0x10 + i) |= 1 << PORT_PULLUPEN_bp;
	}

	for (uint8_t i = 0; i < 8; i++) {
		*((uint8_t *)&PORTB + 0x10 + i) |= 1 << PORT_PULLUPEN_bp;
	}
	
	// Set up PINs
	SPI_init();
	
	// Set up RTC and interruption
	RTC_init();
	sei();
	
	// Set up sleep
	set_sleep_mode(SLEEP_MODE_STANDBY);
	sleep_enable();

	// Counters
	unsigned int frameCounter = 0;
	unsigned int sleepCounter = 0;
	
	while (1)
	{
		if (sleepCounter % (SLEEP_MINUTES + 1) == 0) {
			// BME Initialization
			struct bme280_dev bme280;
			BME280_init(&bme280);

			// Collect Data
			uint32_t hum32 = 0;
			uint32_t pre32 = 0;
			uint32_t tem32 = 0;
			struct bme280_data comp_data;
			for (uint8_t i = 0; i <= 5; i++) {
				BME280_measure(&comp_data, &bme280);
				if (i == 0) {
					continue;
				}
				hum32 += comp_data.humidity;
				pre32 += comp_data.pressure;
				tem32 += comp_data.temperature;
			}
			
			hum32 = hum32 / 5;
			pre32 = pre32 / 5;
			tem32 = tem32 / 5;

			unsigned char hum[4], pre[4], tem[4];
			hum[0] = (hum32 >> 24) & 0xFF;
			hum[1] = (hum32 >> 16) & 0xFF;
			hum[2] = (hum32 >> 8) & 0xFF;
			hum[3] = hum32 & 0xFF;
			pre[0] = (pre32 >> 24) & 0xFF;
			pre[1] = (pre32 >> 16) & 0xFF;
			pre[2] = (pre32 >> 8) & 0xFF;
			pre[3] = pre32 & 0xFF;
			tem[0] = (tem32 >> 24) & 0xFF;
			tem[1] = (tem32 >> 16) & 0xFF;
			tem[2] = (tem32 >> 8) & 0xFF;
			tem[3] = tem32 & 0xFF;

			// LoRa Initialization
			TinyLoRa lora = TinyLoRa(SPI_SS_LORA);
			lora.setChannel(MULTI);
			lora.setDatarate(SF7BW125);
			lora.begin();

			// Send Data
			unsigned char data[12] = {hum[0], hum[1], hum[2], hum[3], pre[0], pre[1], pre[2], pre[3], tem[0], tem[1], tem[2], tem[3]};
			lora.sendData(data, sizeof(data), frameCounter);
			frameCounter++;
			
			sleepCounter = 1;
		} else {
			sleepCounter++;
			sleep_cpu();
		}
	}

	return 0;
}
