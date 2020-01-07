#include "main.h"

int main(void)
{
	SPI_init();
	
	struct bme280_dev bme280;
	BME280_init(&bme280);
	
	while (1)
	{
		delay_ms(50);

		struct bme280_data comp_data;
		BME280_measure(&comp_data, &bme280);

		delay_ms(50);
	}
	
	return -1;
}