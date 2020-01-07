#include "spi.h"

void SPI_init()
{
	PORTA.DIR |= SPI_MOSI; /*Set MOSI pin direction to output */
	PORTA.DIR &= ~SPI_MISO; /*Set MISO pin direction to input */
	PORTA.DIR |= SPI_CLOCK; /*Set SCK pin direction to output */
	PORTA.DIR |= SPI_SS_LORA; /*Set SS pin 1 direction to output */
	PORTA.DIR |= SPI_SS_BME280; /*Set SS pin 2 direction to output */

	PORTA.OUT |= SPI_SS_LORA;
	PORTA.OUT |= SPI_SS_BME280;

	SPI0.CTRLA = SPI_ENABLE_bm /*Enable module */ |
					SPI_MASTER_bm /*SPI module in Master mode */ |
					SPI_CLK2X_bm /*Enable double-speed */ |
					SPI_PRESC_DIV4_gc; /*System Clock divided by 4 */

	SPI0.CTRLB = SPI_BUFEN_bm /*Buffer Mode Enable */ |
					SPI_SSD_bm /*Slave Select Disable */ |
					SPI_MODE_3_gc; /*SPI Mode 3 */
}

uint8_t SPI_transmit(uint8_t data)
{
	while (!(SPI0.INTFLAGS &(SPI_DREIF_bm)));
	SPI0.DATA = data;
	while (!(SPI0.INTFLAGS &(SPI_RXCIF_bm)));
	return SPI0.DATA;
}

void SPI_read(uint16_t ss_port, uint8_t addr, uint8_t *data, uint16_t len)
{
	PORTA.OUT &= ~ss_port;

	for (uint16_t i = 0; i < len; i++)
	{
		if (i == 0)
		{
			SPI_transmit(addr);
		}
		data[i] = SPI_transmit(0x00);
	}

	PORTA.OUT |= ss_port;
}

void SPI_write(uint16_t ss_port, uint8_t addr, uint8_t *data, uint16_t len)
{
	PORTA.OUT &= ~ss_port;

	for (uint16_t i = 0; i < len; i++)
	{
		SPI_transmit(addr + i);
		SPI_transmit(data[i]);
	}

	PORTA.OUT |= ss_port;
}