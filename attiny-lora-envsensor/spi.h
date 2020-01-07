#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>
#include "delay.h"

#define SPI_MOSI PIN1_bm
#define SPI_MISO PIN2_bm
#define SPI_CLOCK PIN3_bm

#define SPI_ID_BME280 0
#define SPI_SS_BME280 PIN7_bm

#define SPI_ID_LORA 1
#define SPI_SS_LORA PIN6_bm

void SPI_init();
uint8_t SPI_transmit(uint8_t data);
void SPI_read(uint16_t ss_port, uint8_t addr, uint8_t *data, uint16_t len);
void SPI_write(uint16_t ss_port, uint8_t addr, uint8_t *data, uint16_t len);

#endif /* SPI_H_ */