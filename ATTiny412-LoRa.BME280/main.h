#include <avr/io.h>
#include <util/delay.h>

#define S8_C(x)  x
#define U8_C(x)  x ## U
#define S16_C(x) x
#define U16_C(x) x ## U
#define S32_C(x) x
#define U32_C(x) x ## U
#define S64_C(x) x ## L
#define U64_C(x) x ## UL

#include "BME280_driver-bme280_v3.3.7/bme280.h"

#ifndef MAIN_H_
#define MAIN_H_

#define ONE_SECOND 1000

#define SPI_MOSI PIN1_bm
#define SPI_MISO PIN2_bm
#define SPI_CLOCK PIN3_bm

#define SPI_ID_BME280 0
#define SPI_SS_BME280 PIN7_bm

#define SPI_ID_LORA 1
#define SPI_SS_LORA PIN6_bm

#define OK 0
#define ERR_STOPPED 200
#define ERR_SPI_INVALID_DEVICE 201

#endif /* MAIN_H_ */