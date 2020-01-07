#ifndef BME_H_
#define BME_H_

#define S8_C(x)  x
#define U8_C(x)  x ## U
#define S16_C(x) x
#define U16_C(x) x ## U
#define S32_C(x) x
#define U32_C(x) x ## U
#define S64_C(x) x ## L
#define U64_C(x) x ## UL

#include <avr/io.h>
#include "spi.h"
#include "delay.h"
#include "BME280_driver-bme280_v3.3.7/bme280.h"

int8_t BME280_init(bme280_dev* dev);
int8_t BME280_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t BME280_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t BME280_measure(bme280_data *comp_data, bme280_dev *dev);

#endif /* MY_BME_H_ */