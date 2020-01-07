#include "bme.h"

int8_t BME280_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	SPI_read(SPI_SS_BME280, reg_addr, reg_data, len);
	return BME280_OK;
}

int8_t BME280_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	SPI_write(SPI_SS_BME280, reg_addr, reg_data, len);
	return BME280_OK;
}

int8_t BME280_init(bme280_dev* dev)
{
	int8_t rslt = BME280_OK;
	
	dev->dev_id = 0;
	dev->intf = BME280_SPI_INTF;
	dev->read = BME280_read;
	dev->write = BME280_write;
	dev->delay_ms = delay_ms;

	rslt = bme280_init(dev);
	if (rslt != 0) {
		return rslt;
	}
	
	uint8_t settings_sel;
	
    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    return bme280_set_sensor_settings(settings_sel, dev);
}

int8_t BME280_measure(bme280_data *comp_data, bme280_dev *dev) {
	int8_t rslt;
		
	rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
	if (rslt != 0) {
		return rslt;
	}
	
	delay_ms(40);
	
	return bme280_get_sensor_data(BME280_ALL, comp_data, dev);
}