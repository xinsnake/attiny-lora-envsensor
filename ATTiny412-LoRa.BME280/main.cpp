#include "main.h"

/************************************************************************/
/* UTILS                                                                */
/************************************************************************/

void delay_ms(uint32_t ms)
{
	while (ms--) {
		_delay_ms(1);
	}
}

/************************************************************************/
/* SPI                                                                  */
/* https://github.com/MicrochipTech/TB3215_Getting_Started_with_SPI/blob/master/ATmega4809_SPI_Examples/Sending_Data_as_Master/main.c */
/************************************************************************/
void SPI0_init() {
	PORTA.DIRSET = SPI_MOSI | SPI_CLOCK | SPI_SS_LORA | SPI_SS_BME280;
	PORTA.DIRCLR = SPI_MISO;
	SPI0.CTRLA = SPI_CLK2X_bm | SPI_DORD_bm | SPI_ENABLE_bm | SPI_MASTER_bm | SPI_PRESC_DIV4_gc;
}

void SPI0_select_lora() {
	PORTA.OUTCLR |= SPI_SS_LORA;
}

void SPI0_release_lora() {
	PORTA.OUT |= SPI_SS_LORA;
}

void SPI0_select_bme280() {
	PORTA.OUTCLR |= SPI_SS_BME280;
}

void SPI0_release_bme280() {
	PORTA.OUT |= SPI_SS_BME280;
}

/************************************************************************/
/* BME280                                                               */
/* https://github.com/BoschSensortec/BME280_driver                      */
/* https://github.com/mongoose-os-libs/bme280/blob/master/src/mgos_bme280.c */
/************************************************************************/

int8_t BME280_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = BME280_OK;

	if (SPI_ID_BME280 == dev_id) {
		SPI0_select_bme280();
		
		/*
		 * Data on the bus should be like
		 * |----------------+---------------------+-------------|
		 * | MOSI           | MISO                | Chip Select |
		 * |----------------+---------------------|-------------|
		 * | (don't care)   | (don't care)        | HIGH        |
		 * | (reg_addr)     | (don't care)        | LOW         |
		 * | (don't care)   | (reg_data[0])       | LOW         |
		 * | (....)         | (....)              | LOW         |
		 * | (don't care)   | (reg_data[len - 1]) | LOW         |
		 * | (don't care)   | (don't care)        | HIGH        |
		 * |----------------+---------------------|-------------|
		 */
		// TODO
		
		SPI0_release_bme280();
		return rslt;
    }

    return ERR_SPI_INVALID_DEVICE;
}

int8_t BME280_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    int8_t rslt = BME280_OK;

	if (SPI_ID_BME280 == dev_id) {
		SPI0_select_bme280();
		
		/*
		 * Data on the bus should be like
		 * |---------------------+--------------+-------------|
		 * | MOSI                | MISO         | Chip Select |
		 * |---------------------+--------------|-------------|
		 * | (don't care)        | (don't care) | HIGH        |
		 * | (reg_addr)          | (don't care) | LOW         |
		 * | (reg_data[0])       | (don't care) | LOW         |
		 * | (....)              | (....)       | LOW         |
		 * | (reg_data[len - 1]) | (don't care) | LOW         |
		 * | (don't care)        | (don't care) | HIGH        |
		 * |---------------------+--------------|-------------|
		 */
		// TODO
		
		SPI0_release_bme280();
		return rslt;
    }

    return ERR_SPI_INVALID_DEVICE;
}

int8_t BME280_init(bme280_dev *dev) {
	int8_t rslt = BME280_OK;

	dev->dev_id = SPI_ID_BME280;
	dev->intf = BME280_SPI_INTF;
	dev->read = BME280_read;
	dev->write = BME280_write;
	dev->delay_ms = delay_ms;

	rslt = bme280_init(dev);
	if (BME280_OK != rslt) {
		return rslt;
	}
	
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;
    dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

    uint8_t settings_sel = BME280_OSR_PRESS_SEL;
    settings_sel |= BME280_OSR_TEMP_SEL;
    if (BME280_CHIP_ID == dev->chip_id) {
	    settings_sel |= BME280_OSR_HUM_SEL;
    }
    settings_sel |= BME280_STANDBY_SEL;
    settings_sel |= BME280_FILTER_SEL;
    rslt = bme280_set_sensor_settings(settings_sel, dev);
    if (BME280_OK != rslt) {
	    return rslt;
    }

    rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);
    if (BME280_OK != rslt) {
	    return rslt;
    }
	
	return BME280_OK;
}

/************************************************************************/
/* MAIN                                                                 */
/************************************************************************/
int main(void)
{
	int8_t rslt = OK;
	
	SPI0_init();
	
	struct bme280_dev env_sensor;
	rslt = BME280_init(&env_sensor);
	if (rslt != BME280_OK) {
		// TODO
	}
	
	while (1) {
		struct bme280_data comp_data;
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &env_sensor);
		if (rslt != BME280_OK) {
			// TODO
		}
		
		delay_ms(1 * ONE_SECOND);
	}

	return ERR_STOPPED;
}

