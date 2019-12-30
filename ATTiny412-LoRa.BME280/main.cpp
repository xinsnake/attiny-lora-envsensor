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
/* https://github.com/MicrochipTech/TB3215_Getting_Started_with_SPI/    */
/************************************************************************/
void SPI0_init() {
    PORTA.DIR |= SPI_MOSI;              /* Set MOSI pin direction to output */
    PORTA.DIR &= ~SPI_MISO;             /* Set MISO pin direction to input */
    PORTA.DIR |= SPI_CLOCK;             /* Set SCK pin direction to output */
    PORTA.DIR |= SPI_SS_LORA;           /* Set SS pin 1 direction to output */
    PORTA.DIR |= SPI_SS_BME280;         /* Set SS pin 2 direction to output */
	        
    PORTA.OUT |= SPI_SS_LORA;      
    PORTA.OUT |= SPI_SS_BME280;        
	
    SPI0.CTRLA = SPI_ENABLE_bm          /* Enable module */
			   | SPI_MASTER_bm          /* SPI module in Master mode */
			   | SPI_CLK2X_bm           /* Enable double-speed */
			   // | SPI_DORD_bm         /* LSB is transmitted first */
			   | SPI_PRESC_DIV4_gc;    /* System Clock divided by 16 */
			   
	SPI0.CTRLB = SPI_BUFEN_bm           /* Buffer Mode Enable */
			   | SPI_SSD_bm             /* Slave Select Disable */
			   | SPI_MODE_0_gc;         /* SPI Mode 0 */
}

/************************************************************************/
/* BME280                                                               */
/* https://github.com/BoschSensortec/BME280_driver                      */
/* https://github.com/mongoose-os-libs/bme280/blob/master/src/mgos_bme280.c */
/************************************************************************/

int8_t BME280_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    int8_t rslt = BME280_OK;
	PORTA.OUT &= ~SPI_SS_BME280;
	
	for (uint16_t i = 0; i <= len; i++) {
		SPI0.DATA = reg_addr;
		while (!(SPI0.INTFLAGS & SPI_IF_bm))  /* waits until data is exchanged*/
		{
			;
		}
		if (i != 0) {
			reg_data[i - 1] = SPI0.DATA;
		}
	}
		
	PORTA.OUT |= SPI_SS_BME280;
	return rslt;
}

int8_t BME280_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    int8_t rslt = BME280_OK;
    PORTA.OUT &= ~SPI_SS_BME280;

	for (uint16_t i = 0; i < len; i++) {
		SPI0.DATA = reg_data[i];
		while (!(SPI0.INTFLAGS & SPI_IF_bm))  /* waits until data is exchanged*/
		{
			;
		}
	}
		
	PORTA.OUT |= SPI_SS_BME280;
	return rslt;
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
		return ERR_SPI_BME280_INIT_FAILED;
	}
	
	while (1) {
		struct bme280_data comp_data;
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &env_sensor);
		if (rslt != BME280_OK) {
			return ERR_SPI_BME280_READ_ERROR;
		}
		
		delay_ms(1 * ONE_SECOND);
	}

	return ERR_STOPPED;
}

