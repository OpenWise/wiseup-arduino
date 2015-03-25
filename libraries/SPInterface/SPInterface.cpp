#include "SPInterface.h"

spi_interface::spi_interface() {
	ss_high();
}

void spi_interface::init_spi(uint8_t chip_select) {
	pinMode(chip_select, OUTPUT);
	disable_chip(chip_select);
	
	pinMode(SPI_SCK, OUTPUT);
	pinMode(SPI_MOSI, OUTPUT);
	pinMode(SPI_MISO, INPUT);
	
	digitalWrite(SPI_MOSI, LOW);
	digitalWrite(SPI_SCK, LOW);

	SPCR = (1<<SPE)|(1<<MSTR);
	SPCR |= (1<<SPR0);
	/* SPSR |= (1<<SPI2X); /* double speed - not sure nrf can handle it. */
}

void spi_interface::enable_chip(uint8_t chip_select) {
	// cli();
	digitalWrite(chip_select, LOW);
}

void spi_interface::disable_chip(uint8_t chip_select) {
	digitalWrite(chip_select, HIGH);
	// sei();
}

uint8_t spi_interface::xfer_data_cs(uint8_t chip_select, uint8_t data) {
	SPDR = data;
	enable_chip(chip_select);
	while (!(SPSR&(1<<SPIF))) ;
	disable_chip(chip_select);
	return SPDR;
}

uint8_t spi_interface::xfer_data(uint8_t data) {
	SPDR = data;
	while (!(SPSR&(1<<SPIF))) ;
	return SPDR;
}

void spi_interface::ss_high() {
	pinMode(SS, OUTPUT);
	digitalWrite(SS, HIGH);
}