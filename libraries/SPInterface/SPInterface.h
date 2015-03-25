#ifndef SPInterface_h
#define SPInterface_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#define SPI_MOSI				11
#define SPI_MISO				12
#define SPI_SCK					13

class spi_interface {
	public:
		spi_interface();
		void 	init_spi(uint8_t chip_select);
		void 	enable_chip(uint8_t chip_select);
		void 	disable_chip(uint8_t chip_select);
		uint8_t xfer_data_cs(uint8_t chip_select, uint8_t data);
		uint8_t xfer_data(uint8_t data);
		void 	ss_high();
};

#endif