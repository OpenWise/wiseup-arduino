#include "nRFModule.h"

NRF24L01::NRF24L01 (spi_interface* spInstance) {
	spi = spInstance;
}

NRF24L01::~NRF24L01 () {
}

void
NRF24L01::init (uint8_t chipSelect, uint8_t chipEnable) {
	m_csnPin 	= chipSelect;	// default 7
	m_cePin 	= chipEnable;	// default 8
	m_channel 	= 99;
	
	pinMode(m_cePin, OUTPUT);
	pinMode(m_csnPin, OUTPUT);
	
	ceLow();
	csOff ();

	if (bitRead(SPCR, SPE) == 0) {
		spi->init_spi(m_csnPin);
	}
}

void
NRF24L01::configure () {
	/* Set RF channel */
	setRegister (RF_CH, m_channel);

	/* Set length of incoming payload */
	setRegister (RX_PW_P0, m_payload);

	/* Set length of incoming payload for broadcast */
	setRegister (RX_PW_P1, m_payload);
	
	/* Start receiver */
	rxPowerUp ();
	rxFlushBuffer ();
}

void
NRF24L01::send (uint8_t * value) {
	uint8_t status;
	status = getStatus();

	while (m_ptx) {
		status = getStatus();

		if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
			m_ptx = 0;
			break;
		}
	} // Wait until last paket is send

	ceLow();
	txPowerUp (); // Set to transmitter mode , Power up
	txFlushBuffer ();
	
	csOn ();
	spi->xfer_data (W_TX_PAYLOAD); // Write cmd to write payload
	writeBytes (value, NULL, m_payload); // Write payload
	csOff ();
	ceHigh (); // Start transmission

	while (dataSending ()) { }

	delay (10);
	
	// Serial.print ("(NRF)# Channel is ");
	// Serial.println (getRegister(RF_CH));
}

void
NRF24L01::send () {
	send (tx_buffer_ptr);
}

void
NRF24L01::setSourceAddress (uint8_t * addr) {
	ceLow ();
	writeRegister (RX_ADDR_P1, addr, ADDR_LEN);
	ceHigh ();
}

void
NRF24L01::setDestinationAddress (uint8_t * addr) {
	writeRegister (RX_ADDR_P0, addr, ADDR_LEN);
	writeRegister (TX_ADDR, addr, ADDR_LEN);
}

void
NRF24L01::setPayload (uint8_t payload) {
	m_payload = payload;
}

bool
NRF24L01::dataReady () {
	/* See note in getData() function - just checking RX_DR isn't good enough */
	uint8_t status = getStatus();
	/* We can short circuit on RX_DR, but if it's not set, we still need
	 * to check the FIFO for any pending packets */
	if ( status & (1 << RX_DR) ) {
		return 1;
	}
	
	return !rxFifoEmpty();
}

bool
NRF24L01::dataSending () {
	uint8_t status;
	if(m_ptx)	{ // Sending mode.
		status = getStatus();
		/* if sending successful (TX_DS) or max retries exceded (MAX_RT). */
		if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
			rxPowerUp ();
			return false; 
		}
		return true;
	}
	return false;
}

void
NRF24L01::getData (uint8_t * data)  {
	csOn ();
	/* Send cmd to read rx payload */
	spi->xfer_data (R_RX_PAYLOAD);
	/* Read payload */
	writeBytes (data, data, m_payload);
	csOff ();
	/* NVI: per product spec, p 67, note c:
	 * "The RX_DR IRQ is asserted by a new packet arrival event. The procedure
	 * for handling this interrupt should be: 1) read payload through SPI,
	 * 2) clear RX_DR IRQ, 3) read FIFO_STATUS to check if there are more 
	 * payloads available in RX FIFO, 4) if there are more data in RX FIFO,
	 * repeat from step 1)."
	 * So if we're going to clear RX_DR here, we need to check the RX FIFO
	 * in the dataReady() function */
	/* Reset status register */
	setRegister (STATUS, (1<<RX_DR));
}

uint8_t
NRF24L01::getStatus() {
	return getRegister (STATUS);
}

bool
NRF24L01::rxFifoEmpty () {
	uint8_t fifoStatus = getRegister (FIFO_STATUS);
	return (fifoStatus & (1 << RX_EMPTY));
}

void
NRF24L01::rxPowerUp () {
	m_ptx = 0;
	ceLow ();
	setRegister (CONFIG, _CONFIG | ( (1 << PWR_UP) | (1 << PRIM_RX) ));
	ceHigh ();
	setRegister (STATUS, (1 << TX_DS) | (1 << MAX_RT));
}

void
NRF24L01::rxFlushBuffer () {
	sendCommand (FLUSH_RX);
}

void
NRF24L01::txPowerUp () {
	m_ptx = 1;
	setRegister (CONFIG, _CONFIG | ( (1 << PWR_UP) | (0 << PRIM_RX) ));
}

void
NRF24L01::powerDown(){
	ceLow ();
	setRegister (CONFIG, _CONFIG);
}

void
NRF24L01::setChannel (uint8_t channel) {
	m_channel = channel;
	setRegister (RF_CH, channel);
}

void
NRF24L01::setPower (power_t power) {
	uint8_t setupRegisterData = 0;

	switch (power) {
		case NRF_0DBM:
			m_power = 3;
		break;
	    case NRF_6DBM:
	    	m_power = 2;
	    break;
	    case NRF_12DBM:
	    	m_power = 1;
	    break;
	    case NRF_18DBM:
	    	m_power = 0;
	    break;
	}

	setupRegisterData = getRegister (RF_SETUP); // Read current value.
	setupRegisterData &= 0xFC; // Erase the old value;
	setupRegisterData |= (m_power & 0x3);
	setRegister (RF_SETUP, setupRegisterData); // Write the new value.
}

uint8_t
NRF24L01::setSpeedRate (speed_rate_t rate) {
	uint8_t setupRegisterData = 0;

	setupRegisterData = getRegister (RF_SETUP); // Read current value.
	setupRegisterData &= ~((1 << RF_DR_LOW) | (1 << RF_DR_HIGH));

	switch (rate) {
		case NRF_250KBPS:
			setupRegisterData |= (1 << RF_DR_LOW) ;
		break;
		case NRF_1MBPS:
		break;
		case NRF_2MBPS:
			setupRegisterData |= (1 << RF_DR_HIGH);
		break;
	}

	setRegister (RF_SETUP, setupRegisterData); // Write the new value.

	if (setupRegisterData == getRegister (RF_SETUP)) {
		return 0x0;
	}

	return 0x1;
}

void
NRF24L01::ceHigh () {
	digitalWrite (m_cePin, HIGH);
}

void
NRF24L01::ceLow (){
	digitalWrite (m_cePin, LOW);
}

void
NRF24L01::csOn () {
	spi->enable_chip(m_csnPin);
}

void
NRF24L01::csOff () {
	spi->disable_chip(m_csnPin);
}

void
NRF24L01::pollListener() {
	if (dataReady()) {
		getData(rx_buffer_ptr);
		dataRecievedHandler (); /* let know that data arrived */
	}
}

void
NRF24L01::txFlushBuffer () {
	sendCommand (FLUSH_TX);
}

/*
 * ---------------
 * PRIVATE SECTION
 * ---------------
 */

/*
 * Write bytes to the SPI device.
 */
void
NRF24L01::writeBytes (uint8_t * dataout, uint8_t * datain, uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		if (datain != NULL) {
			datain[i] = spi->xfer_data (dataout[i]);
		} else {
			spi->xfer_data (dataout[i]);
		}
	}
}

void
NRF24L01::setRegister (uint8_t reg, uint8_t value) {
	spi->enable_chip (m_csnPin);
	spi->xfer_data (W_REGISTER | (REGISTER_MASK & reg));
	spi->xfer_data (value);
	spi->disable_chip (m_csnPin);
}

uint8_t
NRF24L01::getRegister (uint8_t reg) {
	uint8_t data = 0;
	spi->enable_chip (m_csnPin);
	spi->xfer_data (R_REGISTER | (REGISTER_MASK & reg));
	data = spi->xfer_data (data);
	spi->disable_chip (m_csnPin);

	return data;
}

void
NRF24L01::readRegister (uint8_t reg, uint8_t * value, uint8_t len) {
	spi->enable_chip (m_csnPin);
	spi->xfer_data (R_REGISTER | (REGISTER_MASK & reg));
	writeBytes (value, value, len);
	spi->disable_chip (m_csnPin);
}

void
NRF24L01::writeRegister (uint8_t reg, uint8_t * value, uint8_t len) {
	spi->enable_chip (m_csnPin);
	spi->xfer_data (W_REGISTER | (REGISTER_MASK & reg));
	writeBytes (value, NULL, len);
	spi->disable_chip (m_csnPin);
}

void
NRF24L01::sendCommand (uint8_t cmd) {
	spi->enable_chip (m_csnPin);
	spi->xfer_data (cmd);
	spi->disable_chip (m_csnPin);
}

void
NRF24L01::printDetails() 
{
	Serial.println("NRF print details\n");
	//RX_ADDR_P0;	// need to change len to 5 for address
	//STATUS;
	//RF_CH;
	printByte("STATUS",STATUS,1,'B');
	printByte("RF_CH",RF_CH,1,'D');
	//printByte("RX_PW_P0-6",RX_PW_P0-6);
	printByte("EN_AA",EN_AA,1,'B');
	printByte("EN_RXADDR",EN_RXADDR,1,'B');
	printByte("RF_SETUP",RF_SETUP,1,'B');
	printByte("CONFIG",CONFIG,1,'B');
	//printByte("DYNPD/FEATURE",DYNPD);
	printAddr("RX_ADDR_P0",RX_ADDR_P0,5);
	
	
	Serial.println("End of print details\n");

}

void
NRF24L01::printByte(char s[],uint8_t reg, uint8_t len, char type) 
{
	uint8_t value;
	spi->enable_chip (m_csnPin);
	(reg<16)?(Serial.print("Reg=0x0")):(Serial.print("Reg=0x"));
	Serial.print(reg,HEX);
	Serial.print("\t\t");
	Serial.print(s);
	Serial.print("=");
	while (len--)
	{
		spi->enable_chip (m_csnPin);
		spi->xfer_data (R_REGISTER | (REGISTER_MASK & reg++));
		writeBytes (&value, &value, 1);
		
		switch( type ) 
			{
				case 'H':
					(value<16)?(Serial.print(" 0x0")):(Serial.print(" 0x"));
					Serial.print(value,HEX);
					break;
				case 'D':
					Serial.print(value,DEC);
					break;
				case 'B':
					if (value==0) {
						Serial.print("0000000");
					}
					else 
					{
						for (int i=1; i<(8-log(value)/log(2)); i++) {
							Serial.print("0");
						}
					}
					Serial.print(value,BIN);
					Serial.print("b");
					break;
				default :
					Serial.print(value);
			}
		spi->disable_chip (m_csnPin);
	}
	Serial.println();

}

void
NRF24L01::printAddr(char s[],uint8_t reg, uint8_t len) 
{
	uint8_t value[len];
	spi->enable_chip (m_csnPin);
	(reg<16)?(Serial.print("Reg=0x0")):(Serial.print("Reg=0x"));
	//Serial.print("Reg=0x");
	Serial.print(reg,HEX);
	Serial.print("\t");
	Serial.print(s);
	Serial.print("=0x");

	spi->enable_chip (m_csnPin);
	spi->xfer_data (R_REGISTER | (REGISTER_MASK & reg++));
	writeBytes (&value[0], &value[0], len);
	//(value<16)?(Serial.print(" 0")):(Serial.print(" "));
	for (int i=0; i<len; i++) {
		Serial.print(value[i],HEX);
	}
	spi->disable_chip (m_csnPin);
	Serial.println();

}

uint8_t
NRF24L01::bv (uint8_t shift) {
	return 1 << shift;
}