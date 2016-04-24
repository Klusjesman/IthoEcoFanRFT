/*
 * Author: Klusjesman
 */

#include "CC1101.h"
#include "../time/delay.h"
#include "../suart/SerialDebug.h"

// default constructor
CC1101::CC1101(SPI *spi)
{
	this->spi = spi;
} //CC1101

// default destructor
CC1101::~CC1101()
{
} //~CC1101

void CC1101::spi_waitMiso()
{
	while ((SPI_PORT_MISO >> SPI_PIN_MISO) & 0x01);
}

void CC1101::init()
{
	reset();
}

void CC1101::reset()
{
	spi->deselect();
	delay_us(5);
	spi->select();
	delay_us(10);
	spi->deselect();
	delay_us(45);
	spi->select();

	spi_waitMiso();
	writeCommand(CC1101_SRES);
	delay_ms(10);
	spi_waitMiso();
	spi->deselect();
}

uint8_t CC1101::writeCommand(uint8_t command) 
{
	uint8_t result;
	
	spi->select();
	spi_waitMiso();
	result = spi->write(command);
	spi->deselect();
	
	return result;
}

void CC1101::writeRegister(uint8_t address, uint8_t data) 
{
	spi->select();
	spi_waitMiso();
	spi->write(address);
	spi->write(data);
	spi->deselect();
}

uint8_t CC1101::readRegister(uint8_t address)
{
	uint8_t val;

	spi->select();
	spi_waitMiso();
	spi->write(address);
	val = spi->read();
	spi->deselect();
	
	return val;
}

/* Known SPI/26MHz synchronization bug (see CC1101 errata)
This issue affects the following registers: SPI status byte (fields STATE and FIFO_BYTES_AVAILABLE), 
FREQEST or RSSI while the receiver is active, MARCSTATE at any time other than an IDLE radio state, 
RXBYTES when receiving or TXBYTES when transmitting, and WORTIME1/WORTIME0 at any time.*/
uint8_t CC1101::readRegisterWithSyncProblem(uint8_t address, uint8_t registerType)
{
	uint8_t value1, value2;	
	int i;
	
	value1 = readRegister(address | registerType);
	
	//if two consecutive reads gives us the same result then we know we are ok
	do 
	{
		value2 = value1;
		value1 = readRegister(address | registerType);
	} 
	while (value1 != value2);
	
	return value1;
}

//registerType = CC1101_CONFIG_REGISTER or CC1101_STATUS_REGISTER
uint8_t CC1101::readRegister(uint8_t address, uint8_t registerType)
{
	switch (address)
	{
		case CC1101_FREQEST:
		case CC1101_MARCSTATE:
		case CC1101_RXBYTES:
		case CC1101_TXBYTES:
		case CC1101_WORTIME1:
		case CC1101_WORTIME0:	
			return readRegisterWithSyncProblem(address, registerType);	
			
		default:
			return readRegister(address | registerType);
	}
}

void CC1101::writeBurstRegister(uint8_t address, uint8_t* data, uint8_t length)
{
	uint8_t i;

	spi->select();
	spi_waitMiso();
	spi->write(address | CC1101_WRITE_BURST);

	//write all data bytes
	for(i=0 ; i<length ; i++)
		spi->write(data[i]);

	spi->deselect();
}

void CC1101::readBurstRegister(uint8_t* buffer, uint8_t address, uint8_t length)
{
	uint8_t i;
	
	spi->select();
	spi_waitMiso();
	spi->write(address | CC1101_READ_BURST);
	
	//read all data bytes
	for (i=0; i<length; i++)
		buffer[i] = spi->read();
		
	spi->deselect();
}

uint8_t CC1101::receiveData(CC1101Packet* packet, uint8_t length)
{
	uint8_t rxBytes = readRegisterWithSyncProblem(CC1101_RXBYTES, CC1101_STATUS_REGISTER);
	rxBytes = rxBytes & CC1101_BITS_RX_BYTES_IN_FIFO;
	
	//check for rx fifo overflow
	if ((readRegisterWithSyncProblem(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & CC1101_BITS_MARCSTATE) == CC1101_MARCSTATE_RXFIFO_OVERFLOW)
	{
		writeCommand(CC1101_SIDLE);	//idle
		writeCommand(CC1101_SFRX); //flush RX buffer
		writeCommand(CC1101_SRX); //switch to RX state	
	}
	else if (rxBytes == length)
	{
		readBurstRegister(packet->data, CC1101_RXFIFO, rxBytes);

		//continue RX
		writeCommand(CC1101_SIDLE);	//idle		
		writeCommand(CC1101_SFRX); //flush RX buffer
		writeCommand(CC1101_SRX); //switch to RX state	
		
		packet->length = rxBytes;				
	}
	else
	{
		//empty fifo
		packet->length = 0;
	}

	return packet->length;
}

bool CC1101::sendData(CC1101Packet *packet)
{
	/************************************************************************/
	/* this function is not tested!                                                                     */
	/************************************************************************/
	
	writeCommand(CC1101_SIDLE);		//idle

	uint8_t txStatus = readRegisterWithSyncProblem(CC1101_TXBYTES, CC1101_STATUS_REGISTER);
	
	//clear TX fifo if needed
	if (txStatus & CC1101_BITS_TX_FIFO_UNDERFLOW)
	{
		writeCommand(CC1101_SIDLE);	//idle
		writeCommand(CC1101_SFTX);	//flush TX buffer
	}

	writeCommand(CC1101_SIDLE);		//idle

	writeBurstRegister(CC1101_TXBYTES, packet->data, packet->length);

	writeCommand(CC1101_SIDLE);
	writeCommand(CC1101_STX);		//switch to TX state

 	//wait until transmission is finished (TXOFF_MODE is expected to be set to 0/IDLE)
	while ((readRegisterWithSyncProblem(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & CC1101_BITS_MARCSTATE) != CC1101_MARCSTATE_IDLE);
}

void CC1101::printRegisters()
{
	debug.serOut("Status registers\n");
	printRegister(CC1101_PARTNUM, CC1101_STATUS_REGISTER, "CC1101_PARTNUM");
	printRegister(CC1101_VERSION, CC1101_STATUS_REGISTER, "CC1101_VERSION");
	printRegister(CC1101_MARCSTATE, CC1101_STATUS_REGISTER, "CC1101_MARCSTATE");
	printRegister(CC1101_PKTSTATUS, CC1101_STATUS_REGISTER, "CC1101_PKTSTATUS");
	printRegister(CC1101_VCO_VC_DAC, CC1101_STATUS_REGISTER, "CC1101_VCO_VC_DAC");
	printRegister(CC1101_TXBYTES, CC1101_STATUS_REGISTER, "CC1101_TXBYTES");
	debug.serOut("\n");

	debug.serOut("Configuration registers\n");
	printRegister(CC1101_IOCFG2,  CC1101_CONFIG_REGISTER,"CC1101_IOCFG2");
	printRegister(CC1101_IOCFG1,  CC1101_CONFIG_REGISTER,"CC1101_IOCFG1");
	printRegister(CC1101_IOCFG0,  CC1101_CONFIG_REGISTER,"CC1101_IOCFG0");
	printRegister(CC1101_FIFOTHR,  CC1101_CONFIG_REGISTER,"CC1101_FIFOTHR");

	printRegister(CC1101_SYNC1, CC1101_CONFIG_REGISTER,"CC1101_SYNC1");
	printRegister(CC1101_SYNC0, CC1101_CONFIG_REGISTER,"CC1101_SYNC0");

	printRegister(CC1101_CHANNR, CC1101_CONFIG_REGISTER,"CC1101_CHANNR");
	printRegister(CC1101_ADDR,  CC1101_CONFIG_REGISTER,"CC1101_ADDR");

	printRegister(CC1101_FSCTRL1,  CC1101_CONFIG_REGISTER,"CC1101_FSCTRL1");
	printRegister(CC1101_FSCTRL0,  CC1101_CONFIG_REGISTER,"CC1101_FSCTRL0");

	// Set default carrier frequency = 868 MHz
	printRegister(CC1101_FREQ2,  CC1101_CONFIG_REGISTER,"CC1101_FREQ2");
	printRegister(CC1101_FREQ1,  CC1101_CONFIG_REGISTER,"CC1101_FREQ1");
	printRegister(CC1101_FREQ0,  CC1101_CONFIG_REGISTER,"CC1101_FREQ0");

	printRegister(CC1101_MDMCFG4,  CC1101_CONFIG_REGISTER,"CC1101_MDMCFG4");
	printRegister(CC1101_MDMCFG3,  CC1101_CONFIG_REGISTER,"CC1101_MDMCFG3");
	printRegister(CC1101_MDMCFG2,  CC1101_CONFIG_REGISTER,"CC1101_MDMCFG2");
	printRegister(CC1101_MDMCFG1,  CC1101_CONFIG_REGISTER,"CC1101_MDMCFG1");
	printRegister(CC1101_MDMCFG0,  CC1101_CONFIG_REGISTER,"CC1101_MDMCFG0");
	printRegister(CC1101_DEVIATN,  CC1101_CONFIG_REGISTER,"CC1101_DEVIATN");

	printRegister(CC1101_PKTLEN,  CC1101_CONFIG_REGISTER,"CC1101_PKTLEN");
	printRegister(CC1101_PKTCTRL1,  CC1101_CONFIG_REGISTER,"CC1101_PKTCTRL1");
	printRegister(CC1101_PKTCTRL0,  CC1101_CONFIG_REGISTER,"CC1101_PKTCTRL0");

	printRegister(CC1101_MCSM2,  CC1101_CONFIG_REGISTER,"CC1101_MCSM2");
	printRegister(CC1101_MCSM1,  CC1101_CONFIG_REGISTER,"CC1101_MCSM1");
	printRegister(CC1101_MCSM0,  CC1101_CONFIG_REGISTER,"CC1101_MCSM0");
	printRegister(CC1101_FOCCFG,  CC1101_CONFIG_REGISTER,"CC1101_FOCCFG");
	printRegister(CC1101_BSCFG,  CC1101_CONFIG_REGISTER,"CC1101_BSCFG");
	printRegister(CC1101_AGCCTRL2,  CC1101_CONFIG_REGISTER,"CC1101_AGCCTRL2");
	printRegister(CC1101_AGCCTRL1,  CC1101_CONFIG_REGISTER,"CC1101_AGCCTRL1");
	printRegister(CC1101_AGCCTRL0,  CC1101_CONFIG_REGISTER,"CC1101_AGCCTRL0");
	printRegister(CC1101_WOREVT1,  CC1101_CONFIG_REGISTER,"CC1101_WOREVT1");
	printRegister(CC1101_WOREVT0,  CC1101_CONFIG_REGISTER,"CC1101_WOREVT0");
	printRegister(CC1101_WORCTRL,  CC1101_CONFIG_REGISTER,"CC1101_WORCTRL");
	printRegister(CC1101_FREND1,  CC1101_CONFIG_REGISTER,"CC1101_FREND1");
	printRegister(CC1101_FREND0,  CC1101_CONFIG_REGISTER,"CC1101_FREND0");
	printRegister(CC1101_FSCAL3,  CC1101_CONFIG_REGISTER,"CC1101_FSCAL3");
	printRegister(CC1101_FSCAL2,  CC1101_CONFIG_REGISTER,"CC1101_FSCAL2");
	printRegister(CC1101_FSCAL1,  CC1101_CONFIG_REGISTER,"CC1101_FSCAL1");
	printRegister(CC1101_FSCAL0,  CC1101_CONFIG_REGISTER,"CC1101_FSCAL0");
	printRegister(CC1101_RCCTRL1,  CC1101_CONFIG_REGISTER,"CC1101_RCCTRL1");
	printRegister(CC1101_RCCTRL0,  CC1101_CONFIG_REGISTER,"CC1101_RCCTRL0");
	printRegister(CC1101_FSTEST,  CC1101_CONFIG_REGISTER,"CC1101_FSTEST");
	printRegister(CC1101_PTEST,  CC1101_CONFIG_REGISTER,"CC1101_PTEST");
	printRegister(CC1101_AGCTEST,  CC1101_CONFIG_REGISTER,"CC1101_AGCTEST");
	printRegister(CC1101_TEST2,  CC1101_CONFIG_REGISTER,"CC1101_TEST2");
	printRegister(CC1101_TEST1,  CC1101_CONFIG_REGISTER,"CC1101_TEST1");
	printRegister(CC1101_TEST0,  CC1101_CONFIG_REGISTER,"CC1101_TEST0");
}

void CC1101::printRegister(uint8_t address, uint8_t registerType, char *name)
{
	debug.serOut(name);
	debug.serOut(" = ");
	uint8_t val = readRegister(address, registerType);
	debug.serOutIntToHex(val);
	debug.serOut("\n");
}