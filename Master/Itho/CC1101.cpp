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

//wait for fixed length in rx fifo
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

//This function is able to send packets bigger then the FIFO size.
bool CC1101::sendData(CC1101Packet *packet)
{
	uint8_t txStatus;
	uint8_t length, index;
	
	writeCommand(CC1101_SIDLE);		//idle

	txStatus = readRegisterWithSyncProblem(CC1101_TXBYTES, CC1101_STATUS_REGISTER);
		
	//clear TX fifo if needed
	if (txStatus & CC1101_BITS_TX_FIFO_UNDERFLOW)
	{
		writeCommand(CC1101_SIDLE);	//idle
		writeCommand(CC1101_SFTX);	//flush TX buffer
	}	
	
	//loop while there are more bytes to transmit
	while (index < packet->length)
	{
		writeCommand(CC1101_SIDLE);		//idle

		//check how many bytes are left
		if (packet->length - index <= CC1101_DATA_LEN)
		{
			length = packet->length - index;
		}
		else
		{
			length = CC1101_DATA_LEN;
		}

		writeBurstRegister(CC1101_TXFIFO, &packet->data[index], length);

		index += length;

		writeCommand(CC1101_SIDLE);
		writeCommand(CC1101_STX);		//switch to TX state
	}
	
	//wait until transmission is finished (TXOFF_MODE is expected to be set to 0/IDLE)
	while ((readRegisterWithSyncProblem(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & CC1101_BITS_MARCSTATE) != CC1101_MARCSTATE_IDLE);	
}

