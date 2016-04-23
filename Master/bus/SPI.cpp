/*
 * Author: Klusjesman
 */

#include "SPI.h"

// default constructor
SPI::SPI(DigitalPin *ssPin)
{
	this->ssPin = ssPin;
} //SPI

// default destructor
SPI::~SPI()
{
} //~SPI

void SPI::init()
{
    /* enable outputs for MOSI, SCK, SS, input for MISO */
    SPI_DDR_MOSI |= (1 << SPI_PIN_MOSI);	//mosi output
    SPI_DDR_SCK |= (1 << SPI_PIN_SCK);		//sck output
	ssPin->makeOutput();					//ss output
    SPI_DDR_MISO &= ~(1 << SPI_PIN_MISO);	//miso input
		
    deselect();

    SPCR =	(0 << SPIE) | /* SPI Interrupt Enable */
			(1 << SPE)  | /* SPI Enable */
			(0 << DORD) | /* Data Order: MSB first */
			(1 << MSTR) | /* Master mode */
			(0 << CPOL) | /* Clock Polarity: SCK low when idle */
			(0 << CPHA) | /* Clock Phase: sample on rising SCK edge */
			(1 << SPR1) | /* Clock Frequency: f_OSC / 128 */
			(1 << SPR0);
    SPSR &= ~(1 << SPI2X); /* No doubled clock frequency */	
	
    deselect();
}

void SPI::waitMiso()
{
	while ((SPI_PORT_MISO >> SPI_PIN_MISO) & 0x01);
}

void SPI::select()
{
	ssPin->write(false);
}

void SPI::deselect()
{
	ssPin->write(true);
}

uint8_t SPI::write(uint8_t value)
{
    SPDR = value;
    /* wait for byte to be shifted out */
    while (!(SPSR & (1 << SPIF)));
    SPSR &= ~(1 << SPIF);
	
	return SPDR;
}

uint8_t SPI::read()
{
    SPDR = 0xFF;
    while (!(SPSR & (1 << SPIF)));
    SPSR &= ~(1 << SPIF);

    return SPDR;	
}

void SPI::attachInterrupt() 
{
	SPCR |= (1<<SPIE);
}

void SPI::detachInterrupt() 
{
	SPCR &= ~(1<<SPIE);
}
