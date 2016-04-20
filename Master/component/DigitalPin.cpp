/*
 * Author: Klusjesman
 */

#include "DigitalPin.h"

void swap(DigitalPin& dst, const DigitalPin& src)
{
	dst.port = src.port;
	dst.pin = src.pin;
	dst.iomode = src.iomode;	
}

DigitalPin::DigitalPin()
{
}

DigitalPin& DigitalPin::operator=(const DigitalPin &c)
{
	swap(*this,c);
	return *this;
}

DigitalPin::DigitalPin(const DigitalPin &c)
{
	swap(*this,c);
}

DigitalPin::DigitalPin(volatile uint8_t * port, uint8_t pin)
{
	this->port = port;
	this->pin = pin;
	this->iomode = None;
}

DigitalPin::DigitalPin(volatile uint8_t * port, uint8_t pin, IOMode iomode)
{
	this->port = port;
	this->pin = pin;
	
	//init port
	switch (iomode)
	{
		case Input:
			makeInput();
			break;
		
		case Output:
			makeOutput();
			break;
		
		case None:
		break;
	}
}

DigitalPin::~DigitalPin()
{
}

void DigitalPin::makeOutput(void)
{
	//set DDRx.pin to be output
	*(port-1) |= (1 << pin);
	iomode = Output;
}

void DigitalPin::makeInput(void)
{
	//set DDRx.pin to be input
	*(port-1) &= ~(1 << pin);
	iomode = Input;
}


void DigitalPin::write(bool val)
{
	if (val)
	{
		//DigitalPin on
		*port |= (1<< pin);
	}
	else
	{
		//DigitalPin off
		*port &= ~(1 << pin);
	}
}

bool DigitalPin::read()
{
	return (*(port-2) & (1 << pin));
}
