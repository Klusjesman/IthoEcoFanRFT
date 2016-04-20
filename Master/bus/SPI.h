/*
 * Author: Klusjesman
 */

#ifndef __SPI_H__
#define __SPI_H__

#include <stdio.h>
#include "../component/DigitalPin.h"

/*	SPI ports and pins configuration */
#define SPI_PORT_MOSI PORTB		//PB3 = MOSI
#define SPI_PIN_MOSI 3
#define SPI_DDR_MOSI DDRB

#define SPI_PORT_MISO PORTB		//PB4 = MISO
#define SPI_PIN_MISO 4
#define SPI_DDR_MISO DDRB

#define SPI_PORT_SCK PORTB		//PB5 = SCK
#define SPI_PIN_SCK 5
#define SPI_DDR_SCK DDRB


class SPI
{
	//variables
	public:
	protected:
	private:
		DigitalPin *ssPin;

	//functions
	public:
		SPI(DigitalPin *ssPin);
		~SPI();
		
		void init();
		void waitMiso();
		
		void select();
		void deselect();
		
		uint8_t write(uint8_t value);
		uint8_t read();
		
		void attachInterrupt();
		void detachInterrupt();
		
	protected:
	private:
		SPI();
		SPI( const SPI &c );
		SPI& operator=( const SPI &c );

}; //SPI

#endif //__SPI_H__
