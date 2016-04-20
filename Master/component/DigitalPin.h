/*
 * Author: Klusjesman
 */

#ifndef __DigitalPin_H__
#define __DigitalPin_H__

#include <avr/io.h>


class DigitalPin
{
	public:
		enum IOMode {None,Input,Output};
		
		private:
		uint8_t					pin;
		volatile uint8_t *		port;
		IOMode					iomode;
	
	
	private:
		DigitalPin();
		
		void init(void);
		friend void swap(DigitalPin& dst, const DigitalPin& src);

	public:
		DigitalPin(const DigitalPin &c);
		DigitalPin& operator=(const DigitalPin &c);	
		
		DigitalPin(volatile uint8_t * port, uint8_t pin);
		DigitalPin(volatile uint8_t * port, uint8_t pin, IOMode iomode);
		~DigitalPin();
		
		void makeInput(void);
		void makeOutput(void);
		
		void write(bool val);
		bool read();
};

#endif //__DigitalPin_H__
