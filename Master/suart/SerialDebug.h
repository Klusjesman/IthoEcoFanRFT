/*
 * Author: Klusjesman
 */

#ifndef __SERIALDEBUG_H__
#define __SERIALDEBUG_H__

#include <stdio.h>
#include <avr/io.h>
#include <stdlib.h>


class SerialDebug
{
	//variables
	public:
	protected:
	private:

	//functions
	public:
		void serOut(uint8_t c);
		void serOut(const char* str);
		void serOut(const uint8_t str[], uint16_t size);
		void serOutInt(int16_t i);
		void serOutIntToHex(uint32_t i);
		void serOutDouble(double d, uint8_t precision);
		
		SerialDebug();
		~SerialDebug();
	protected:
	private:
		SerialDebug( const SerialDebug &c );
		SerialDebug& operator=( const SerialDebug &c );

}; //SerialDebug

extern SerialDebug debug;

#endif //__SERIALDEBUG_H__
