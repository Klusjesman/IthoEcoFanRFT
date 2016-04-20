/*
 * Author: Klusjesman
 */

#include "SerialDebug.h"
#include "BBuart.h"
#include <stdio.h>

// default constructor
SerialDebug::SerialDebug()
{
} //SerialDebug

// default destructor
SerialDebug::~SerialDebug()
{
} //~SerialDebug

void SerialDebug::serOut(const char* str)
{
	while (*str) TxByte (*str++);
}

void SerialDebug::serOut(const uint8_t str[], uint16_t size)
{
	for (uint16_t i = 0; i<size; i++)
		serOut(str[i]);
}

void SerialDebug::serOut(uint8_t c)
{
	TxByte(c);
}

void SerialDebug::serOutInt(int16_t i)
{
	uint8_t started = 0;
	uint16_t pow = 10000;

	if (i < 0)
	{
		serOut('-');
		i = -i;
	}

	while(pow >= 1)
	{
		if(i / pow > 0 || started || pow==1)
		{
			serOut((uint8_t) (i/pow) + '0');
			started = 1;
			i = i % pow;
		}

		pow = pow / 10;
	}
}

void SerialDebug::serOutDouble(double d, uint8_t precision)
{
	char output[16] = "\0";
	dtostrf(d,10,precision,&output[0]);
	serOut(output);
}

void SerialDebug::serOutIntToHex(uint32_t i)
{
	char str[16];
	sprintf(str, "0x%X", i);
	serOut(str);
}

SerialDebug debug;
