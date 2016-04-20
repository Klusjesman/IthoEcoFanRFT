/* Optimized half-duplex serial uart implementation
 * timing within 2% using at 230.4kbps @ 8Mhz 
 * @author: Ralph Doncaster
 * @version: $Id$
 */

#ifndef __BBUART_H__
#define __BBUART_H__


#include <avr/io.h>
#include <stdlib.h>


#define BAUD_RATE 115200


#ifdef F_CPU
  /* account for integer truncation by adding 3/2 = 1.5 */
  #define TXDELAY (((F_CPU/BAUD_RATE)-7 +1.5)/3)
  #define RXDELAY (((F_CPU/BAUD_RATE)-5 +1.5)/3)
  #define RXROUNDED (((F_CPU/BAUD_RATE)-5 +2)/3)
#else
  #error CPU frequency F_CPU undefined
#endif


#if RXROUNDED > 127
  #error low baud rates unsupported - use higher BAUD_RATE
#endif

extern "C" void TxTimedByte(char, char);
extern "C" char RxTimedByte(char, char); /* 1.5 bit delay, 1 bit delay */


void TxByte(char C)
{
	TxTimedByte(C , TXDELAY);
}

char RxByte()
{
	return RxTimedByte((RXDELAY*1.5)-2.5, RXDELAY);
}

#endif //__BBUART_H__



