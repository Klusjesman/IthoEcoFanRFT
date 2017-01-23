/*
 * Author: Klusjesman
 *
 * Tested with STK500 + ATMega328P + 14.7456MHz crystal
 * GCC-AVR compiler
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include "time/millis.h"
#include "time/delay.h"
#include "suart/SerialDebug.h"
#include "Itho/IthoCC1101.h"
#include "itho/IthoPacket.h"



int main(void)
{
	IthoPacket packet;
	millis_t last;
	
	millis_init();
	delay_ms(500);
	
	//slave select CC1101
	DigitalPin ss(&PORTB,2);
	
	//set up SPI
	SPI spi(&ss);
	spi.init();
	
	//init CC1101
	IthoCC1101 rf(&spi);
	rf.init();
	
	//set CC1101 registers
	rf.initReceive();
			
	debug.serOut("start\n");
	last = millis();
	sei();

	while (1==1)
	{

		if (rf.checkForNewPacket())
		{
			packet = rf.getLastPacket();
			
			//show counter
			debug.serOut("counter=");
			debug.serOutInt(packet.counter);
			debug.serOut(", ");
			
			//show command
			switch (packet.command)
			{
				case unknown:
					debug.serOut("unknown\n");
					break;
				case rft_fullpower:
					debug.serOut("fullpower\n");
					break;
				case rft_standby:
				case duco_standby:
					debug.serOut("standby\n");
					break;
				case rft_low:
				case duco_low:
					debug.serOut("low\n");
					break;
				case rft_medium:
				case duco_medium:
					debug.serOut("medium\n");
					break;
				case rft_high:
				case duco_high:
					debug.serOut("high\n");
					break;
				case rft_timer1:
					debug.serOut("timer1\n");
					break;
				case rft_timer2:
					debug.serOut("timer2\n");
					break;
				case rft_timer3:
					debug.serOut("timer3\n");
					break;
				case join:
					debug.serOut("join\n");
					break;
				case leave:
					debug.serOut("leave\n");
					break;
			}
				
		}
		
		if (millis() - last > 15000)
		{
			last = millis();
			rf.sendCommand(rft_fullpower);
			rf.initReceive();
		}

	}

}


