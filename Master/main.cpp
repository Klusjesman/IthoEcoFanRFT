/*
 * Author: Klusjesman
 *
 * Tested with STK500 + ATMega328P
 */


/*
TODO
- CC1101 RX fifo last byte bug (see errata)
- parse message1 (previous command, counters, verify device id)
- parse message2
- send (message1, message2)
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
	
	//ignore Itho message2
	//rf.setMessage2Requirement(false);
	
	
	debug.serOut("start\n");
	sei();

	while (1==1)
	{

		if (rf.checkForNewPacket())
		{
			packet = rf.getLastPacket();

			//show device id
			debug.serOut("device id: ");
			
			for (int i=0;i<3;i++)
			{
				debug.serOutIntToHex(packet.deviceId[i]);
				debug.serOut(",");
			}
			debug.serOut("\n");
			
			//show command
			switch (packet.command)
			{
				case unknown:
					debug.serOut("unknown\n");
					break;
				case low:
					debug.serOut("low\n");
					break;
				case medium:
					debug.serOut("medium\n");
					break;
				case full:
					debug.serOut("full\n");
					break;
				case timer1:
					debug.serOut("timer1\n");
					break;
				case timer2:
					debug.serOut("timer2\n");
					break;
				case timer3:
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

	}

}


