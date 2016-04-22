/*
 * Author: Klusjesman
 */

#ifndef ITHOPACKET_H_
#define ITHOPACKET_H_


typedef enum IthoCommand
{

	timer3 = 0,		//??
	
	timer2 = 3,		//??
	timer1 = 7,		//??	
	
	
			
	full = 11,
	medium = 12,	
	low = 13,
	
	join = 22,		//??
	leave = 23,		//??

	unknown = 32	
};


class IthoPacket
{
	public:
		uint8_t deviceId[3];	//not sure of this is correct
		IthoCommand command;
		
		uint8_t counter; //0-255	
};


#endif /* ITHOPACKET_H_ */