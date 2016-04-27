/*
 * Author: Klusjesman
 */

#ifndef ITHOPACKET_H_
#define ITHOPACKET_H_


//do not change enum because they are used in calculations!
typedef enum IthoCommand 
{
	unknown = 0,
		
	join = 4,
	leave = 8,
				
	low = 35,	
	medium = 36,	
	full = 37,
	
	timer1 = 41,
	timer2 = 51,
	timer3 = 61
};


class IthoPacket
{
	public:
		uint8_t deviceId[8];
		IthoCommand command;
		IthoCommand previous;
		
		uint8_t counter;		//0-255, counter is increased on every remote button press
};


#endif /* ITHOPACKET_H_ */