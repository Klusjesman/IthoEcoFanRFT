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
		uint8_t deviceId[3];	//not sure of this is correct
		IthoCommand command;
		
		uint8_t counter; //0-255	
};


#endif /* ITHOPACKET_H_ */