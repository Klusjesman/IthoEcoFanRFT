/*
 * Author: Klusjesman
 */

#ifndef ITHOPACKET_H_
#define ITHOPACKET_H_


typedef enum IthoCommand
{
	unknown = 0,
	low,
	medium,
	full,
	timer1,
	timer2,
	timer3,
	join,
	leave
};


class IthoPacket
{
	public:
		uint8_t deviceId[3];	//not sure of this is correct
		IthoCommand command;
		
		uint8_t counter1; //0-255
		
};


#endif /* ITHOPACKET_H_ */