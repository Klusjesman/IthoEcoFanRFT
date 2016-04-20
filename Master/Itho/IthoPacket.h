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
		uint8_t deviceId[3];	//not sure of thise is correct
		IthoCommand command;
		IthoCommand previousCommand;
		
};


#endif /* ITHOPACKET_H_ */