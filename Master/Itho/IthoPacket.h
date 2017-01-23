/*
 * Author: Klusjesman
 */

#ifndef ITHOPACKET_H_
#define ITHOPACKET_H_


typedef enum IthoMessageType
{
	msg_unknown = 0,
	msg_control = 1,
	msg_join = 2,
	msg_leave = 3
};

//do not change enum because they are used in calculations!
typedef enum IthoCommand
{
	unknown = 0,
		
	join = 4,
	leave = 8,
		
	rft_standby = 34,
	rft_low = 35,	
	rft_medium = 36,	
	rft_high = 37,
	rft_fullpower = 38,
	
	rft_timer1 = 41,
	rft_timer2 = 51,
	rft_timer3 = 61,

	//duco c system remote
	duco_standby = 251,
	duco_low = 252,
	duco_medium = 253,
	duco_high = 254
};

class IthoPacket
{
	public:
		uint8_t deviceId1[6];
		uint8_t deviceId2[8];
		IthoMessageType messageType;
		IthoCommand command;
		IthoCommand previous;
		
		uint8_t counter;		//0-255, counter is increased on every remote button press
};


#endif /* ITHOPACKET_H_ */