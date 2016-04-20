/*
 * Author: Klusjesman
 */

#ifndef __ITHOCC1101_H__
#define __ITHOCC1101_H__

#include <stdio.h>
#include "CC1101.h"
#include "IthoPacket.h"
#include "../time/millis.h"


//pa table settings
const uint8_t ithoPaTableSend[8] = {0x6F, 0x26, 0x2E, 0x8C, 0x87, 0xCD, 0xC7, 0xC0};
const uint8_t ithoPaTableReceive[8] = {0x6F, 0x26, 0x2E, 0x7F, 0x8A, 0x84, 0xCA, 0xC4};

//message 1 commands
const uint8_t ithoMessage1FullCommandBytes[] = {1,84,213,85,50,203,52};
const uint8_t ithoMessage1MediumCommandBytes[] = {1,84,213,85,74,213,52};
const uint8_t ithoMessage1LowCommandBytes[] = {1,84,213,85,83,83,84};	
const uint8_t ithoMessage1Timer1CommandBytes[] = {1,83,83,84,204,202,180};	
const uint8_t ithoMessage1Timer2CommandBytes[] = {1,83,83,83,53,52,180};		
const uint8_t ithoMessage1Timer3CommandBytes[] = {1,83,83,82,173,82,180};			
const uint8_t ithoMessage1JoinCommandBytes[] = {0,170,171,85,84,202,180};	
const uint8_t ithoMessage1LeaveCommandBytes[] = {0,170,173,85,83,43,84};	

//message 1 previous commands
//TODO
/*
const uint8_t ithoMessage1PreviousFullCommandBytes[] = {1,84};
const uint8_t ithoMessage1PreviousMediumCommandBytes[] = {1,84};
const uint8_t ithoMessage1PreviousLowCommandBytes[] = {1,84};
const uint8_t ithoMessage1PreviousTimer1CommandBytes[] = {1,83};
const uint8_t ithoMessage1PreviousTimer2CommandBytes[] = {1,83};
const uint8_t ithoMessage1PreviousTimer3CommandBytes[] = {1,83};
const uint8_t ithoMessage1PreviousJoinCommandBytes[] = {0,170};
const uint8_t ithoMessage1PreviousLeaveCommandBytes[] = {0,170};
*/

typedef enum IthoReceiveStates
{
	ExpectMessage1,
	ExpectMessage2
};



class IthoCC1101 : public CC1101
{
	private:
		//receive
		IthoReceiveStates receiveState;
		millis_t lastMessage1Received;
		bool isMessage2Required;
		CC1101PACKET packet1;
		CC1101PACKET packet2;		
		IthoPacket ithoPacket;
		
	//functions
	public:
		IthoCC1101(SPI *spi);
		~IthoCC1101();
		
		void initSendMessage1();
		void initSendMessage2();
		void finishTransfer();
		
		void initReceive();
		void setMessage2Requirement(bool message2Required);
		
		bool checkForNewPacket();
		IthoPacket getLastPacket() { return ithoPacket; }		
				
	protected:
	private:
		IthoCC1101();
		IthoCC1101( const IthoCC1101 &c );
		IthoCC1101& operator=( const IthoCC1101 &c );
		
		bool isValidMessage1();
		bool isValidMessage2();	
			
		void initReceiveMessage1();
		void initReceiveMessage2();	
		
		void parseReceivedPackets();
		
		void createMessage1(IthoPacket *packet);
		uint8_t* getMessage1CommandBytes(IthoCommand command);

}; //IthoCC1101


extern volatile uint32_t data1[];

#endif //__ITHOCC1101_H__
