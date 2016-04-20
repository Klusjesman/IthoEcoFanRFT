/*
 * Author: Klusjesman
 */

#include "IthoCC1101.h"
#include <string.h>
#include "../time/delay.h"
#include "../suart/SerialDebug.h"

// default constructor
IthoCC1101::IthoCC1101(SPI *spi) : CC1101(spi)
{
	receiveState = ExpectMessage1;
	isMessage2Required = true;
} //IthoCC1101

// default destructor
IthoCC1101::~IthoCC1101()
{
} //~IthoCC1101

void IthoCC1101::initSendMessage1()
{
	/*
	Configuration reverse engineered from remote print
		
	Base frequency		868.299866MHz
	Channel				0
	Channel spacing		199.951172kHz
	Carrier frequency	868.299866MHz
	Xtal frequency		26.000000MHz
	Data rate			8.00896kBaud
	Manchester			disabled
	Modulation			2-FSK
	Deviation			25.390625kHz
	TX power			?
	PA ramping			enabled
	Whitening			disabled
	*/
	writeCommand(CC1101_SRES);
	delay_us(1);
	writeRegister(CC1101_IOCFG0 ,0x2E);		//High impedance (3-state)
	writeRegister(CC1101_FREQ2 ,0x21);		//00100001	878MHz-927.8MHz
	writeRegister(CC1101_FREQ1 ,0x65);		//01100101
	writeRegister(CC1101_FREQ0 ,0x6A);		//01101010
	writeRegister(CC1101_MDMCFG4 ,0x07);	//00000111
	writeRegister(CC1101_MDMCFG3 ,0x43);	//01000011
	writeRegister(CC1101_MDMCFG2 ,0x00);	//00000000	2-FSK, no manchester encoding/decoding, no preamble/sync
	writeRegister(CC1101_MDMCFG1 ,0x22);	//00100010
	writeRegister(CC1101_MDMCFG0 ,0xF8);	//11111000
	writeRegister(CC1101_CHANNR ,0x00);		//00000000
	writeRegister(CC1101_DEVIATN ,0x40);	//01000000
	writeRegister(CC1101_FREND0 ,0x17);		//00010111	use index 7 in PA table
	writeRegister(CC1101_MCSM0 ,0x18);		//00011000	PO timeout Approx. 146탎 - 171탎, Auto calibrate When going from IDLE to RX or TX (or FSTXON)
	writeRegister(CC1101_FSCAL3 ,0xA9);		//10101001
	writeRegister(CC1101_FSCAL2 ,0x2A);		//00101010
	writeRegister(CC1101_FSCAL1 ,0x00);		//00000000
	writeRegister(CC1101_FSCAL0 ,0x11);		//00010001
	writeRegister(CC1101_FSTEST ,0x59);		//01011001	For test only. Do not write to this register.
	writeRegister(CC1101_TEST2 ,0x81);		//10000001	For test only. Do not write to this register.
	writeRegister(CC1101_TEST1 ,0x35);		//00110101	For test only. Do not write to this register.
	writeRegister(CC1101_TEST0 ,0x0B);		//00001011	For test only. Do not write to this register.
	writeRegister(CC1101_PKTCTRL0 ,0x12);	//00010010	Enable infinite length packets, CRC disabled, Turn data whitening off, Serial Synchronous mode
	writeRegister(CC1101_ADDR ,0x00);		//00000000
	writeRegister(CC1101_PKTLEN ,0xFF);		//11111111	//Not used, no hardware packet handling

	//0x6F,0x26,0x2E,0x8C,0x87,0xCD,0xC7,0xC0
	writeBurstRegister(CC1101_PATABLE | CC1101_WRITE_BURST, (uint8_t*)ithoPaTableSend, 8);

	writeCommand(CC1101_SIDLE);
	writeCommand(CC1101_SIDLE);
	writeCommand(CC1101_SIDLE);

	writeRegister(CC1101_MDMCFG4 ,0x08);	//00001000
	writeRegister(CC1101_MDMCFG3 ,0x43);	//01000011
	writeRegister(CC1101_DEVIATN ,0x40);	//01000000
	writeRegister(CC1101_IOCFG0 ,0x2D);		//GDO0_Z_EN_N. When this output is 0, GDO0 is configured as input (for serial TX data).
	writeRegister(CC1101_IOCFG1 ,0x0B);		//Serial Clock. Synchronous to the data in synchronous serial mode.
	
	writeCommand(CC1101_STX);
	writeCommand(CC1101_SIDLE);
	delay_us(1);
	writeCommand(CC1101_SIDLE);

	writeRegister(CC1101_MDMCFG4 ,0x08);	//00001000
	writeRegister(CC1101_MDMCFG3 ,0x43);	//01000011
	writeRegister(CC1101_DEVIATN ,0x40);	//01000000
	writeRegister(CC1101_IOCFG0 ,0x2D);		//GDO0_Z_EN_N. When this output is 0, GDO0 is configured as input (for serial TX data).
	writeRegister(CC1101_IOCFG1 ,0x0B);		//Serial Clock. Synchronous to the data in synchronous serial mode.
	
	writeCommand(CC1101_STX);	
}

void IthoCC1101::initSendMessage2()
{
	//finishTransfer();
	writeCommand(CC1101_SIDLE);
	delay_us(1);
	writeRegister(CC1101_IOCFG0 ,0x2E);
	delay_us(1);
	writeRegister(CC1101_IOCFG1 ,0x2E);
	delay_us(1);
	writeCommand(CC1101_SIDLE);
	writeCommand(CC1101_SPWD);
	delay_us(2);
	
	/*
	Configuration reverse engineered from remote print
		
	Base frequency		868.299866MHz
	Channel				0
	Channel spacing		199.951172kHz
	Carrier frequency	868.299866MHz
	Xtal frequency		26.000000MHz
	Data rate			38.3835kBaud
	Manchester			disabled
	Modulation			2-FSK
	Deviation			50.781250kHz
	TX power			?
	PA ramping			enabled
	Whitening			disabled
	*/	
	writeCommand(CC1101_SRES);
	delay_us(1);
	writeRegister(CC1101_IOCFG0 ,0x2E);		//High impedance (3-state)
	writeRegister(CC1101_FREQ2 ,0x21);		//00100001	878MHz-927.8MHz
	writeRegister(CC1101_FREQ1 ,0x65);		//01100101
	writeRegister(CC1101_FREQ0 ,0x6A);		//01101010	
	writeRegister(CC1101_MDMCFG4 ,0x5A);	//difference compared to message1
	writeRegister(CC1101_MDMCFG3 ,0x83);	//difference compared to message1
	writeRegister(CC1101_MDMCFG2 ,0x00);	//00000000	2-FSK, no manchester encoding/decoding, no preamble/sync
	writeRegister(CC1101_MDMCFG1 ,0x22);	//00100010
	writeRegister(CC1101_MDMCFG0 ,0xF8);	//11111000
	writeRegister(CC1101_CHANNR ,0x00);		//00000000
	writeRegister(CC1101_DEVIATN ,0x50);	//difference compared to message1
	writeRegister(CC1101_FREND0 ,0x17);		//00010111	use index 7 in PA table
	writeRegister(CC1101_MCSM0 ,0x18);		//00011000	PO timeout Approx. 146탎 - 171탎, Auto calibrate When going from IDLE to RX or TX (or FSTXON)
	writeRegister(CC1101_FSCAL3 ,0xA9);		//10101001
	writeRegister(CC1101_FSCAL2 ,0x2A);		//00101010
	writeRegister(CC1101_FSCAL1 ,0x00);		//00000000
	writeRegister(CC1101_FSCAL0 ,0x11);		//00010001
	writeRegister(CC1101_FSTEST ,0x59);		//01011001	For test only. Do not write to this register.
	writeRegister(CC1101_TEST2 ,0x81);		//10000001	For test only. Do not write to this register.
	writeRegister(CC1101_TEST1 ,0x35);		//00110101	For test only. Do not write to this register.
	writeRegister(CC1101_TEST0 ,0x0B);		//00001011	For test only. Do not write to this register.
	writeRegister(CC1101_PKTCTRL0 ,0x12);	//00010010	Enable infinite length packets, CRC disabled, Turn data whitening off, Serial Synchronous mode
	writeRegister(CC1101_ADDR ,0x00);		//00000000
	writeRegister(CC1101_PKTLEN ,0xFF);		//11111111	//Not used, no hardware packet handling

	//0x6F,0x26,0x2E,0x8C,0x87,0xCD,0xC7,0xC0
	writeBurstRegister(CC1101_PATABLE | CC1101_WRITE_BURST, (uint8_t*)ithoPaTableSend, 8);

	//difference, message1 sends a STX here
	writeCommand(CC1101_SIDLE);
	writeCommand(CC1101_SIDLE);

	writeRegister(CC1101_MDMCFG4 ,0x5A);	//difference compared to message1
	writeRegister(CC1101_MDMCFG3 ,0x83);	//difference compared to message1
	writeRegister(CC1101_DEVIATN ,0x50);	//difference compared to message1
	writeRegister(CC1101_IOCFG0 ,0x2D);		//GDO0_Z_EN_N. When this output is 0, GDO0 is configured as input (for serial TX data).
	writeRegister(CC1101_IOCFG1 ,0x0B);		//Serial Clock. Synchronous to the data in synchronous serial mode.

	writeCommand(CC1101_STX);
	writeCommand(CC1101_SIDLE);

	writeRegister(CC1101_MDMCFG4 ,0x5A);	//difference compared to message1
	writeRegister(CC1101_MDMCFG3 ,0x83);	//difference compared to message1
	writeRegister(CC1101_DEVIATN ,0x50);	//difference compared to message1
	writeRegister(CC1101_IOCFG0 ,0x2D);		//GDO0_Z_EN_N. When this output is 0, GDO0 is configured as input (for serial TX data).
	writeRegister(CC1101_IOCFG1 ,0x0B);		//Serial Clock. Synchronous to the data in synchronous serial mode.

	writeCommand(CC1101_STX);
}

void IthoCC1101::finishTransfer()
{
	writeCommand(CC1101_SIDLE);
	delay_us(1);
	writeRegister(CC1101_IOCFG0 ,0x2E);
	writeRegister(CC1101_IOCFG1 ,0x2E);
	
	writeCommand(CC1101_SIDLE);
	writeCommand(CC1101_SPWD);
}

void IthoCC1101::initReceive()
{
	/*
	Configuration reverse engineered from RFT print
	
	Base frequency		868.299866MHz
	Channel				0
	Channel spacing		199.951172kHz
	Carrier frequency	868.299866MHz
	Xtal frequency		26.000000MHz
	Data rate			38.3835kBaud
	RX filter BW		325.000000kHz
	Manchester			disabled
	Modulation			2-FSK
	Deviation			50.781250kHz
	TX power			0x6F,0x26,0x2E,0x7F,0x8A,0x84,0xCA,0xC4
	PA ramping			enabled
	Whitening			disabled
	*/	
	writeCommand(CC1101_SRES);

	writeRegister(CC1101_TEST0 ,0x09);
	writeRegister(CC1101_FSCAL2 ,0x00);
	
	//0x6F,0x26,0x2E,0x7F,0x8A,0x84,0xCA,0xC4
	writeBurstRegister(CC1101_PATABLE | CC1101_WRITE_BURST, (uint8_t*)ithoPaTableReceive, 8);
	
	writeCommand(CC1101_SCAL);

	//wait for calibration to finish
	while ((readRegisterWithSyncProblem(CC1101_MARCSTATE, CC1101_STATUS_REGISTER)) != CC1101_MARCSTATE_IDLE);

	writeRegister(CC1101_FSCAL2 ,0x00);
	writeRegister(CC1101_MCSM0 ,0x18);			//no auto calibrate
	writeRegister(CC1101_FREQ2 ,0x21);
	writeRegister(CC1101_FREQ1 ,0x65);
	writeRegister(CC1101_FREQ0 ,0x6A);
	writeRegister(CC1101_IOCFG0 ,0x2E);			//High impedance (3-state)
	writeRegister(CC1101_IOCFG2 ,0x2E);			//High impedance (3-state)
	writeRegister(CC1101_FSCTRL1 ,0x06);
	writeRegister(CC1101_FSCTRL0 ,0x00);
	writeRegister(CC1101_MDMCFG4 ,0x5A);
	writeRegister(CC1101_MDMCFG3 ,0x83);
	writeRegister(CC1101_MDMCFG2 ,0x00);		//Enable digital DC blocking filter before demodulator, 2-FSK, Disable Manchester encoding/decoding, No preamble/sync 
	writeRegister(CC1101_MDMCFG1 ,0x22);		//Disable FEC
	writeRegister(CC1101_MDMCFG0 ,0xF8);
	writeRegister(CC1101_CHANNR ,0x00);
	writeRegister(CC1101_DEVIATN ,0x50);
	writeRegister(CC1101_FREND1 ,0x56);
	writeRegister(CC1101_FREND0 ,0x17);
	writeRegister(CC1101_MCSM0 ,0x18);			//no auto calibrate
	writeRegister(CC1101_FOCCFG ,0x16);
	writeRegister(CC1101_BSCFG ,0x6C);
	writeRegister(CC1101_AGCCTRL2 ,0x43);
	writeRegister(CC1101_AGCCTRL1 ,0x40);
	writeRegister(CC1101_AGCCTRL0 ,0x91);
	writeRegister(CC1101_FSCAL3 ,0xA9);
	writeRegister(CC1101_FSCAL2 ,0x2A);
	writeRegister(CC1101_FSCAL1 ,0x00);
	writeRegister(CC1101_FSCAL0 ,0x11);
	writeRegister(CC1101_FSTEST ,0x59);
	writeRegister(CC1101_TEST2 ,0x81);
	writeRegister(CC1101_TEST1 ,0x35);
	writeRegister(CC1101_TEST0 ,0x0B);
	writeRegister(CC1101_PKTCTRL1 ,0x04);		//No address check, Append two bytes with status RSSI/LQI/CRC OK, 
	writeRegister(CC1101_PKTCTRL0 ,0x32);		//Infinite packet length mode, CRC disabled for TX and RX, No data whitening, Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins 
	writeRegister(CC1101_ADDR ,0x00);
	writeRegister(CC1101_PKTLEN ,0xFF);
	writeRegister(CC1101_TEST0 ,0x09);
	writeRegister(CC1101_FSCAL2 ,0x00);

	writeCommand(CC1101_SCAL);

	//wait for calibration to finish
	while ((readRegisterWithSyncProblem(CC1101_MARCSTATE, CC1101_STATUS_REGISTER)) != CC1101_MARCSTATE_IDLE);

	writeRegister(CC1101_MCSM0 ,0x18);			//no auto calibrate
	
	writeCommand(CC1101_SIDLE);
	writeCommand(CC1101_SIDLE);
	
	writeRegister(CC1101_MDMCFG2 ,0x00);		//Enable digital DC blocking filter before demodulator, 2-FSK, Disable Manchester encoding/decoding, No preamble/sync 
	writeRegister(CC1101_IOCFG0 ,0x0D);			//Serial Data Output. Used for asynchronous serial mode.

	writeCommand(CC1101_SRX);
	
	while ((readRegisterWithSyncProblem(CC1101_MARCSTATE, CC1101_STATUS_REGISTER)) != CC1101_MARCSTATE_RX);
	
	initReceiveMessage1();
}

void IthoCC1101::initReceiveMessage1()
{
	uint8_t marcState;
	
	writeCommand(CC1101_SIDLE);	//idle
	
	//set datarate
	writeRegister(CC1101_MDMCFG4 ,0x08);
	writeRegister(CC1101_MDMCFG3 ,0x43);
	writeRegister(CC1101_DEVIATN ,0x40);
	
	//set fifo mode with fixed packet length and sync bytes
	writeRegister(CC1101_PKTLEN , 16);
	writeRegister(CC1101_PKTCTRL0 ,0x00);
	writeRegister(CC1101_SYNC1 ,170);
	writeRegister(CC1101_SYNC0 ,173);
	writeRegister(CC1101_MDMCFG2 ,0x02);
	writeRegister(CC1101_PKTCTRL1 ,0x00);	
	
	writeCommand(CC1101_SRX); //switch to RX state

	// Check that the RX state has been entered
	while (((marcState = readRegisterWithSyncProblem(CC1101_MARCSTATE, CC1101_STATUS_REGISTER)) & CC1101_BITS_MARCSTATE) != CC1101_MARCSTATE_RX)
	{
		if (marcState == CC1101_MARCSTATE_RXFIFO_OVERFLOW) // RX_OVERFLOW
			writeCommand(CC1101_SFRX); //flush RX buffer
	}
	
	receiveState = ExpectMessage1;
}

void IthoCC1101::initReceiveMessage2()
{
	uint8_t marcState;
	
	writeCommand(CC1101_SIDLE);	//idle
	
	//set datarate	
	writeRegister(CC1101_MDMCFG4 ,0x5A);
	writeRegister(CC1101_MDMCFG3 ,0x83);
	writeRegister(CC1101_DEVIATN ,0x50);
	
	//set fifo mode with fixed packet length and sync bytes
	writeRegister(CC1101_PKTLEN ,44);
	writeRegister(CC1101_PKTCTRL0 ,0x00);
	writeRegister(CC1101_SYNC1 ,170);
	writeRegister(CC1101_SYNC0 ,171);
	writeRegister(CC1101_MDMCFG2 ,0x02);
	writeRegister(CC1101_PKTCTRL1 ,0x00);		
	
	writeCommand(CC1101_SRX); //switch to RX state

	// Check that the RX state has been entered
	while (((marcState = readRegisterWithSyncProblem(CC1101_MARCSTATE, CC1101_STATUS_REGISTER)) & CC1101_BITS_MARCSTATE) != CC1101_MARCSTATE_RX)
	{
		if (marcState == CC1101_MARCSTATE_RXFIFO_OVERFLOW) // RX_OVERFLOW
			writeCommand(CC1101_SFRX); //flush RX buffer
	}
	
	receiveState = ExpectMessage2;
}

void IthoCC1101::setMessage2Requirement(bool message2Required)
{
	isMessage2Required = message2Required;
}

bool IthoCC1101::checkForNewPacket()
{
	bool result = false;	
	uint8_t length;
	
	switch (receiveState)
	{
		case ExpectMessage1:
			length = receiveData(&packet1, 16);
					
			//check if message1 is received
			if ((length > 0) && (isValidMessage1()))
			{
				if (isMessage2Required)
				{
					//switch to message2 RF settings
					initReceiveMessage2();
				}
				else
				{
					//ignore message2
					//extract data from message1 only
					parseReceivedPackets();
					result = true;
				}
				
				lastMessage1Received = millis();
			}
						
			break;
		
		case ExpectMessage2:
			length = receiveData(&packet2, 44);
						
			//check if message2 is received
			if ((length > 0) && (isValidMessage2()))
			{
				//switch back to message1 RF settings
				initReceiveMessage1();
				
				//extract data from message1 and message2
				parseReceivedPackets();
				
				//both messages are received
				result = true;
			}	
			else
			{
				//check if waiting timeout is reached (message2 is expected within 22ms after message1)
				if (millis() - lastMessage1Received > 24)
				{
					//switch back to message1 RF settings
					initReceiveMessage1();
				}
			}
					
			break;
	}
	
	return result;
}

bool IthoCC1101::isValidMessage1()
{
	if (packet1.data[12] != 170)	
	{
		return false;
	}
	
/*
		debug.serOutInt(packet1.data[14]);
		debug.serOut("\n");

		debug.serOutInt(packet1.data[15]);
		debug.serOut("\n");
			*/
	return true;
}

bool IthoCC1101::isValidMessage2()
{
	if (packet2.data[12] != 170)
	{
		return false;
	}
	
	/*
	for (int i=0; i<packet2.length;i++)
	{
		debug.serOutInt(packet2.data[i]);
		debug.serOut("\n");
	}
			
	debug.serOut("-----------------");
	*/
	
	return true;
}

void IthoCC1101::parseReceivedPackets()
{
	//parseMessage1();

/*
is the rft using both messages, maybe only one is required, probably the second one
- maybe the first message is used for compatibility with older/different systems only?
*/

	if (isMessage2Required)
	{
		parseMessage2();
	}
}

void IthoCC1101::parseMessage1()
{
	bool isFullCommand = true;
	bool isMediumCommand = true;
	bool isLowCommand = true;
	bool isTimer1Command = true;
	bool isTimer2Command = true;
	bool isTimer3Command = true;
	bool isJoinCommand = true;
	bool isLeaveCommand = true;
	
	//copy device id from packet
	// TODO: verify if this really is this the device id
	ithoPacket.deviceId[0] = packet1.data[2];
	ithoPacket.deviceId[1] = packet1.data[3];
	ithoPacket.deviceId[2] = packet1.data[4] & 0b11111110;	//last bit is part of command
	
	//copy command data from packet
	//message1 command starts at index 5, last bit!
	uint8_t commandBytes[7];
	commandBytes[0] = packet1.data[5] & 0b00000001;
	commandBytes[1] = packet1.data[6];
	commandBytes[2] = packet1.data[7];
	commandBytes[3] = packet1.data[8];
	commandBytes[4] = packet1.data[9];
	commandBytes[5] = packet1.data[10];
	commandBytes[6] = packet1.data[11];
	
	//match received commandBytes with known command bytes
	for (int i=0; i<7; i++)
	{
		if (commandBytes[i] != ithoMessage1FullCommandBytes[i]) isFullCommand = false;
		if (commandBytes[i] != ithoMessage1MediumCommandBytes[i]) isMediumCommand = false;
		if (commandBytes[i] != ithoMessage1LowCommandBytes[i]) isLowCommand = false;
		if (commandBytes[i] != ithoMessage1Timer1CommandBytes[i]) isTimer1Command = false;
		if (commandBytes[i] != ithoMessage1Timer2CommandBytes[i]) isTimer2Command = false;
		if (commandBytes[i] != ithoMessage1Timer3CommandBytes[i]) isTimer3Command = false;
		if (commandBytes[i] != ithoMessage1JoinCommandBytes[i]) isJoinCommand = false;
		if (commandBytes[i] != ithoMessage1LeaveCommandBytes[i]) isLeaveCommand = false;
	}
	
	//determine command
	ithoPacket.command = unknown;
	if (isFullCommand) ithoPacket.command = full;
	if (isMediumCommand) ithoPacket.command = medium;
	if (isLowCommand) ithoPacket.command = low;
	if (isTimer1Command) ithoPacket.command = timer1;
	if (isTimer2Command) ithoPacket.command = timer2;
	if (isTimer3Command) ithoPacket.command = timer3;
	if (isJoinCommand) ithoPacket.command = join;
	if (isLeaveCommand) ithoPacket.command = leave;	
}

void IthoCC1101::parseMessage2()
{
	bool isFullCommand = true;
	bool isMediumCommand = true;
	bool isLowCommand = true;
	bool isTimer1Command = true;
	bool isTimer2Command = true;
	bool isTimer3Command = true;
	bool isJoinCommand = true;
	bool isLeaveCommand = true;
		
	//match received commandBytes with known command bytes
	for (int i=0; i<14; i++)
	{
		if (packet2.data[19+i] != ithoMessage2FullCommandBytes[i]) isFullCommand = false;
		if (packet2.data[19+i] != ithoMessage2MediumCommandBytes[i]) isMediumCommand = false;
		if (packet2.data[19+i] != ithoMessage2LowCommandBytes[i]) isLowCommand = false;
		if (packet2.data[19+i] != ithoMessage2Timer1CommandBytes[i]) isTimer1Command = false;
		if (packet2.data[19+i] != ithoMessage2Timer2CommandBytes[i]) isTimer2Command = false;
		if (packet2.data[19+i] != ithoMessage2Timer3CommandBytes[i]) isTimer3Command = false;
		if (packet2.data[19+i] != ithoMessage2JoinCommandBytes[i]) isJoinCommand = false;
		if (packet2.data[19+i] != ithoMessage2LeaveCommandBytes[i]) isLeaveCommand = false;
	}	
		
	//determine command
	ithoPacket.command = unknown;
	if (isFullCommand) ithoPacket.command = full;
	if (isMediumCommand) ithoPacket.command = medium;
	if (isLowCommand) ithoPacket.command = low;
	if (isTimer1Command) ithoPacket.command = timer1;
	if (isTimer2Command) ithoPacket.command = timer2;
	if (isTimer3Command) ithoPacket.command = timer3;
	if (isJoinCommand) ithoPacket.command = join;
	if (isLeaveCommand) ithoPacket.command = leave;		
}

void IthoCC1101::createMessage1(IthoPacket *packet)
{
	uint8_t bytes[20];
	
	//fixed
	bytes[0] = 170;
	bytes[1] = 170;
	bytes[2] = 170;
	bytes[3] = 173;
	
	//device id ??
	bytes[4] = 51;
	bytes[5] = 83;	
	bytes[6] = packet->deviceId[0];
	bytes[7] = packet->deviceId[1];
	bytes[8] = packet->deviceId[2];
	bytes[9] = 204;

	//command
	uint8_t *commandBytes = getMessage1CommandBytes(packet->command);
	bytes[9] = bytes[9] & commandBytes[0];	//only last bit is set
	bytes[10] = commandBytes[1];
	bytes[11] = commandBytes[2];
	bytes[12] = commandBytes[3];
	bytes[13] = commandBytes[4];
	bytes[14] = commandBytes[5];
	bytes[15] = commandBytes[6];
	
	//fixed
	bytes[16] = 170;
	bytes[17] = 171;
	
	//previous command (not important)
	bytes[18] = 85;
	bytes[19] = 77;
}

uint8_t* IthoCC1101::getMessage1CommandBytes(IthoCommand command)
{
	switch (command)
	{
		case full:
			return (uint8_t*)&ithoMessage1FullCommandBytes[0];
		case medium:
			return (uint8_t*)&ithoMessage1FullCommandBytes[0];
		case low:
			return (uint8_t*)&ithoMessage1MediumCommandBytes[0];
		case timer1:
			return (uint8_t*)&ithoMessage1Timer1CommandBytes[0];
		case timer2:
			return (uint8_t*)&ithoMessage1Timer2CommandBytes[0];
		case timer3:
			return (uint8_t*)&ithoMessage1Timer3CommandBytes[0];
		case join:
			return (uint8_t*)&ithoMessage1JoinCommandBytes[0];
		case leave:
			return (uint8_t*)&ithoMessage1LeaveCommandBytes[0];
	}	
}