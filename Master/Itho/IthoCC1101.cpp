/*
 * Author: Klusjesman
 */

#include "IthoCC1101.h"
#include <string.h>
#include "../time/delay.h"
#include "../suart/SerialDebug.h"

// default constructor
IthoCC1101::IthoCC1101(SPI *spi, uint8_t counter, uint8_t sendTries) : CC1101(spi)
{
	this->outIthoPacket.counter = counter;
	this->outIthoPacket.previous = low;
	this->sendTries = sendTries;
	this->receiveState = ExpectMessage1;
	this->isMessage2Required = true;
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
	writeRegister(CC1101_PKTLEN , 16);		//16 bytes message (sync at beginning of message is removed by CC1101)
	writeRegister(CC1101_PKTCTRL0 ,0x00);
	writeRegister(CC1101_SYNC1 ,170);		//message1 byte2
	writeRegister(CC1101_SYNC0 ,173);		//message1 byte3
	writeRegister(CC1101_MDMCFG2 ,0x02);
	writeRegister(CC1101_PKTCTRL1 ,0x00);	
	
	writeCommand(CC1101_SRX);				//switch to RX state

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
	writeRegister(CC1101_PKTLEN ,44);			//44 bytes message (sync at beginning of message is removed by CC1101)
	writeRegister(CC1101_PKTCTRL0 ,0x00);
	writeRegister(CC1101_SYNC1 ,170);			//message2 byte6
	writeRegister(CC1101_SYNC0 ,171);			//message2 byte7
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
			length = receiveData(&inMessage1, 16);
					
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
			length = receiveData(&inMessage2, 44);
						
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
	if (inMessage1.data[12] != 170)	
	{
		return false;
	}
	
	return true;
}

bool IthoCC1101::isValidMessage2()
{
	if (inMessage2.data[37] != 170) 
	{
		return false;
	}
	
	return true;
}

void IthoCC1101::parseReceivedPackets()
{
	parseMessage1();

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
	inIthoPacket.deviceId[0] = inMessage1.data[2];
	inIthoPacket.deviceId[1] = inMessage1.data[3];
	inIthoPacket.deviceId[2] = inMessage1.data[4] & 0b11111110;	//last bit is part of command
	
	//copy command data from packet
	//message1 command starts at index 5, last bit!
	uint8_t commandBytes[7];
	commandBytes[0] = inMessage1.data[5] & 0b00000001;
	commandBytes[1] = inMessage1.data[6];
	commandBytes[2] = inMessage1.data[7];
	commandBytes[3] = inMessage1.data[8];
	commandBytes[4] = inMessage1.data[9];
	commandBytes[5] = inMessage1.data[10];
	commandBytes[6] = inMessage1.data[11];
	
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
	inIthoPacket.command = unknown;
	if (isFullCommand) inIthoPacket.command = full;
	if (isMediumCommand) inIthoPacket.command = medium;
	if (isLowCommand) inIthoPacket.command = low;
	if (isTimer1Command) inIthoPacket.command = timer1;
	if (isTimer2Command) inIthoPacket.command = timer2;
	if (isTimer3Command) inIthoPacket.command = timer3;
	if (isJoinCommand) inIthoPacket.command = join;
	if (isLeaveCommand) inIthoPacket.command = leave;	
	
	//previous command
	debug.serOutInt(inMessage1.data[14]);	debug.serOut("\n");
	debug.serOutInt(inMessage1.data[15]);	debug.serOut("\n");
	/*
	byte14:
	77 = join
	82 = leave
	85 = low,full,med,t1,t2,t3
	
	byte15:
	??? the last byte is always corrupted, maybe due to incorrect RX settings. 
	*/
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
		
	//counter1
	uint8_t row0 = inMessage2.data[16];
	uint8_t row1 = inMessage2.data[17];
	uint8_t row2 = inMessage2.data[18] & 0b11110000;	//4 bits are part of command
	inIthoPacket.counter = calculateMessageCounter(row0, row1, row2);
	
	//copy command data from packet
	uint8_t commandBytes[15];
	commandBytes[0] = inMessage2.data[18] & 0b00001111;	//4 bits are part of counter1
	commandBytes[1] = inMessage2.data[19];
	commandBytes[2] = inMessage2.data[20];
	commandBytes[3] = inMessage2.data[21];
	commandBytes[4] = inMessage2.data[22];		
	commandBytes[5] = inMessage2.data[23];		
	commandBytes[6] = inMessage2.data[24];		
	commandBytes[7] = inMessage2.data[25];		
	commandBytes[8] = inMessage2.data[26];		
	commandBytes[9] = inMessage2.data[27];		
	commandBytes[10] = inMessage2.data[28];		
	commandBytes[11] = inMessage2.data[29];
	commandBytes[12] = inMessage2.data[30];		
	commandBytes[13] = inMessage2.data[31];		
	commandBytes[14] = inMessage2.data[32];				
						
	//match received commandBytes with known command bytes
	for (int i=0; i<15; i++)
	{
		if (commandBytes[i] != ithoMessage2FullCommandBytes[i]) isFullCommand = false;
		if (commandBytes[i] != ithoMessage2MediumCommandBytes[i]) isMediumCommand = false;
		if (commandBytes[i] != ithoMessage2LowCommandBytes[i]) isLowCommand = false;
		if (commandBytes[i] != ithoMessage2Timer1CommandBytes[i]) isTimer1Command = false;
		if (commandBytes[i] != ithoMessage2Timer2CommandBytes[i]) isTimer2Command = false;
		if (commandBytes[i] != ithoMessage2Timer3CommandBytes[i]) isTimer3Command = false;
		if (commandBytes[i] != ithoMessage2JoinCommandBytes[i]) isJoinCommand = false;
		if (commandBytes[i] != ithoMessage2LeaveCommandBytes[i]) isLeaveCommand = false;
	}	
		
	//determine command
	inIthoPacket.command = unknown;
	if (isFullCommand) inIthoPacket.command = full;
	if (isMediumCommand) inIthoPacket.command = medium;
	if (isLowCommand) inIthoPacket.command = low;
	if (isTimer1Command) inIthoPacket.command = timer1;
	if (isTimer2Command) inIthoPacket.command = timer2;
	if (isTimer3Command) inIthoPacket.command = timer3;
	if (isJoinCommand) inIthoPacket.command = join;
	if (isLeaveCommand) inIthoPacket.command = leave;	
	
	
	//bug detection
	if (calculateMessage2Byte24(inIthoPacket.counter) != inMessage2.data[16])
		debug.serOut("error byte24\n");
	if (calculateMessage2Byte25(inIthoPacket.counter) != inMessage2.data[17])
		debug.serOut("error byte25\n");
	if (calculateMessage2Byte26(inIthoPacket.counter) != (inMessage2.data[18] & 0b11110000))
		debug.serOut("error byte26\n");
	if (calculateMessage2Byte41(inIthoPacket.counter, inIthoPacket.command) != inMessage2.data[33])
		debug.serOut("error byte41\n");
	if (calculateMessage2Byte42(inIthoPacket.counter, inIthoPacket.command) != inMessage2.data[34])
		debug.serOut("error byte42\n");
	if (calculateMessage2Byte43(inIthoPacket.counter, inIthoPacket.command) != inMessage2.data[35])
		debug.serOut("error byte43\n");
}

void IthoCC1101::sendCommand(IthoCommand command)
{
	CC1101Packet outMessage1;
	CC1101Packet outMessage2;
	
	//update itho packet data
	outIthoPacket.previous = outIthoPacket.command;
	outIthoPacket.command = command;
	outIthoPacket.counter += 1;
	
	//get message1 bytes
	createMessage1(&outIthoPacket, &outMessage1);
	
	//get message2 bytes
	createMessage2(&outIthoPacket, &outMessage2);
	
	//send messages
	for (int i=0;i<sendTries;i++)
	{
		//message1
		initSendMessage1();
		sendData(outMessage1);
		
		delay_ms(4);
		
		//message2
		initSendMessage2();
		sendData(outMessage2);
		
		finishTransfer();
		delay_ms(40);
	}
}

void IthoCC1101::createMessage1(IthoPacket *itho, CC1101Packet *packet)
{
	/************************************************************************/
	/* this function is not tested!                                                                     */
	/************************************************************************/

	packet->length = 20;

	//fixed
	packet->data[0] = 170;
	packet->data[1] = 170;
	packet->data[2] = 170;	
	packet->data[3] = 173;
	
	//device id ??
	packet->data[4] = 51;
	packet->data[5] = 83;	
	packet->data[6] = itho->deviceId[0];
	packet->data[7] = itho->deviceId[1];
	packet->data[8] = itho->deviceId[2];
	packet->data[9] = 204;

	//command
	uint8_t *commandBytes = getMessage1CommandBytes(itho->command);
	packet->data[9] = packet->data[9] | commandBytes[0];	//only last bit is set
	packet->data[10] = commandBytes[1];
	packet->data[11] = commandBytes[2];
	packet->data[12] = commandBytes[3];
	packet->data[13] = commandBytes[4];
	packet->data[14] = commandBytes[5];
	packet->data[15] = commandBytes[6];
	
	//fixed
	packet->data[16] = 170;
	packet->data[17] = 171;
	
	//previous command
	packet->data[18] = getMessage1Byte18(itho->previous);
	packet->data[19] = 77;
}

void IthoCC1101::createMessage2(IthoPacket *itho, CC1101Packet *packet)
{
	/************************************************************************/
	/* this function is not tested!                                                                     */
	/************************************************************************/
	
	packet->length = 50;
	
	//fixed
	packet->data[0] = 170;
	packet->data[1] = 170;
	packet->data[2] = 170;
	packet->data[3] = 170;
	packet->data[4] = 170;
	packet->data[5] = 170;
	packet->data[6] = 170;				
	packet->data[7] = 171;	
	packet->data[8] = 254;				
	packet->data[9] = 0;				
	packet->data[10] = 179;				
	packet->data[11] = 42;				
	packet->data[12] = 171;				
	packet->data[13] = 42;				
	packet->data[14] = 149;				
	packet->data[15] = 154;				
	
	//device id???
	packet->data[16] = 0;
	packet->data[17] = 0;
	packet->data[18] = 0;
	packet->data[19] = 0;
	packet->data[20] = 0;				
	packet->data[21] = 0;
	packet->data[22] = 0;
	packet->data[23] = 0;
	
	//counter bytes
	packet->data[24] = calculateMessage2Byte24(itho->counter);
	packet->data[25] = calculateMessage2Byte25(itho->counter);
	packet->data[26] = calculateMessage2Byte26(itho->counter);

	//command
	uint8_t *commandBytes = getMessage2CommandBytes(itho->command);
	packet->data[26] = packet->data[26] | commandBytes[0];
	packet->data[27] = commandBytes[1];
	packet->data[28] = commandBytes[2];
	packet->data[29] = commandBytes[3];
	packet->data[30] = commandBytes[4];
	packet->data[31] = commandBytes[5];
	packet->data[32] = commandBytes[6];
	packet->data[33] = commandBytes[7];
	packet->data[34] = commandBytes[8];
	packet->data[35] = commandBytes[9];
	packet->data[36] = commandBytes[10];
	packet->data[37] = commandBytes[11];
	packet->data[38] = commandBytes[12];
	packet->data[39] = commandBytes[13];	
	packet->data[40] = commandBytes[14];

	//counter bytes
	packet->data[41] = calculateMessage2Byte41(itho->counter, itho->command);
	packet->data[42] = calculateMessage2Byte42(itho->counter, itho->command);
	packet->data[43] = calculateMessage2Byte43(itho->counter, itho->command);
	
	//fixed
	packet->data[44] = 172;
	packet->data[45] = 170;
	packet->data[46] = 170;
	packet->data[47] = 170;
	packet->data[48] = 170;
	packet->data[49] = 170;
}

//calculate 0-255 number out of 3 counter bytes
uint8_t IthoCC1101::calculateMessageCounter(uint8_t byte24, uint8_t byte25, uint8_t byte26)
{
	uint8_t result;
	
	uint8_t a = getCounterIndex(&counterBytes24a[0],2,byte24 & 0b00000011);	//last 2 bits only
	uint8_t b = getCounterIndex(&counterBytes24b[0],8,byte24 & 0b11111100);	//first 6 bits
	uint8_t c = getCounterIndex(&counterBytes25[0],8,byte25);
	uint8_t d = getCounterIndex(&counterBytes26[0],2,byte26);
	
	result = (a * 128) + (b * 16) + (d * 8) + c;
	
	return result;
}

uint8_t IthoCC1101::getMessage1Byte18(IthoCommand command)
{
	switch (command)
	{
		case join:
			return 77;
		
		case leave:
			return 82;
		
		default:
			return 85;
	}
}

uint8_t IthoCC1101::calculateMessage2Byte24(uint8_t counter)
{
	return counterBytes24a[(counter / 128)] | counterBytes24b[(counter % 128) / 16];	
}

uint8_t IthoCC1101::calculateMessage2Byte25(uint8_t counter)
{
	return counterBytes25[(counter % 16) % 8];
}

uint8_t IthoCC1101::calculateMessage2Byte26(uint8_t counter)
{
	return counterBytes26[(counter % 16) / 8];
}

uint8_t IthoCC1101::calculateMessage2Byte41(uint8_t counter, IthoCommand command)
{
	int var = 0;
	uint8_t hi = 0;
	
	switch (command)
	{
		case timer1:
		case timer3:
			hi = 160;
			var = 48 - command;
			if (counter < var) counter = 64 - counter;
			break;
			
		case join:
			hi = 96;
			counter = 0;
			break;
				
		case leave:
			hi = 160;
			counter = 0;
			break;
							
		case low:
		case medium:
		case full:
		case timer2:
			hi = 96;
			var = 48 - command;
			if (counter < var) counter = 74 - counter;
			break;		
	}

	return (hi | counterBytes41[((counter - var) % 64) / 16]);
}

uint8_t IthoCC1101::calculateMessage2Byte42(uint8_t counter, IthoCommand command)
{
	uint8_t result;
	
	if (command == join || command == leave)
	{
		counter = 1;
	}
	else
	{
		counter += command;
	}

	result = counterBytes42[counter / 64];

	if (counter % 2 == 1) result -= 1;
		
	return result;
}

uint8_t IthoCC1101::calculateMessage2Byte43(uint8_t counter, IthoCommand command)
{
	switch (command)
	{
		case medium:
			break;
				
		case low:
		case timer2:
			if (counter % 2 == 0) counter -= 1;
			break;
			
		case full:
			counter += 2;
			if (counter % 2 == 0) counter -= 1;
			break;
			
		case timer1:
			counter += 6;
			if (counter % 2 == 0) counter -= 1;		
			break;
			
		case timer3:
			counter += 10;
			if (counter % 2 == 0) counter -= 1;		
			break;
			
		case join:
		case leave:
			counter = 0;
			break;	
	}

	return counterBytes43[(counter % 16) / 2];
}

uint8_t* IthoCC1101::getMessage1CommandBytes(IthoCommand command)
{
	switch (command)
	{
		case full:
		return (uint8_t*)&ithoMessage1FullCommandBytes[0];
		case medium:
		return (uint8_t*)&ithoMessage1MediumCommandBytes[0];
		case low:
		return (uint8_t*)&ithoMessage1LowCommandBytes[0];
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

uint8_t* IthoCC1101::getMessage2CommandBytes(IthoCommand command)
{
	switch (command)
	{
		case full:
		return (uint8_t*)&ithoMessage2FullCommandBytes[0];
		case medium:
		return (uint8_t*)&ithoMessage2MediumCommandBytes[0];
		case low:
		return (uint8_t*)&ithoMessage2LowCommandBytes[0];
		case timer1:
		return (uint8_t*)&ithoMessage2Timer1CommandBytes[0];
		case timer2:
		return (uint8_t*)&ithoMessage2Timer2CommandBytes[0];
		case timer3:
		return (uint8_t*)&ithoMessage2Timer3CommandBytes[0];
		case join:
		return (uint8_t*)&ithoMessage2JoinCommandBytes[0];
		case leave:
		return (uint8_t*)&ithoMessage2LeaveCommandBytes[0];
	}
}

//lookup value in array
uint8_t IthoCC1101::getCounterIndex(const uint8_t *arr, uint8_t length, uint8_t value)
{
	for (uint8_t i=0; i<length; i++)
	if (arr[i] == value)
	return i;
	
	//-1 should never be returned!
	return -1;
}