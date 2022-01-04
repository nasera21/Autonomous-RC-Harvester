/* EECS 452 Serial Car Wrapper
Team 8 : John Wolvereene*/

/*Transmitted message to Rpi is in format:
	Start 8 BITS = IMPOSSIBLE VALUE (DE)
	First message 8 bits =  right sensor
	Second message 8 bits = Front right sensor
	end 8 BITS = IMPOSSIBLE VALUE (FF)
	*/

/*
	Received message from Rpi is in format:
	START 8 BITS = IMPOSSIBLE VALUE (DE)
	First message 8 bits = Distance calculated
	end 8 BITS = IMPOSSIBLE VALUE (FF)
*/

#ifndef CAR_SERIAL_H
#define CAR_SERIAL_H

#include "Arduino.h"

class carSerial {
public:
	const uint16_t start = 0x1400;
	const uint16_t end = 0xFFFF;
	int* output;
	byte MSB_message;
	byte LSB_message;

//Default Constructor default baudrate: 9600
	carSerial();
// constructor if baud rate is not default
	carSerial(int baudrate);

	void init();

	void init(int baudrate);
	
	//Input: uint16 data value
	//Output: 8 most significant bits of input
	byte split16to8MSB(uint16_t number);

	//Input: uint16 data value
	//Output: 8 least significant bits of input
	byte split16to8LSB(uint16_t number);

	//Input: Distance sensor messages (2 uint16 numbers)
	//This function sends 4 uint16 messages from Arduino to Raspberry Pi throigh 8 bit packets
	void send_message16(uint16_t sensorMessage1, uint16_t sensorMessage2);

	//Input: uinnt8 message
	//This function sends an 8 bit packet to the Raspberry Pi
	void send_message8(byte message);

	//This function ensures that the UART protocol is satisfied
	//If we lose a packet anywhere in this process, the message is dropped
	int check_format();

	


private:

	//Function for Arduino to wait for message from Raspberry Pi
	void waitForByte();

};
#endif
