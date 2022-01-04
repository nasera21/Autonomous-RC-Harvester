/* EECS 452 Serial Car Wrapper
Team 8 : John Wolvereene*/

/*Transmitted message to Rpi is in format:
	Start 8 BITS = IMPOSSIBLE VALUE (DE)
	First message 8 bits =  MSB sensor data
	Second message 8 bits = LSB sensor data
	end 8 BITS = IMPOSSIBLE VALUE (FF)
	*/

	/*
		Received message from Rpi is in format:
		START 8 BITS = IMPOSSIBLE VALUE (DE)
		First message 8 bits = Distance calculated
		end 8 BITS = IMPOSSIBLE VALUE (FF)
	*/

#include "carSerial.h"

carSerial::carSerial() {
	Serial.begin(9600);
}

carSerial::carSerial(int baudrate) {
	Serial.begin(baudrate);
}

void carSerial::init() {
	Serial.begin(9600);
}

void carSerial::init(int baudrate) {
	Serial.begin(baudrate);
}

byte carSerial::split16to8MSB(uint16_t number){
	int numberMSB= (number & 0xFF00) >> 8;
	return numberMSB;

}

byte carSerial::split16to8LSB(uint16_t number) {
	int numberLSB = number & 0xFF;
	return numberLSB;

}

void carSerial::send_message16(uint16_t sensorMessage1, uint16_t sensorMessage2) {
	/*
	split16to8(start);
	Serial.println("Number output send_message16:");
	Serial.println(output[0]);
	Serial.println(output[1]);
	*/
	send_message8(split16to8MSB(start));
	send_message8(split16to8LSB(start));
	send_message8(split16to8MSB(sensorMessage1));
	send_message8(split16to8LSB(sensorMessage1));
	send_message8(split16to8MSB(sensorMessage2));
	send_message8(split16to8LSB(sensorMessage2));
	send_message8(split16to8MSB(end));
	send_message8(split16to8LSB(end));

}

void carSerial::send_message8(byte message){
	Serial.write(message);
}


int carSerial::check_format() {
	waitForByte();
	byte inbyteMSB = Serial.read();
	waitForByte();
	byte inbyteLSB = Serial.read();
	uint16_t inbyte = uint16_t(inbyteMSB) <<8 + uint16_t(inbyteLSB);
	if (inbyte != start){
		return -1;
	}

	int messageData;
	waitForByte();
	messageData = Serial.read();

	waitForByte();
	inbyteMSB = Serial.read();
	waitForByte();
	inbyteLSB = Serial.read();
	inbyte = uint16_t(inbyteMSB) <<8 + uint16_t(inbyteLSB);
	if (inbyte != end){
		return -1;
	}
	return messageData;
}

void carSerial::waitForByte() {
	while (Serial.available() == 0) {
		delay(10);
	}
}
