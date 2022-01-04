/*
 *  WRITTEN FOR: EECS 452 MDE Project Winter 2021
 *              Team: John Wolvereene
 *              Authors: Elisabeth Jahnke
 *
 */

#include "DistanceSensorLib.h"
#include <NewPing.h>


 //Default Construction -- Sets everything to Default Configuration
 // Default Configuration:
	// DISTANCE SENSOR 1 CONFIGURATION (RIGHT FRONT SENSOR):
	 //          Echo - D2
	 //          Trigger - D13 
	 // DISTANCE SENSOR 2 CONFIGURATION (FRONT SENSOR):
	 //          Echo - D11 (Check that these pins can be used)
	 //          Trigger - D9 (Check that these pins can be used) 
	//DISTANCE SENSOR 3 CONFIGURATION (FRONT SENSOR):
	//			Echo - D5
	//			Trigger - D3 

distSensor::distSensor()
{
	distSensor(13,2, 9,11, 3, 5);
}

//Non-default Constructor 
distSensor::distSensor(int trigger_pin_1, int echo_pin_1, int trigger_pin_2, int echo_pin_2, int trigger_pin_3, int echo_pin_3){
	 NewPing RightSensor(trigger_pin_1, echo_pin_1);
	 rightSensor = RightSensor;
	 NewPing FrontSensor(trigger_pin_2, echo_pin_2);
	 frontSensor = FrontSensor;
	 NewPing FrontRightSensor(trigger_pin_3, echo_pin_3);
	 frontRightSensor = FrontRightSensor;
}

// Non-Default Constructor
void distSensor::init() {
	init(13,2,9,11,3,5);
}

// Non-Default Constructor
void distSensor::initDistSensors(NewPing RightSensor, NewPing FrontSensor, NewPing FrontRightSensor) {
	rightSensor = RightSensor;
	frontSensor = FrontSensor;
	frontRightSensor = FrontRightSensor;
}
// Non-Default Constructor
void distSensor::init(int trigger_pin_1, int echo_pin_1, int trigger_pin_2, int echo_pin_2, int trigger_pin_3, int echo_pin_3) {
	NewPing rightSensor(trigger_pin_1, echo_pin_1);
	NewPing frontSensor(trigger_pin_2, echo_pin_2);
	NewPing frontRightSensor(trigger_pin_3, echo_pin_3);
}

// Inputs: Number of Data points to take, distance sensor. For WhichSesnor: 0 - right sensor, 1 - Front right, 2- Right
// Outputs: Median duration time
int distSensor::read_duration(int iterations, uint8_t whichSensor) {
	if (whichSensor == 0){
		rightDuration = rightSensor.ping_median(iterations);
		return rightDuration;
	}
	else if(whichSensor == 1) {
		frontRightDuration = frontRightSensor.ping_median(iterations);
		return frontRightDuration;
	}
	else {
		frontDuration = frontSensor.ping_median(iterations);
		return frontDuration;
	}
}

//Inputs: Duration of time it took ultrasonic wave to come back, For WhichSesnor: 0 - right sensor, 1 - Front right, 2- Right
//Outputs: Distance in inches from one sensor
float distSensor::output_distance(uint8_t whichSensor) {
	float soundcm = 331.4 / 10000;
	if (whichSensor == 0) {
		rightDistance = ((float(rightDuration) / 2)* soundcm) / 2.54;
		return rightDistance;
	}
	else if (whichSensor == 1) {
		frontRightDistance = ((float(frontRightDuration) / 2)* soundcm) / 2.54;
		return frontRightDistance;
	}
	else {
		frontDistance = ((float(frontDuration) / 2)* soundcm) / 2.54;
		return frontDistance;
	}

}
