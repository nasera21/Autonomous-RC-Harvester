/*
 *  WRITTEN FOR: EECS 452 MDE Project Winter 2021
 *              Team: John Wolvereene
 *              Authors: Elisabeth Jahnke
 *
 */


#ifndef DISTANCE_SENSOR_h
#define DISTANCE_SENSOR_h

#include "NewPing.h"
#include "Arduino.h"


class distSensor
{

public:
	int iterations = 5;
	NewPing rightSensor{ 2,13 };
	NewPing frontRightSensor{ 11,9 };
	NewPing frontSensor{3,5};
	uint8_t whichSensor = 0;


	//Default Constructor -- sets everything to Default Configureation
	// Default Configuration:
	// DISTANCE SENSOR 1 CONFIGURATION (RIGHT SENSOR):
	//          Echo - D2
	//          Trigger - D13 
	// DISTANCE SENSOR 2 CONFIGURATION (FRONT RIGHT SENSOR):
	//          Echo - D11 (Check that these pins can be used)
	//          Trigger - D9 (Check that these pins can be used) 
	//DISTANCE SENSOR 3 CONFIGURATION (FRONT SENSOR):
	//			Echo - D5
	//			Trigger - D3 


	distSensor();


	//Non-Default Constructors
	distSensor(int trigger_pin_1, int echo_pin_1, int trigger_pin_2, int echo_pin_2, int trigger_pin_3, int echo_pin_3);

	void init();

	void init(int trigger_pin_1, int echo_pin_1, int trigger_pin_2, int echo_pin_2, int trigger_pin_3, int echo_pin_3);

	void initDistSensors(NewPing RightSensor, NewPing FrontSensor, NewPing FrontRightSensor);

	// Inputs: Number of iterations in data points to take then outputs it
	// uint8 value determines which sensor is used; 0 - right sensor, 1 - front right sensor, 2 - front sensor
	// Outputs: Median duration time
	int read_duration(int iterations, uint8_t whichSensor);

	//Inputs: Duration of time it took ultrasonic wave to come back, bool determines which sensor is measuring
	// uint8 value determines which sensor is used; 0 - right sensor, 1 - front right sensor, 2 - front sensor
	//Outputs: Distance in inches from one sensor
	float output_distance(uint8_t whichSensor);

private:
	// Pins, distances and Durations are private variables
	int echo_pin_1;
	int trigger_pin_1;
	int echo_pin_2;
	int trigger_pin_2;
	int echo_pin_3;
	int trigger_pin_3;
	float rightDistance;
	float frontRightDistance;
	float frontDistance;
	int rightDuration;
	int frontRightDuration;
	int frontDuration;
};

#endif
