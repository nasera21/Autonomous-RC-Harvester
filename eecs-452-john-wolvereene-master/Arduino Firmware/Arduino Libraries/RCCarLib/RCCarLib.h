/*
*   WRITTEN FOR: EECS 452 MDE Project Winter 2021
*         Team : John Wolvereene
*      Authors : Adithya Subbiah
*
*/



#ifndef	RC_CAR_LIB_H
#define RC_CAR_LIB_H


#include "Arduino.h"
#include "Servo.h"
#include "Wire.h"
#include "Adafruit_MotorShield.h"



class RC_Car
{
    public:
    Servo Car_Servo;
    Adafruit_MotorShield AFMS;
    Adafruit_DCMotor *Car_Motor;
    int curr_Speed = 0;
    int curr_Steering = 0;
    
    // Default Constructor -- sets everything to Default Configuration
    // Default Configuration:
    //      Servo - Pin 10
    //      Motor - Motor 4
    //      Baudrate - 9600
    //      
    RC_Car();


    // Non default Contstructor
    RC_Car(int MotorNum);

    // If Arduino isn't playing nice, you can use these instead
    void init();

    void begin(int ServoPin);

    void init(int MotorNum);

    // Full RC Car Command, set Speed of drive and steering degrees
    // Speed < 0 : Backwards
    // Speed > 0 : Forwards
    // -1.0 < Speed < 1.0
    // -30 < SteeringDegrees < 30
    void RC_Car_Command(float Speed, int SteeringDegrees);

    // Set only Steering
    void RC_Car_Steering(int SteeringDegrees);

    // Set only Speed
    void RC_Car_Speed(float Speed);

    private:
    // Check Speed and SteeringDegrees parameters
    int param_Check(float Speed, int SteeringDegrees);

    // Convert SteeringDegress to Servo command
    // -30 < SteeringDegrees < 30 ---> 60 < ServoCommand < 120
    int degrees_Convert(int SteeringDegrees);
};
#endif
