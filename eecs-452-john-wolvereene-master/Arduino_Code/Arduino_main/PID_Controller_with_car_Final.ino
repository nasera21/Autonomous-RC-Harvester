#include <DistanceSensorLib.h>
#include <RCCarLib.h>
#include <NewPing.h>
#include "carSerial.h"

//Toggle CV System On/off 
bool cv_system = true; 
int incomingByte; 
float pi_dist;
carSerial ser(9600);
//RC Car 
RC_Car myCar(4);


//Distance Sensor 
//Right Distance Sensor 
#define TRIGGER_PIN 13
#define ECHO_PIN 2

//Front Right Distance Sensor 
#define TRIGGER_PIN_2 9
#define ECHO_PIN_2 11

//Front Distance Sensor 
#define ECHO_PIN3 5
#define TRIGGER_PIN3 3

//Distance sensor Variables and Inits
float rightDist = 0; 
float frontRightDist = 0;
float frontDist = 0;
int rightDuration = 0;
int frontRightDuration = 0;
int frontDuration = 0;
distSensor mySensors; 

//PID Gains 
float Kp = 3; 
float Ki = 0.001;  
float Kd = 10;

//PID and Turn Angle Variables 
int prop;
int inte;
int deriv;
int last_prop; 
int servo_turn; //Turn angle Variable 
int a_position; //Position Variable 
int set_point; 
int error;//PID Output 

//Inner and outer Set Points 
int set_point_I; 
int set_point_O;


//Distance sensor parameters
NewPing rightSensor(TRIGGER_PIN,ECHO_PIN);
NewPing frontRightSensor(TRIGGER_PIN_2,ECHO_PIN_2);
NewPing frontSensor(TRIGGER_PIN3, ECHO_PIN3);

void setup() {
 //Car Initialization 
 myCar.begin(10);
 Serial.begin(9600);
 //Distance Sensor Initialization
 mySensors.initDistSensors(rightSensor, frontSensor, frontRightSensor);
 //UART GPIO Pin Initialization and delay
 pinMode(A0, OUTPUT);
delay(1000); 
//Car Initial Boost 
 myCar.RC_Car_Command(.5,0);
 delay(500);
}

void loop() {
    digitalWrite(A0, HIGH);
    //Rapberry Pi Distance Readings VIA UART 
    if (Serial.available() > 0) {
      // read the incoming byte:
      digitalWrite(A0, LOW);
      incomingByte = ser.check_format();
      
       //For Debugging on the Rpi End Uncomment
       /*
       if(incomingByte!= '\xff'){
        //covert hex string to int can't just cast it
        Serial.write(incomingByte);
        pi_dist = back2Float(int(incomingByte)); 
       }
       */
    }

    //Distance Sensor Readings
    rightDuration = mySensors.read_duration(5,0);
    frontRightDuration = mySensors.read_duration(5,1);
    frontDuration = mySensors.read_duration(5,2);
    rightDist = mySensors.output_distance(0);
    frontRightDist = mySensors.output_distance(1);
    frontDist = mySensors.output_distance(2);

    //If there is an obstacle in front, stop 
    if(frontDist < 20 && frontDist != 0){
        myCar.RC_Car_Command(0.2,0);
        myCar.RC_Car_Command(0,0);
        delay(5000); 
    }
    
     //If CV System is Toggled on we get our position from there! 
     if(cv_system == true){ 
      a_position = pi_dist; 
    }
    else{ //We get our position from Distance sensors 
      distance_sensors();
    }
   
    set_point = 12; //Car must be 12 inches from the boundary. 
    set_point_I = 10; //Inner Set Point if we want a range
    set_point_O = 12; //Outer Set Point
    pid_loop(); //Calculates the error from the position

    //If position is within a range don't turn 
    if(a_position < set_point_O && a_position > set_point_I){
      myCar.RC_Car_Command(.4,0);
    }
    turn_angle(); //Calculate turn angle from error 
    myCar.RC_Car_Command(.4,servo_turn); //Send Turn angle to the RC Car
    
}


void distance_sensors(){
  //Error Checking,for when distance sensors are out of range
  if(frontRightDist==0){
        a_position = rightDist; 
      }
      else if(rightDist == 0){
        a_position = frontRightDist; 
      }

      else if((rightDist == 0) && (frontRightDist ==0)){
        a_position = 10; 
      }
      else{
      a_position = min(frontRightDist,rightDist);
      }
    }

void pid_loop(){ 
  prop = a_position-set_point; 
  inte = inte + prop;
  deriv = prop - last_prop;
  last_prop= prop;
  error = int(prop * Kp + inte * Ki + deriv * Kd);
}


void turn_angle(){
//We want to constrain the error between (-128)- (+127) 
//because that corresponds to the max output of the Arduino 8-Bit DAC 
if ((error< -128) && (error < 0)){
  error = -128;
 }  

if (error > 127){
  error = 127;
 }
 
 //Scales the error value into a turn angle. More error = more turning. 
 servo_turn = int(error* 15.0/127.0); 
 }


 //UART Reverse Quantization 
float back2Float(int value){
  float converted_value = value/pow(2,3);
  return converted_value;
}


 
