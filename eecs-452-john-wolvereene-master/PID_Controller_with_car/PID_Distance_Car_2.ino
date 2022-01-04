#include <DistanceSensorLib.h>
#include <RCCarLib.h>
#include <NewPing.h>
//RC Car 
RC_Car myCar(4);

//Distance Sensor 
#define TRIGGER_PIN 13
#define ECHO_PIN 2
#define TRIGGER_PIN_2 9
#define ECHO_PIN_2 11
#define ECHO_PIN3 5
#define TRIGGER_PIN3 3
float rightDist = 0; 
float frontRightDist = 0;
float frontDist = 0;
int rightDuration = 0;
int frontRightDuration = 0;
int frontDuration = 0;
distSensor mySensors; 

//PID Gains (TODO: Tune)
float Kp = 5;   //30 //11
float Ki = 0.001;  //0.003; //0.001
float Kd = 7;  //8; //10

int proportional;
int integral;
int derivative;
int last_proportional; 
int servo_turn; 
int a_position; 
int set_point; 
int error_value;
int set_point_I; 
int set_point_O;


//Distance sensor parameters
NewPing rightSensor(TRIGGER_PIN,ECHO_PIN);
NewPing frontRightSensor(TRIGGER_PIN_2,ECHO_PIN_2);
NewPing frontSensor(TRIGGER_PIN3, ECHO_PIN3);

void setup() {
  // put your setup code here, to run once:
 myCar.begin(10);
 Serial.begin(9600);
 mySensors.initDistSensors(rightSensor, frontSensor, frontRightSensor);
 //myCar.RC_Car_Command(.40,0);
 delay(500);
}

void loop() {
    //Distance Sensor 
    rightDuration = mySensors.read_duration(5,0);
    frontRightDuration = mySensors.read_duration(5,1);
    frontDuration = mySensors.read_duration(5,2);
    rightDist = mySensors.output_distance(0);
    frontRightDist = mySensors.output_distance(1);
    frontDist = mySensors.output_distance(2);
    Serial.print("Right Dist:");
    Serial.print(rightDist);
    Serial.println(" ");
    Serial.print("Front Right Dist:");
    Serial.print(frontRightDist);
    Serial.println(" ");
    Serial.print("Front Dist:");
    Serial.print(frontDist);
    Serial.println(" ");
    
//Actual Distance in the Camera Field of vision the Asparagus is 
    a_position = min(frontRightDist,rightDist); //REPLACE (With Distance Sensor & Camera Code)
    set_point = 12; //Car must be 5 inches from the boundary. 
    set_point_I = 10; //Inner Set Point if we want a range
    set_point_O = 12; //Outer Set Point

    /* edge cases:
    if(a_position > 30){ //Too far out
      myCar.RC_Car_Command(0.3,15);
    }

    if(){ //Too close in and facing the wall (see discrepancy betweenn short camera distance and large sensor distance for starting pos?)
      myCar.RC_Car_Command(0.3,-30);
    }
    */
    
    pid_calc(); 

    //Add this line, to say if the error is really small, the robot is correctly
    //following the line, no need to turn. 
    //if(front_dist < 10){
      // myCar.RC_Car_Command(0,0);
      // delay(5000);
    //}
    /*
    if(a_position < set_point_O && a_position > set_point_I){
      myCar.RC_Car_Command(.3,0);
    }*/
    calc_turn(); 
    myCar.RC_Car_Steering(servo_turn);
    delay(500); 
    /*myCar.RC_Car_Speed(.50);
    myCar.RC_Car_Command(.3,servo_turn);*/
    //delay(500);
} 



void pid_calc(){ 
  proportional = a_position-set_point; 
  integral = integral + proportional;
  derivative = proportional - last_proportional;
  last_proportional = proportional;
  error_value = int(proportional * Kp + integral * Ki + derivative * Kd);
  //Serial.println(error_value);
}


//We want to constrain the error between (-128)- (+127) because that corresponds to the max output of the Arduino 8-Bit DAC 

void calc_turn(){
if ((error_value< -128) && (error_value < 0)){
  error_value = -128;
 }  

if (error_value > 127){
  error_value = 127;
 }
 //Scales the error value into a turn angle. More error = more turning. 
 servo_turn = int(error_value* 30.0/127.0); 
 Serial.println(servo_turn); 
 }
 
