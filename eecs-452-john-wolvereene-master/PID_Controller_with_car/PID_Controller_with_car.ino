
#include <RCCarLib.h>

//RC Car 
RC_Car myCar(4);

//PID Gains (TODO: Tune)
float Kp = 30;
float Ki = 0.003;
float Kd = 8;

int proportional;
int integral;
int derivative;
int last_proportional; 
int servo_turn; 
int a_position; 
int set_point; 
int error_value;


void setup() {
  // put your setup code here, to run once:
 myCar.begin(10);

}

void loop() {
  if (Serial.available() > 0) {
  //Actual Distance in the Camera Field of vision the Asparagus is 
    a_position = Serial.parseInt(); //REPLACE (With Distance Sensor & Camera Code)
    set_point = 5; //Car must be 5 inches from the boundary. 
    pid_calc(); 
    calc_turn(); 
    myCar.RC_Car_Steering(servo_turn);
    delay(5000);
    myCar.RC_Car_Speed(.40);
    delay(5000);
    
  }
} 



void pid_calc(){ 
  proportional = a_position-set_point; 
  integral = integral + proportional;
  derivative = proportional - last_proportional;
  last_proportional = proportional;
  error_value = int(proportional * Kp + integral * Ki + derivative * Kd);
  Serial.println(error_value);
}


void calc_turn(){
if ((error_value< -256) && (error_value < 0)){
  error_value = -256;
 }  

if (error_value > 256){
  error_value = 256;
 }
 Serial.println("error Value"); 
 Serial.println(error_value); 
 //Scales the error value into a turn 
 servo_turn = int(error_value* 30.0/256.0); 
  Serial.print("Servo Turn "); 
  Serial.println(servo_turn);
 }
