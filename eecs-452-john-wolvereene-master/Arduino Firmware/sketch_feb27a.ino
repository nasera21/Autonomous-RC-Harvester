// Include the Servo library 
#include <Servo.h> 
// Declare the Servo pin 
int servoPin = 10; 
// Create a servo object 
Servo Servo1; 
void setup() { 
   // We need to attach the servo to the used pin number
   Serial.begin(9600); 
   Servo1.attach(servoPin);
   Servo1.write(90);
   delay(1000); 
}
void loop(){ 
  if (Serial.available() > 0) 
  {
      int input = Serial.read();
     if(input == 'L'){
      Servo1.write(60);
      Serial.println("Left");
     }
     else if(input == 'R'){
      Servo1.write(120);
      Serial.println("Right");
     }
     else if(input == 'S'){
      Servo1.write(90);
      Serial.println("Straight");
     }
  }
}



// 60 degrees = Left
// 90 degrees = straight
// 120 degrees = right






//// Include the Servo library 
//#include <Servo.h> 
//// Declare the Servo pin 
//int servoPin = 10; 
//// Create a servo object 
//Servo Servo1; 
//void setup() { 
//   // We need to attach the servo to the used pin number 
//   Servo1.attach(servoPin); 
//   Serial.begin(9600);
//}
//void loop(){ 
//   // Make servo go to 0 degrees 
//  int angle = Servo1.read();
//  Serial.println(angle);
//  Serial.println('\n');
//
//   if (Serial.available() > 0) 
//   {
//      int input = Serial.read();
//      Servo1.write(int(input));
//   }
//}
