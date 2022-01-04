

#include <DistanceSensorLib.h>
#include <NewPing.h>
#define TRIGGER_PIN_1 13
#define ECHO_PIN_1 2
#define TRIGGER_PIN_2 9
#define ECHO_PIN_2 11
#define TRIGGER_PIN3 3
#define ECHO_PIN3 5
int duration1;
int duration2;
int duration3;
float  distance1;
float distance2;
float distance3;

distSensor mySensors;
NewPing RightSensor(TRIGGER_PIN_1, ECHO_PIN_1);
NewPing FrontSensor(TRIGGER_PIN_2, ECHO_PIN_2);
NewPing frontRightSensor(TRIGGER_PIN3, ECHO_PIN3);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
 mySensors.initDistSensors(RightSensor, FrontSensor, frontRightSensor);
 
}

void loop() {
  // put your main code here, to run repeatedly:
  //duration = LeftSensor.ping_median(5);
  //Serial.print(duration);
  // Measure Duration of Pulses for Distance Sensors
  delay(1000);
  duration1 = mySensors.read_duration(5, 0);
  duration2 = mySensors.read_duration(5, 1);
  duration3 = mySensors.read_duration(5,2);

  // Calculate Distance
  distance1 = mySensors.output_distance(0);
  distance2 = mySensors.output_distance(1); 
  distance3 = mySensors.output_distance(2);
  
  // Send to Serial Port
  
  Serial.print("Right Sensor: ");
  Serial.print("Duration: ");
  Serial.print(duration1);
  Serial.print("Distance: ");
  Serial.print(distance1);
  Serial.println(" ");
  
  Serial.print("Front Right Sensor: ");
  Serial.print("Duration: ");
  Serial.print(duration2);
  Serial.print("Distance: ");
  Serial.print(distance2);
  Serial.println(" ");
  
  Serial.print("Front Sensor: ");
  Serial.print("Duration: ");
  Serial.print(duration3);
  Serial.print("Distance: ");
  Serial.print(distance3);
  Serial.println(" ");
  
  
}
