
#include <RCCarLib.h>

RC_Car myCar(4);

void setup() {
  // put your setup code here, to run once:
myCar.begin(10);

}

void loop() {
  // put your main code here, to run repeatedly:
  myCar.RC_Car_Command(.75, 25);
  delay(5000);
  myCar.RC_Car_Steering(0);
  delay(5000);
  myCar.RC_Car_Speed(.50);
  delay(5000);
}
