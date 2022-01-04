/*
*   WRITTEN FOR: EECS 452 MDE Project Winter 2021
*         Team : John Wolvereene
*      Authors : Adithya Subbiah
*
*/


#include "RCCarLib.h"




    
    // Default Constructor -- sets everything to Default Configuration
    // Default Configuration:
    //      Servo - Pin 10
    //      Motor - Motor 4
    //      
    RC_Car::RC_Car()
    {
        RC_Car(4);
    }


    // Non default Contstructor
    RC_Car::RC_Car(int MotorNum)
    {
        AFMS = Adafruit_MotorShield();
        Car_Motor = AFMS.getMotor(MotorNum);

    }

    // If Arduino isn't playing nice, you can use these instead
    void RC_Car::init()
    {
        init(4);
    }

    void RC_Car::init(int MotorNum)
    {
        AFMS = Adafruit_MotorShield();
        Car_Motor = AFMS.getMotor(MotorNum);

    }

    void RC_Car::begin(int ServoPin)
    {
        // Servo Setup and set to Straight
        Car_Servo.attach(ServoPin);
        Car_Servo.write(degrees_Convert(0));
        delay(1000);
        AFMS.begin();

        // Initialize Motor in Safe State
        Car_Motor->run(FORWARD);
        Car_Motor->setSpeed(150);
        //delay(5000);
        
        Car_Motor->run(RELEASE);
    }

    // Full RC Car Command, set Speed of drive and steering degrees
    // Speed < 0 : Backwards
    // Speed > 0 : Forwards
    // -1.0 < Speed < 1.0
    // -30 < SteeringDegrees < 30
    void RC_Car::RC_Car_Command(float Speed, int SteeringDegrees)
    {
        if(param_Check(Speed, SteeringDegrees) == 0)
        {
            Car_Servo.write(degrees_Convert(SteeringDegrees));
            if(Speed < 0)
            {
                Car_Motor->run(BACKWARD);
                Car_Motor->setSpeed(255*Speed);
            }
            else
            {
                Car_Motor->run(FORWARD);
                Car_Motor->setSpeed(255*Speed);
            }
            curr_Speed = Speed;
            curr_Steering = SteeringDegrees;
        }
    }

    // Set only Steering
    void RC_Car::RC_Car_Steering(int SteeringDegrees)
    {
        if(param_Check(0, SteeringDegrees) == 0)
        {
            Car_Servo.write(degrees_Convert(SteeringDegrees));
            curr_Steering = SteeringDegrees;
        }
    }

    // Set only Speed
    void RC_Car::RC_Car_Speed(float Speed)
    {
        if(param_Check(Speed, 0) == 0)
        {
            if(Speed < 0)
            {
                Car_Motor->run(BACKWARD);
                Car_Motor->setSpeed((255)*Speed);
            }
            else
            {
                Car_Motor->run(FORWARD);
                Car_Motor->setSpeed((255)*Speed);
            }
            curr_Speed = Speed;
        }
    }

    // Check Speed and Steering Degrees parameters
    int RC_Car::param_Check(float Speed, int SteeringDegrees)
    {
        if(Speed < -1 || Speed > 1)
        {
            return 1;
        }

        if(SteeringDegrees < -30 || SteeringDegrees > 30)
        {
            return 1;
        }

        return 0;
    }

    // Convert SteeringDegress to Servo command
    // -30 < SteeringDegrees < 30 ---> 60 < ServoCommand < 120
    int RC_Car::degrees_Convert(int SteeringDegrees)
    {
        return SteeringDegrees + 90;
    }
