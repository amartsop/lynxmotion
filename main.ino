
#include "Arduino.h"
#include "./include/lynxmotion.h"
#include "BasicLinearAlgebra.h"

#define servoNum 6

// Robot instance
LynxMotion robot;

// Joint space vector
int jointVec[servoNum] = {0};


void setup()
{
    robot.init();
    Serial.begin(9600);
}


void loop()
{
    /******************** Inverse Kinematics ******************/
    // Joystick
    
    // Desired position
    BLA::Matrix<3, 1> dFeF0 = {0.05, -0.1, 0.1};

    BLA::Matrix<3, 3> RFeF0{1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, -1.0};

    robot.inverseKinematics(dFeF0, RFeF0);



    
}



