#include "Arduino.h"
#include "./include/lynxmotion.h"

// #define servoNum 6

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
    /******************** Forward Kinematics ******************/


    //Joystick

    jointVec[robot.jointIndices.joint1] = 0;
    jointVec[robot.jointIndices.joint2] = 80;
    jointVec[robot.jointIndices.joint3] = 0;
    jointVec[robot.jointIndices.joint4] = 0;
    jointVec[robot.jointIndices.joint5] = 90;


    // Forward Kinematics
    robot.forwardKinematics(jointVec);




    delay(50);


    // Serial.println(angle);

    

}



