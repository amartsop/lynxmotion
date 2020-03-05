#include "Arduino.h"
#include "./include/lynxmotion.h"

#define joint1PotPin A0
#define joint2PotPin A1
#define joint3PotPin A2
#define joint4PotPin A3
#define joint5PotPin A4

const int m_upperPotVal = 1023;
const int m_lowerPotVal = 0;

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
    jointVec[robot.jointIndices.joint1] = 
        map(analogRead(joint1PotPin), m_lowerPotVal, m_upperPotVal,
        robot.upperJointLimits[robot.jointIndices.joint1], 
        robot.lowerJointLimits[robot.jointIndices.joint1]);

    jointVec[robot.jointIndices.joint2] = 80;
        // map(analogRead(joint2PotPin), m_lowerPotVal, m_upperPotVal,
        // robot.upperJointLimits[robot.jointIndices.joint2], 
        // robot.lowerJointLimits[robot.jointIndices.joint2]);

    jointVec[robot.jointIndices.joint3] = 
        map(analogRead(joint3PotPin), m_lowerPotVal, m_upperPotVal,
        robot.upperJointLimits[robot.jointIndices.joint3], 
        robot.lowerJointLimits[robot.jointIndices.joint3]); 

    jointVec[robot.jointIndices.joint4] = 0;
    jointVec[robot.jointIndices.joint5] = 90;    

    // Forward Kinematics
    robot.forwardKinematics(jointVec);




    // delay(10);


    // Serial.println(angle);

    

}



