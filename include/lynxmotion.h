#ifndef LYNXMOTION_H
#define LYNXMOTION_H


#include <Arduino.h>
#include <Servo.h>


#define joint1ServoPin 3 
#define joint2ServoPin 5
#define joint3ServoPin 6
#define joint4ServoPin 9
#define joint5ServoPin 10
#define gripperServoPin 11

#define servoNum 6

class LynxMotion
{
    public:

        // Constructor
        LynxMotion();

        // Initialize servos
        void init(void);
        
        // Forward kinematics
        void forwardKinematics(int *jointVec);

        // Joint space indices
       struct jointSpace
        {   int joint1; int joint2; int joint3;
            int joint4; int joint5; int gripper;
        };
        const struct jointSpace jointIndices = {0, 1, 2, 3, 4, 5};

        // Joint angles limits
        const int lowerJointLimits[servoNum] = {-90, 0, -120, -120, -90, 0};
        const int upperJointLimits[servoNum] = {90, 120, 0, 0, 90, 0};

        // Deconstructor
        ~LynxMotion();
    
    private:

        // Servo objects
        Servo m_servoJoint1; Servo m_servoJoint2; Servo m_servoJoint3;
        Servo m_servoJoint4; Servo m_servoJoint5; Servo m_servoGripper;

        /*************** Joint to servo mapping **************/
        // Servo limits 
        const int m_lowerServoLimit = 0;
        const int m_upperServoLimit = 180;

        // Joint limits
        const int m_lowerJointLimits[servoNum] = {-90, 0, -180, -180, -90, 0};
        const int m_upperJointLimits[servoNum] = {90, 180, 0, 0, 90, 0};
};



#endif