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

        // Cartesian space indices

        // Deconstructor
        ~LynxMotion();
    
    private:


        // Servo objects
        Servo m_servoJoint1; Servo m_servoJoint2; Servo m_servoJoint3;
        Servo m_servoJoint4; Servo m_servoJoint5; Servo m_servoGripper;
};



#endif