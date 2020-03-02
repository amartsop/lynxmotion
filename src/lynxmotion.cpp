#include "../include/lynxmotion.h"
#include "../include/signal_processing.hpp"


LynxMotion::LynxMotion()
{

}


void LynxMotion::init(void)
{
    m_servoJoint1.attach(joint1ServoPin); m_servoJoint2.attach(joint2ServoPin);
    m_servoJoint3.attach(joint3ServoPin); m_servoJoint4.attach(joint4ServoPin);
    m_servoJoint5.attach(joint5ServoPin); m_servoGripper.attach(gripperServoPin);
}



void LynxMotion::forwardKinematics(int *jointVec)
{  
    // Clipping
    jointVec[jointIndices.joint1] = 
        SignalProcessing<int>::clipping(jointVec[jointIndices.joint1], 
            lowerJointLimits[jointIndices.joint1], 
            upperJointLimits[jointIndices.joint1]);

    jointVec[jointIndices.joint2] = 
        SignalProcessing<int>::clipping(jointVec[jointIndices.joint2], 
            lowerJointLimits[jointIndices.joint2], 
            upperJointLimits[jointIndices.joint2]);

    jointVec[jointIndices.joint3] = 
        SignalProcessing<int>::clipping(jointVec[jointIndices.joint3], 
            lowerJointLimits[jointIndices.joint3], 
            upperJointLimits[jointIndices.joint3]);

    jointVec[jointIndices.joint4] = 
        SignalProcessing<int>::clipping(jointVec[jointIndices.joint4], 
            lowerJointLimits[jointIndices.joint4], 
            upperJointLimits[jointIndices.joint4]);

    jointVec[jointIndices.joint5] = 
        SignalProcessing<int>::clipping(jointVec[jointIndices.joint5], 
            lowerJointLimits[jointIndices.joint5], 
            upperJointLimits[jointIndices.joint5]);

    // Mapping
    int servoJoint1 = SignalProcessing<int>::map(jointVec[jointIndices.joint1], 
        m_upperJointLimits[jointIndices.joint1], 
        m_lowerJointLimits[jointIndices.joint1], 
        m_lowerServoLimit, m_upperServoLimit);

    int servoJoint2 = SignalProcessing<int>::map(jointVec[jointIndices.joint2], 
        m_lowerJointLimits[jointIndices.joint2], 
        m_upperJointLimits[jointIndices.joint2], 
        m_lowerServoLimit, m_upperServoLimit);

    int servoJoint3 = SignalProcessing<int>::map(jointVec[jointIndices.joint3], 
        m_upperJointLimits[jointIndices.joint3], 
        m_lowerJointLimits[jointIndices.joint3], 
        m_lowerServoLimit, m_upperServoLimit);


    int servoJoint4 = SignalProcessing<int>::map(jointVec[jointIndices.joint4], 
        m_lowerJointLimits[jointIndices.joint4],
        m_upperJointLimits[jointIndices.joint4],
        m_lowerServoLimit, m_upperServoLimit);


   int servoJoint5 = SignalProcessing<int>::map(jointVec[jointIndices.joint5], 
        m_lowerJointLimits[jointIndices.joint5],
        m_upperJointLimits[jointIndices.joint5], 
        m_lowerServoLimit, m_upperServoLimit);


    m_servoJoint1.write(servoJoint1); 
    m_servoJoint2.write(servoJoint2);
    m_servoJoint3.write(servoJoint3); 
    m_servoJoint4.write(servoJoint4);
    m_servoJoint5.write(servoJoint5); 
    m_servoGripper.write(jointVec[jointIndices.gripper]);
}


LynxMotion::~LynxMotion()
{

}