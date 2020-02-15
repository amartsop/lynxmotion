#include "../include/lynxmotion.h"


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
    m_servoJoint1.write(jointVec[jointIndices.joint1]); 
    m_servoJoint2.write(jointVec[jointIndices.joint2]);
    m_servoJoint3.write(jointVec[jointIndices.joint3]); 
    m_servoJoint4.write(jointVec[jointIndices.joint4]);
    m_servoJoint5.write(jointVec[jointIndices.joint5]); 
    m_servoGripper.write(jointVec[jointIndices.gripper]);
}


LynxMotion::~LynxMotion()
{

}