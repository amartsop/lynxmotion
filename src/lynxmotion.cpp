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


void LynxMotion::inverseKinematics(BLA::Matrix<3, 1> dFeF0, BLA::Matrix<3, 3> RFeF0)
{

    BLA::Matrix<3, 1> rd = {0.0, 0.0, m_L4 + m_Le};
    BLA::Matrix<3, 1> dF3F0 = dFeF0 - RFeF0 * rd;

    float px3 = dF3F0(0, 0); float py3 = dF3F0(1, 0); float pz3 = dF3F0(2, 0);

    // Joint angle 1
    float theta1 = atan2(py3, px3);

    // Joint angle 3
    float k1 = powf(pz3 - m_L1, 2.0f) + powf(px3, 2.0f) + 
        powf(py3, 2.0f) - powf(m_L2, 2.0f) - powf(m_L3, 2.0);
    float ct3 = k1 / (2.0 * m_L2 * m_L3); 
    float st3 = m_t3S * sqrtf(1 - powf(ct3, 2.0f));
    float theta3 = atan2(st3, ct3);

    // Joint angle 2
    float phi1 = atan2(-m_L3 * sinf(theta3), m_L2 + m_L3 * cosf(theta3));
    float rho = sqrtf(powf(m_L2 + m_L3 * cosf(theta3), 2.0f) + 
        powf(m_L3 * sinf(theta3), 2.0f));
    float st2Phi1 = (pz3 - m_L1) / rho;
    float ct2Phi1 = m_t2Phi1S * sqrtf(1.0f - powf(st2Phi1, 2.0f));
    float theta2 = phi1 + atan2(st2Phi1, ct2Phi1);

    // R_f3_f0
    BLA::Matrix<3, 3> RF3F0 = {cosf(theta1) * cosf(theta2 + theta3), 
        -cosf(theta1) * sinf(theta2 + theta3), sinf(theta1), sinf(theta1) * 
        cosf(theta2 + theta3), -sinf(theta1) * sinf(theta2 + theta3), 
        -cosf(theta1), sinf(theta2 + theta3), cosf(theta2 + theta3), 0.0f};

    // Joint angles theta4 and theta5
    BLA::Matrix<3, 3> RFeF3 = (~RF3F0) * RFeF0;
    float theta4 = atan2(RFeF3(1, 2), RFeF3(0, 2));
    float theta5 = atan2(RFeF3(2, 0), RFeF3(2, 1));
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