#ifdef _SPORTCALCULATE_H_
#define _SPORTCALCULATE_H_

#include <Arduino.h>

class SportCalculate
{
public:
    SportCalculate();
    ~SportCalculate();
    void ForwardKinematics(FSUS_SERVO_ANGLE_T hipAngle, FSUS_SERVO_ANGLE_T kneeAngle, FSUS_SERVO_ANGLE_T ankleAngle, float &x, float &y, float &z); // 正运动学解算
    void InverseKinematics(float x, float y, float z, FSUS_SERVO_ANGLE_T &hipAngle, FSUS_SERVO_ANGLE_T &kneeAngle, FSUS_SERVO_ANGLE_T &ankleAngle); // 逆运动学逆解
};

#endif // DEBUG