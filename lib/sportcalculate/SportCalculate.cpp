#include "SportCalculate.h"


/*正运动学解算*/
void ForwardKinematics(FSUS_SERVO_ANGLE_T hipAngle, FSUS_SERVO_ANGLE_T kneeAngle, FSUS_SERVO_ANGLE_T ankleAngle, float &x, float &y, float &z)
{
    // 机械臂的三个关节的长度
    float L1 = 0.1;
    float L2 = 0.1;
    float L3 = 0.1;

    // 弧度制
    float hipAngleRad = hipAngle * PI / 180;
    float kneeAngleRad = kneeAngle * PI / 180;
    float ankleAngleRad = ankleAngle * PI / 180;

    // 机械臂的正运动学解算
    x = L1 * cos(hipAngleRad) + L2 * cos(kneeAngleRad) * cos(hipAngleRad) + L3 * cos(kneeAngleRad + ankleAngleRad) * cos(hipAngleRad);
    y = L1 * sin(hipAngleRad) + L2 * sin(kneeAngleRad) * sin(hipAngleRad) + L3 * sin(kneeAngleRad + ankleAngleRad) * sin(hipAngleRad);
    z = -L3 * sin(kneeAngleRad + ankleAngleRad) - L2 * sin(kneeAngleRad);
}
/*运动学逆解*/
void InverseKinematics(float x, float y, float z, FSUS_SERVO_ANGLE_T &hipAngle, FSUS_SERVO_ANGLE_T &kneeAngle, FSUS_SERVO_ANGLE_T &ankleAngle)
{
    // 机械臂的三个关节的长度
    float L1 = 0.1;
    float L2 = 0.1;
    float L3 = 0.1;

    // 机械臂的逆运动学解算
    hipAngle = atan2(y, x) * 180 / PI;
    float f1 = x * cos(hipAngle) +y * sin(hipAngle);
    float f2 = z;
    float temp = f1 - L1;
    kneeAngle = atan2(temp,f2) - asin(pow(temp, 2) + pow(f1, 2) - pow(L3, 2) - pow(L2, 2)) / (2 * L2 * sqrt(pow((f1-L2), 2) + pow(f2, 2))) * 180 / PI;
    ankleAngle = acos((pow(temp, 2) + pow(f2, 2) - pow(L3, 2) - pow(L2, 2))/(2 * L2 * L3)) * 180 / PI;
}
