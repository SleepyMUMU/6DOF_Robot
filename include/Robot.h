#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <Arduino.h>
#include <Servo.h>
#include <FashionStar_UartServo.h>
#include <FashionStar_UartServoProtocol.h>
#define RobotInitPosTime 1000
#define defaultpos1 0
#define defaultpos2 1
extern float defaultPosition[11][3];

class Robot
{
public:
    Robot(LegConfig leg1, LegConfig leg2, LegConfig leg3, LegConfig leg4, LegConfig leg5, LegConfig leg6);
    LegConfig Leg[6];

    Robot(FSUS_Protocol INputPol1, FSUS_Protocol INputPol2);

    void SetPos(float Leg1Position[3], float Leg2Position[3], float Leg3Position[3], float Leg4Position[3], float Leg5Position[3], float Leg6Position[3]);
    void SetPos(float Leg1Position[3], float Leg2Position[3], float Leg3Position[3], float Leg4Position[3], float Leg5Position[3], float Leg6Position[3], FSUS_INTERVAL_T time);
    void InitPos(u8_t pos1, u8_t pos2);
    void InitPos(u8_t pos1, u8_t pos2, u8_t loopnum, FSUS_INTERVAL_T time);
    u8_t loopnum = 1;
    u8_t defaultPos1;
    u8_t defaultPos2;
};

void RobotPos_Task(void *pvParameters);
void RobotPosUp_Task(void *pvParameters);
void RobotPosDown_Task(void *pvParameters);
void RobotPoscar_Task(void *pvParameters);
#endif // _ROBOT_H_