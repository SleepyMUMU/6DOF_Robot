#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <Arduino.h>
#include <Servo.h>
#include <FashionStar_UartServo.h>
#include <FashionStar_UartServoProtocol.h>

// #include <mymath.h>
#define RobotInitPosTime 1000
#define defaultpos1 0
#define defaultpos2 1

extern float defaultPosition[11][3];

#define Lenth 225.68     // 前端长度mm
#define Mid_Lenth 242.68 // 中间长度mm
#define Width 86         // 宽度mm

extern float vector_Stand[6][3];

class Robot
{
public:
    Robot(LegConfig leg1, LegConfig leg2, LegConfig leg3, LegConfig leg4, LegConfig leg5, LegConfig leg6);
    LegConfig Leg[6];

    Position Vector_bh[6];   // 身体到足起始向量
    Position Vector_S;       // 站立足起始端到足末端向量
    Position Vector_Move[6]; // 身体移动后起始到末端向量
    Position Change_Pos;     // 身体移动向量
    Robot(FSUS_Protocol INputPol1, FSUS_Protocol INputPol2);
    void RobotInit();
    void SetPos(float Leg1Position[3], float Leg2Position[3], float Leg3Position[3], float Leg4Position[3], float Leg5Position[3], float Leg6Position[3]);
    void SetPos(float Leg1Position[3], float Leg2Position[3], float Leg3Position[3], float Leg4Position[3], float Leg5Position[3], float Leg6Position[3], FSUS_INTERVAL_T time);
    void InitPos(u8_t pos1, u8_t pos2);
    void InitPos(u8_t pos1, u8_t pos2, u8_t loopnum, FSUS_INTERVAL_T time);
    void MoveBody(float x, float y, float z);
    u8_t loopnum = 1;
    u8_t defaultPos1;
    u8_t defaultPos2;
};
void car();
void stand();
void RobotPos_Task(void *pvParameters);
void RobotPoscar_Task(void *pvParameters);
void RobotChangePos_Task(void *pvParameters);
void RobotPosStand_Task(void *pvParameters);
#endif // _ROBOT_H_