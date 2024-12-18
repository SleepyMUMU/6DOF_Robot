#include <Robot.h>
#include <main.h>
float vector_Stand[6][3] = {
    {157.5, 0, -140.8},
    {157.5, 0, -140.8},
    {157.5, 0, -140.8},
    {157.5, 0, -140.8},
    {157.5, 0, -140.8},
    {157.5, 0, -140.8},
}; // 站立足起始端到足末端向量默认值

float defaultPosition[11][3] = {
    {10.26, 2.389, -22.53}, // 0
    {220.1, 84.48, -64.17}, // 1
    {156.6, 49.98, -140.1}, // 2
    {140.7, 46, -196},      // 3
    {157.5, 0, -140.8},     // 4
    {65.57, 0, -69.96},     // 5
    {120.8, 0, 77.15},      // 6
    {11.2, 0, -18.9},       // 7
    {7.07, -7.07, -18.9},   // 8前腿车位置
    {7.07, 7.07, -18.9},    // 9后腿车位置
    {168.91, 0, -68.65}
    // {7.07, 7.07, -22.53}    // 9后腿车位置
};

void Robot::RobotInit()
{
    // 身体到足起始向量
    Vector_bh[0] = Position(-Lenth / 2, Width / 2, 0);
    Vector_bh[1] = Position(-Mid_Lenth / 2, 0, 0);
    Vector_bh[2] = Position(-Lenth / 2, -Width / 2, 0);
    Vector_bh[3] = Position(Lenth / 2, Width / 2, 0);
    Vector_bh[4] = Position(Mid_Lenth / 2, 0, 0);
    Vector_bh[5] = Position(Lenth / 2, -Width / 2, 0);

    // 站立足起始端到足末端向量
    Vector_S = Position(157.5, 0, -140.8);
}
Robot::Robot(FSUS_Protocol INputPol1, FSUS_Protocol INputPol2)
{
    Leg[0] = LegConfig(INputPol2, 1);
    Leg[1] = LegConfig(INputPol2, 2);
    Leg[2] = LegConfig(INputPol2, 3);
    Leg[3] = LegConfig(INputPol1, 4);
    Leg[4] = LegConfig(INputPol1, 5);
    Leg[5] = LegConfig(INputPol1, 6);

    defaultPos1 = defaultpos1;
    defaultPos2 = defaultpos2;
}

Robot::Robot(LegConfig leg1, LegConfig leg2, LegConfig leg3, LegConfig leg4, LegConfig leg5, LegConfig leg6)
{
    Leg[0] = leg1;
    Leg[1] = leg2;
    Leg[2] = leg3;
    Leg[3] = leg4;
    Leg[4] = leg5;
    Leg[5] = leg6;
    defaultPos1 = defaultpos1;
    defaultPos2 = defaultpos2;
}

void Robot::SetPos(float Leg1Position[3], float Leg2Position[3], float Leg3Position[3], float Leg4Position[3], float Leg5Position[3], float Leg6Position[3])
{
    Leg[0].LegMoving(Leg1Position[0], Leg1Position[1], Leg1Position[2], servoDefaultTime);
    Leg[1].LegMoving(Leg2Position[0], Leg2Position[1], Leg2Position[2], servoDefaultTime);
    Leg[2].LegMoving(Leg3Position[0], Leg3Position[1], Leg3Position[2], servoDefaultTime);
    Leg[3].LegMoving(Leg4Position[0], Leg4Position[1], Leg4Position[2], servoDefaultTime);
    Leg[4].LegMoving(Leg5Position[0], Leg5Position[1], Leg5Position[2], servoDefaultTime);
    Leg[5].LegMoving(Leg6Position[0], Leg6Position[1], Leg6Position[2], servoDefaultTime);
}

void Robot::SetPos(float Leg1Position[3], float Leg2Position[3], float Leg3Position[3], float Leg4Position[3], float Leg5Position[3], float Leg6Position[3], FSUS_INTERVAL_T time)
{
    Leg[0].LegMoving(Leg1Position[0], Leg1Position[1], Leg1Position[2], time);
    Leg[1].LegMoving(Leg2Position[0], Leg2Position[1], Leg2Position[2], time);
    Leg[2].LegMoving(Leg3Position[0], Leg3Position[1], Leg3Position[2], time);
    Leg[3].LegMoving(Leg4Position[0], Leg4Position[1], Leg4Position[2], time);
    Leg[4].LegMoving(Leg5Position[0], Leg5Position[1], Leg5Position[2], time);
    Leg[5].LegMoving(Leg6Position[0], Leg6Position[1], Leg6Position[2], time);
}
void Robot::InitPos(u8_t pos1, u8_t pos2)
{
    this->defaultPos1 = pos1;
    this->defaultPos2 = pos2;
    u8_t loopnum = 1;
    FSUS_INTERVAL_T time = RobotInitPosTime;
    for (u8_t i = 0; i < loopnum; i++)
    {
        robot.SetPos(defaultPosition[defaultPos1], defaultPosition[defaultPos1], defaultPosition[defaultPos1], defaultPosition[defaultPos1], defaultPosition[defaultPos1], defaultPosition[defaultPos1], time);
        delay(time);
        robot.SetPos(defaultPosition[defaultpos2], defaultPosition[defaultpos2], defaultPosition[defaultpos2], defaultPosition[defaultpos2], defaultPosition[defaultpos2], defaultPosition[defaultpos2], time);
        delay(time);
    }
}
void Robot::InitPos(u8_t pos1, u8_t pos2, u8_t loopnum, FSUS_INTERVAL_T time)
{
    this->defaultPos1 = pos1;
    this->defaultPos2 = pos2;
    for (u8_t i = 0; i < loopnum; i++)
    {
        robot.SetPos(defaultPosition[defaultPos1], defaultPosition[defaultPos1], defaultPosition[defaultPos1], defaultPosition[defaultPos1], defaultPosition[defaultPos1], defaultPosition[defaultPos1], time);
        delay(time);
        robot.SetPos(defaultPosition[defaultpos2], defaultPosition[defaultpos2], defaultPosition[defaultpos2], defaultPosition[defaultpos2], defaultPosition[defaultpos2], defaultPosition[defaultpos2], time);
        delay(time);
    }
}

void Robot::MoveBody(float x, float y, float z)
{
    Vector_Move[0] = Position(Vector_S.x + x, Vector_S.y + y, Vector_S.z - z);
    Vector_Move[1] = Position(Vector_S.x + x, Vector_S.y + y, Vector_S.z - z);
    Vector_Move[2] = Position(Vector_S.x + x, Vector_S.y + y, Vector_S.z - z);
    Vector_Move[3] = Position(Vector_S.x - x, Vector_S.y - y, Vector_S.z - z);
    Vector_Move[4] = Position(Vector_S.x - x, Vector_S.y - y, Vector_S.z - z);
    Vector_Move[5] = Position(Vector_S.x - x, Vector_S.y - y, Vector_S.z - z);

    Leg[0].LegMoving(Vector_Move[0].x, Vector_Move[0].y, Vector_Move[0].z, 2000);
    Leg[1].LegMoving(Vector_Move[1].x, Vector_Move[1].y, Vector_Move[1].z, 2000);
    Leg[2].LegMoving(Vector_Move[2].x, Vector_Move[2].y, Vector_Move[2].z, 2000);
    Leg[3].LegMoving(Vector_Move[3].x, Vector_Move[3].y, Vector_Move[3].z, 2000);
    Leg[4].LegMoving(Vector_Move[4].x, Vector_Move[4].y, Vector_Move[4].z, 2000);
    Leg[5].LegMoving(Vector_Move[5].x, Vector_Move[5].y, Vector_Move[5].z, 2000);

    // 姿态变换后站立足起始端到足末端向量
    for (u8_t i = 0; i < 6; i++)
    {
        for (u8_t j = 0; j < 3; j++)
        {
            if (j == 0)
                vector_Stand[i][j] = Vector_Move[i].x;
            else if (j == 1)
                vector_Stand[i][j] = Vector_Move[i].y;
            else if (j == 2)
                vector_Stand[i][j] = Vector_Move[i].z;
        }
    }
}
// void car()
// {

//     for (uint8_t flag = 0; flag < 4; flag++)
//     {
//         if (flag == 0)
//         {
//             for (size_t i = 0; i < AddedNumofLeg; i++)
//             {
//                 LegConfig *TargetLeg;
//                 xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
//                 switch (i)
//                 {
//                 case 0:
//                     TargetLeg->LegSetAngle(defaultLeg1HipAngle, defaultLeg1KneeAngle, defaultLeg1AnkleAngle, 1000);
//                     break;
//                 case 1:
//                     TargetLeg->LegSetAngle(defaultLeg2HipAngle, defaultLeg2KneeAngle, defaultLeg2AnkleAngle, 1000);
//                     break;
//                 case 2:
//                     TargetLeg->LegSetAngle(defaultLeg3HipAngle, defaultLeg3KneeAngle, defaultLeg3AnkleAngle, 1000);
//                     break;
//                 case 3:
//                     TargetLeg->LegSetAngle(defaultLeg4HipAngle, defaultLeg4KneeAngle, defaultLeg4AnkleAngle, 1000);
//                     break;

//                 case 4:
//                     TargetLeg->LegSetAngle(defaultLeg5HipAngle, defaultLeg5KneeAngle, defaultLeg5AnkleAngle, 1000);
//                     break;
//                 case 5:
//                     TargetLeg->LegSetAngle(defaultLeg6HipAngle, defaultLeg6KneeAngle, defaultLeg6AnkleAngle, 1000);
//                     break;
//                 default:
//                     break;
//                 }
//             }
//             // vTaskDelay(500);
//         }
//         if (flag == 1)
//         {
//             for (size_t i = 0; i < AddedNumofLeg; i++)
//             {
//                 LegConfig *TargetLeg;
//                 xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
//                 switch (i)
//                 {
//                 case 0:
//                     TargetLeg->LegSetAngle(-45 + defaultLeg1HipAngle, defaultLeg1KneeAngle, defaultLeg1AnkleAngle, 1000);
//                     break;
//                 case 1:
//                     TargetLeg->LegSetAngle(0 + defaultLeg2HipAngle, defaultLeg2KneeAngle, defaultLeg2AnkleAngle, 1000);
//                     break;
//                 case 2:
//                     TargetLeg->LegSetAngle(45 + defaultLeg3HipAngle, defaultLeg3KneeAngle, defaultLeg3AnkleAngle, 1000);
//                     break;
//                 case 3:
//                     TargetLeg->LegSetAngle(-45 + defaultLeg4HipAngle, defaultLeg4KneeAngle, defaultLeg4AnkleAngle, 1000);
//                     break;

//                 case 4:
//                     TargetLeg->LegSetAngle(0 + defaultLeg5HipAngle, defaultLeg5KneeAngle, defaultLeg5AnkleAngle, 1000);
//                     break;
//                 case 5:
//                     TargetLeg->LegSetAngle(45 + defaultLeg6HipAngle, defaultLeg6KneeAngle, defaultLeg6AnkleAngle, 1000);
//                     break;
//                 default:
//                     break;
//                 }
//             }
//             // vTaskDelay(500);
//         }
//         if (flag == 2)
//         {
//             for (size_t i = 0; i < AddedNumofLeg; i++)
//             {
//                 LegConfig *TargetLeg;
//                 xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
//                 switch (i)
//                 {
//                 case 0:
//                     TargetLeg->LegSetAngle(-45 + defaultLeg1HipAngle, -55 + defaultLeg1KneeAngle, -35 + defaultLeg1AnkleAngle, 2000);
//                     break;
//                 case 1:
//                     TargetLeg->LegSetAngle(0 + defaultLeg2HipAngle, -35 + defaultLeg2KneeAngle, 50 + defaultLeg2AnkleAngle, 2000);
//                     break;
//                 case 2:
//                     TargetLeg->LegSetAngle(45 + defaultLeg3HipAngle, -55 + defaultLeg3KneeAngle, -35 + defaultLeg3AnkleAngle, 2000);
//                     break;
//                 case 3:
//                     TargetLeg->LegSetAngle(-45 + defaultLeg4HipAngle, -55 + defaultLeg4KneeAngle, -35 + defaultLeg4AnkleAngle, 2000);
//                     break;

//                 case 4:
//                     TargetLeg->LegSetAngle(0 + defaultLeg5HipAngle, -35 + defaultLeg5KneeAngle, 50 + defaultLeg5AnkleAngle, 2000);
//                     break;
//                 case 5:
//                     TargetLeg->LegSetAngle(45 + defaultLeg6HipAngle, -55 + defaultLeg6KneeAngle, -35 + defaultLeg6AnkleAngle, 2000);
//                     break;
//                 default:
//                     break;
//                 }
//             }
//             // vTaskDelay(500);
//         }
//         if (flag == 3)
//         {

//             for (size_t i = 0; i < AddedNumofLeg; i++)
//             {
//                 LegConfig *TargetLeg;
//                 xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
//                 switch (i)
//                 {
//                 case 0:
//                     TargetLeg->LegSetAngle(-45 + defaultLeg1HipAngle, -65 + defaultLeg1KneeAngle, 30 + defaultLeg1AnkleAngle, 1000);
//                     break;
//                 case 1:
//                     TargetLeg->LegSetAngle(0 + defaultLeg2HipAngle, -65 + defaultLeg2KneeAngle, 25 + defaultLeg2AnkleAngle, 1000);
//                     break;
//                 case 2:
//                     TargetLeg->LegSetAngle(45 + defaultLeg3HipAngle, -65 + defaultLeg3KneeAngle, 30 + defaultLeg3AnkleAngle, 1000);
//                     break;
//                 case 3:
//                     TargetLeg->LegSetAngle(-45 + defaultLeg4HipAngle, -65 + defaultLeg4KneeAngle, 30 + defaultLeg4AnkleAngle, 1000);
//                     break;

//                 case 4:
//                     TargetLeg->LegSetAngle(0 + defaultLeg5HipAngle, -65 + defaultLeg5KneeAngle, 25 + defaultLeg5AnkleAngle, 1000);
//                     break;
//                 case 5:
//                     TargetLeg->LegSetAngle(45 + defaultLeg6HipAngle, -65 + defaultLeg6KneeAngle, 30 + defaultLeg6AnkleAngle, 1000);
//                     break;
//                 default:
//                     break;
//                 }
//             }
//         }
//     }
// }

void RobotInit_Task(void *pvParameters)
{
    TCPConfig *Target = (TCPConfig *)pvParameters; // 接收对应LegConfig对象
    Target->TCP.println("[Robot]Robot init...");

    robot.SetPos(defaultPosition[0], defaultPosition[0], defaultPosition[0], defaultPosition[0], defaultPosition[0], defaultPosition[0], RobotInitPosTime);
    delay(RobotInitPosTime);
    robot.InitPos(0, 1);
    Target->TCP.println("[Robot]Robot init success.");
    vTaskDelete(NULL);
}

void RobotPos_Task(void *pvParameters)
{
    TCPConfig *Target = (TCPConfig *)pvParameters; // 接收对应LegConfig对象

    vTaskDelete(NULL);
}

void RobotChangePos_Task(void *pvParameters)
{
    while (true)
    {
        Position Change_Pos(0, 0, 0);
        robot.RobotInit();
        TCPConfig *Target = (TCPConfig *)pvParameters; // 接收对应LegConfig对象

        Target->TCP.println("[RobotChangePos]set body to the position of the robot.");
        Target->TCP.println("[RobotChangePos]Please enter the position of the robot.");
        Target->TCP.println("[RobotChangePos]Format: x y z");
        while (Target->ReceiveData == "")
            ;

        if (Target->ReceiveData.length() > 0)
        {
            if (Target->ReceiveData == "OK")
            {
                Target->ReceiveData = "";
                Target->truncateStream = false;
                vTaskSuspend(NULL);
            }
            else
            {
                Target->TCP.println("[RobotChangePos]The position of the robot is: " + Target->ReceiveData);
                sscanf(Target->ReceiveData.c_str(), "%f %f %f", &Change_Pos.x, &Change_Pos.y, &Change_Pos.z);
                robot.MoveBody(Change_Pos.x, Change_Pos.y, Change_Pos.z);
                Target->ReceiveData = "";
                Target->TCP.println("[RobotChangePos]The position of the robot is changed.");
                Target->truncateStream = true;
            }
        }
    }

    vTaskDelay(1);
}
