#include <Robot.h>
#include <main.h>
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
    // Target->TCP.println("[RobotPos]set all leg to the position of the robot.");
    // Target->TCP.println("[RobotPos]Please enter PosSerial Number of the Leg you want to Moving,default is 0.");
    // while (Target->ReceiveData == "")
    //     ;
    // u8_t PosSerial = Target->ReceiveData.toInt();
    robot.SetPos(defaultPosition[10], defaultPosition[10], defaultPosition[10], defaultPosition[10], defaultPosition[10], defaultPosition[10]);
    delay(servoDefaultTime);
    vTaskDelete(NULL);
}
void RobotPosUp_Task(void *pvParameters)
{
    TCPConfig *Target = (TCPConfig *)pvParameters; // 接收对应LegConfig对象
    Target->TCP.println("[RobotPosUp]set all leg to the position of the robot.");
    // Target->TCP.println("[RobotPosUp]Please enter PosSerial Number of the Leg you want to Moving,default is 0.");
    // while (Target->ReceiveData == "")
    //     ;
    u8_t PosSerial = Target->ReceiveData.toInt();
    robot.SetPos(defaultPosition[3], defaultPosition[3], defaultPosition[3], defaultPosition[3], defaultPosition[3], defaultPosition[3], servoDefaultTime);
    vTaskDelete(NULL);
}
void RobotPoscar_Task(void *pvParameters)
{
    TCPConfig *Target = (TCPConfig *)pvParameters; // 接收对应LegConfig对象
    Target->TCP.println("[RobotPosDown]set all leg to the position of the robot.");
    // Target->TCP.println("[RobotPosDown]Please enter PosSerial Number of the Leg you want to Moving,default is 0.");
    // while (Target->ReceiveData == "")
    //     ;
    u8_t PosSerial = Target->ReceiveData.toInt();
    robot.SetPos(defaultPosition[2], defaultPosition[2], defaultPosition[2], defaultPosition[2], defaultPosition[2], defaultPosition[2], servoDefaultTime);
    
    delay(2000);
    robot.SetPos(defaultPosition[3], defaultPosition[3], defaultPosition[3], defaultPosition[3], defaultPosition[3], defaultPosition[3], servoDefaultTime);
    delay(2000);
    robot.SetPos(defaultPosition[8], defaultPosition[7], defaultPosition[9], defaultPosition[8], defaultPosition[7], defaultPosition[9], servoDefaultTime);
    vTaskDelete(NULL);
}
void RobotPosDown_Task(void *pvParameters)
{
    TCPConfig *Target = (TCPConfig *)pvParameters; // 接收对应LegConfig对象
    Target->TCP.println("[RobotPosDown]set all leg to the position of the robot.");
    // Target->TCP.println("[RobotPosDown]Please enter PosSerial Number of the Leg you want to Moving,default is 0.");
    // while (Target->ReceiveData == "")
    //     ;
    u8_t PosSerial = Target->ReceiveData.toInt();
    robot.SetPos(defaultPosition[10], defaultPosition[10], defaultPosition[10], defaultPosition[10], defaultPosition[10], defaultPosition[10], servoDefaultTime);
    vTaskDelete(NULL);
}
     