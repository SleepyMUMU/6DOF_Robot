#include "Servo.h"

// 弃用Config，直接传递参数进Init
//  LegConfig::LegConfig(HardwareSerial *serial, uint32_t ServoBaud, uint8_t ServoID, uint8_t ServoID2, uint8_t ServoID3)
//  {
//      this->serial = serial;       // 串口ID
//      this->ServoBaud = ServoBaud; // 舵机波特率
//      this->ServoID = ServoID;     // 舵机ID
//      this->ServoID2 = ServoID2;   // 舵机ID2
//      this->ServoID3 = ServoID3;   // 舵机ID3


LegConfig::LegConfig()
{
}
LegConfig::~LegConfig()
{
}

void LegConfig::LegInit(FSUS_Protocol INputPol, uint8_t hipServoID, uint8_t kneeServoID, uint8_t ankleServoID)
{
    this->hipServoID = hipServoID;                                             // 传入Hip髋关节舵机ID1
    this->kneeServoID = kneeServoID;                                           // 传入Knee膝关节舵机ID2
    this->ankleServoID = ankleServoID;                                         // 传入Ankle踝关节舵机ID3
    this->LegName = String(defaultLegName + String("#") + int(AddedNumofLeg)); // 传入LegName
    this->protocol = INputPol;                                                 // 初始化舵机串口通信协议
    this->hipServo.init(this->hipServoID, &this->protocol);                    // 初始化hip髋关节舵机
    this->kneeServo.init(this->kneeServoID, &this->protocol);                  // 初始化knee膝关节舵机
    this->ankleServo.init(this->ankleServoID, &this->protocol);                // 初始化ankle踝关节舵机
    LegQueue[AddedNumofLeg] = xQueueCreate(1, sizeof(LegConfig *));            // 创建一个消息队列
    LegConfig *LegPointer = this;                                              // 将LegConfig对象的指针传递给消息队列
    xQueueSend(LegQueue[AddedNumofLeg], &LegPointer, portMAX_DELAY);           // 发送消息队列
    AddedNumofLeg++;                                                           // 增加机械臂数量
}
void LegConfig::LegInit(FSUS_Protocol INputPol, uint8_t hipServoID, uint8_t kneeServoID, uint8_t ankleServoID, String LegName)
{
    this->hipServoID = hipServoID;                                   // 传入Hip髋关节舵机ID1
    this->kneeServoID = kneeServoID;                                 // 传入Knee膝关节舵机ID2
    this->ankleServoID = ankleServoID;                               // 传入Ankle踝关节舵机ID3
    this->LegName = LegName;                                         // 传入LegName
    this->protocol = INputPol;                                       // 初始化舵机串口通信协议
    this->hipServo.init(this->hipServoID, &this->protocol);          // 初始化hip髋关节舵机
    this->kneeServo.init(this->kneeServoID, &this->protocol);        // 初始化knee膝关节舵机
    this->ankleServo.init(this->ankleServoID, &this->protocol);      // 初始化ankle踝关节舵机
    LegQueue[AddedNumofLeg] = xQueueCreate(1, sizeof(LegConfig *));  // 创建一个消息队列
    LegConfig *LegPointer = this;                                    // 将LegConfig对象的指针传递给消息队列
    xQueueSend(LegQueue[AddedNumofLeg], &LegPointer, portMAX_DELAY); // 发送消息队列
    AddedNumofLeg++;
}

void LegConfig::LegSetHipAngle(FSUS_SERVO_ANGLE_T targetangle, FSUS_INTERVAL_T runTime)
{
    this->hipServo.setAngle(targetangle, runTime); // 移动hip髋关节舵机
    delay(runTime);                                // 延时
}

void LegConfig::LegSetKneeAngle(FSUS_SERVO_ANGLE_T targetangle, FSUS_INTERVAL_T runTime)
{
    this->kneeServo.setAngle(targetangle, runTime); // 移动knee膝关节舵机
    delay(runTime);                                 // 延时
}

void LegConfig::LegSetAnkleAngle(FSUS_SERVO_ANGLE_T targetangle, FSUS_INTERVAL_T runTime)
{
    this->ankleServo.setAngle(targetangle, runTime); // 移动ankle踝关节舵机
    delay(runTime);                                  // 延时
}
// void LegConfig::LegSetAngle(FSUS_SERVO_ANGLE_T targetangle, FSUS_INTERVAL_T runTime)
// {
//     this->hipServo.setAngle(targetangle, runTime);   // 移动hip髋关节舵机
//     this->kneeServo.setAngle(targetangle, runTime);  // 移动knee膝关节舵机
//     this->ankleServo.setAngle(targetangle, runTime); // 移动ankle踝关节舵机
//     delay(runTime);                                  // 延时
// }

uint8_t LegConfig::LegPing()
{
    // 因为ping返回的是bool值，所以我希望将这三个bool值以二进制的形式返回给调用者，以便调用者知道ping的结果
    return ThreeBool2Bin(this->hipServo.ping(), this->kneeServo.ping(), this->ankleServo.ping());
}

uint8_t LegConfig::ThreeBool2Bin(bool hipServo, bool kneeServo, bool ankleServo) // 三个bool值转换为一个二进制值
{
    return (hipServo << 2) | (kneeServo << 1) | ankleServo;
}

// bin2ThreeBool
void LegConfig::bin2ThreeBool(uint8_t bin, bool &hipServo, bool &kneeServo, bool &ankleServo)
{
    hipServo = (bin >> 2) & 0x01;
    kneeServo = (bin >> 1) & 0x01;
    ankleServo = bin & 0x01;
}

void LegConfig::ForwardKinematics(FSUS_SERVO_ANGLE_T hipAngle, FSUS_SERVO_ANGLE_T kneeAngle, FSUS_SERVO_ANGLE_T ankleAngle, float &x, float &y, float &z)
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
void LegConfig::InverseKinematics(float x, float y, float z, FSUS_SERVO_ANGLE_T &hipAngle, FSUS_SERVO_ANGLE_T &kneeAngle, FSUS_SERVO_ANGLE_T &ankleAngle)
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

// to be continued
// void LegConfig::LegInit(FSUS_Protocol INput)
// {
//     this->protocol = INput; // 初始化舵机串口通信协议
//     // this->Servo1.init(this->ServoID, &this->protocol);  // 初始化舵机1
//     // this->Servo2.init(this->ServoID2, &this->protocol); // 初始化舵机2
//     // this->Servo3.init(this->ServoID3, &this->protocol); // 初始化舵机3
// }

void LegPing_Task(void *pvParameters)
{
    LegConfig *Target = (LegConfig *)pvParameters; // 接收对应LegConfig对象
    while (1)
    {
        uint8_t pingResult = Target->LegPing();
        bool hipServo, kneeServo, ankleServo;
        Target->bin2ThreeBool(pingResult, hipServo, kneeServo, ankleServo);
        if (hipServo)
        {
            DebugSerial.println("Hip Servo is online");
        }
        else
        {
            DebugSerial.println("Hip Servo is offline");
        }

        if (kneeServo)
        {
            DebugSerial.println("Knee Servo is online");
        }
        else
        {
            DebugSerial.println("Knee Servo is offline");
        }

        if (ankleServo)
        {
            DebugSerial.println("Ankle Servo is online");
        }
        else
        {
            DebugSerial.println("Ankle Servo is offline");
        }

        if (hipServo && kneeServo && ankleServo)
        {
            Target->LegSetHipAngle(defaultLeg1HipAngle, 2);
            Target->LegSetKneeAngle(defaultLeg1KneeAngle, 2);
            Target->LegSetAnkleAngle(defaultLeg1AnkleAngle, 2);
        }
        else
        {
            DebugSerial.println("Some Servo is offline");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void LegConfig::userLegSelect(LegConfig *Leg, uint8_t jointNum, float x, float y, float z)
{
    InverseKinematics(x, y, z, this->hipAngle, this->kneeAngle, this->ankleAngle);

    switch (jointNum)
    {
    case 1:
        Leg->LegSetHipAngle(hipAngle, 2);

        break;
    case 2:
        Leg->LegSetKneeAngle(kneeAngle, 2);

        break;
    case 3:
        Leg->LegSetAnkleAngle(ankleAngle, 2);
        break;
    default:
        break;
    }
}