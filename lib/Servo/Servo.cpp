#include "Servo.h"

// 弃用Config，直接传递参数进Init
//  LegConfig::LegConfig(HardwareSerial *serial, uint32_t ServoBaud, uint8_t ServoID, uint8_t ServoID2, uint8_t ServoID3)
//  {
//      this->serial = serial;       // 串口ID
//      this->ServoBaud = ServoBaud; // 舵机波特率
//      this->ServoID = ServoID;     // 舵机ID
//      this->ServoID2 = ServoID2;   // 舵机ID2
//      this->ServoID3 = ServoID3;   // 舵机ID3
//  }

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