#include "Servo.h"

// FSUS_Protocol debugServo(&debugSerial, debugBaundRate);

// FSUS_Servo debug1Hip(Group1_1HipServo, &debugServo);
// FSUS_Servo debug2Hip(Group1_1KneeServo, &debugServo);
// FSUS_Servo debug3Hip(Group1_1AnkleServo, &debugServo);

// void MyServo::Servo_Init()
// {
//     debugServo.init(&debugSerial, debugBaundRate, debugSerial_Rx, debugSerial_Tx);
//     debug1Hip.init();
//     debug2Hip.init();
//     debug3Hip.init();
// }

// void MyServo::Servo_Check()
// {

//     Servo1 = debug1Hip.ping();
//     Servo2 = debug2Hip.ping();
//     Servo3 = debug3Hip.ping();
//     if (Servo1)
//     {
//         Serial.println("Servo1 is online");
//     }
//     else
//     {
//         Serial.println("Servo1 is offline");
//     }

//     if (Servo2)
//     {
//         Serial.println("Servo2 is online");
//     }
//     else
//     {
//         Serial.println("Servo2 is offline");
//     }

//     if (Servo3)
//     {
//         Serial.println("Servo3 is online");
//     }
//     else
//     {
//         Serial.println("Servo3 is offline");
//     }
// }

// 弃用Config，直接传递参数进Init
//  LegConfig::LegConfig(HardwareSerial *serial, uint32_t ServoBaud, uint8_t ServoID, uint8_t ServoID2, uint8_t ServoID3)
//  {
//      this->serial = serial;       // 串口ID
//      this->ServoBaud = ServoBaud; // 舵机波特率
//      this->ServoID = ServoID;     // 舵机ID
//      this->ServoID2 = ServoID2;   // 舵机ID2
//      this->ServoID3 = ServoID3;   // 舵机ID3
//  }
u8_t AddedNumofLeg = 0;
QueueHandle_t LegQueue[numofLeg];//腿部队列

LegConfig::LegConfig()
{
}
LegConfig::~LegConfig()
{
}

LegConfig::LegConfig(FSUS_Protocol INputPol, uint8_t hipServoID, uint8_t kneeServoID, uint8_t ankleServoID)
{   
    this->protocol = INputPol;                                       // 初始化舵机串口通信协议
    
    this->LegServoID[0] = hipServoID;     // 传入Hip髋关节舵机ID
    this->LegServoID[1] = kneeServoID;   // 传入Knee膝关节舵机ID
    this->LegServoID[2] = ankleServoID; // 传入Ankle踝关节舵机ID

    this->LegServo[0].init(this->LegServoID[0], &this->protocol);          // 初始化hip髋关节舵机
    this->LegServo[1].init(this->LegServoID[1], &this->protocol);        // 初始化knee膝关节舵机
    this->LegServo[2].init(this->LegServoID[2], &this->protocol);      // 初始化ankle踝关节舵机

}
void LegConfig:: LegInit(FSUS_Protocol INputPol, uint8_t hipServoID, uint8_t kneeServoID, uint8_t ankleServoID)
{
    this->hipServoID = hipServoID;     // 传入Hip髋关节舵机ID1
    this->kneeServoID = kneeServoID;   // 传入Knee膝关节舵机ID2
    this->ankleServoID = ankleServoID; // 传入Ankle踝关节舵机ID3
    // this->LegName = String(defaultLegName + String("#") + int(AddedNumofLeg)); // 传入LegName
    this->protocol = INputPol;                                       // 初始化舵机串口通信协议
    this->hipServo.init(this->hipServoID, &this->protocol);          // 初始化hip髋关节舵机
    this->kneeServo.init(this->kneeServoID, &this->protocol);        // 初始化knee膝关节舵机
    this->ankleServo.init(this->ankleServoID, &this->protocol);      // 初始化ankle踝关节舵机
    LegQueue[AddedNumofLeg] = xQueueCreate(1, sizeof(LegConfig *));  // 创建一个消息队列
    LegConfig *LegPointer = this;                                    // 将LegConfig对象的指针传递给消息队列
    xQueueSend(LegQueue[AddedNumofLeg], &LegPointer, portMAX_DELAY); // 发送消息队列
    AddedNumofLeg++;                                                 // 增加机械臂数量
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

void LegConfig::fkine(FSUS_SERVO_ANGLE_T hipAngle, FSUS_SERVO_ANGLE_T kneeAngle, FSUS_SERVO_ANGLE_T ankleAngle, float &x, float &y, float &z)
{

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
void LegConfig::ikine(float x, float y, float z)
{
    // 机械臂的逆运动学解算
    this->hipAngle = atan2(y, x) * 180 / PI;
    float f1 = sqrt(pow(x,2)+pow(y,2));
    float f2 = z;
    float temp = L1 - f1;
    this->kneeAngle = (-atan(temp/ f2) + asin(pow(f1, 2) + pow(f2, 2) + pow(L1, 2) + pow(L2, 2) - pow(L3, 2) - 2*f1*L1) / (2 * L2 * sqrt(pow((f1 - L1), 2) + pow(f2, 2)))) * 180 / PI;
    this->ankleAngle = acos((pow(temp, 2) + pow(f2, 2) - pow(L2, 2) - pow(L3, 2)) / (2 * L2 * L3)) * 180 / PI;
}

void LegConfig::LegMoving(float x, float y, float z, uint8_t LegNum)
{
    LegConfig *Target;
    xQueueReceive(LegQueue[LegNum], &Target, portMAX_DELAY);
    Target->ikine(x, y, z);
    Target->hipServo.setAngle(Target->hipAngle + defaultLeg1HipAngle, 1000);
    Target->kneeServo.setAngle(Target->kneeAngle + defaultLeg1KneeAngle, 1000);
    Target->ankleServo.setAngle(-Target->ankleAngle + defaultLeg1AnkleAngle, 1000);
}


// void LegConfig::LegMoving(float x, float y, float z)
// {

//     ikine(x, y, z);
//     hipServo.setAngle(this->hipAngle + defaultLeg1HipAngle,1000);
//     kneeServo.setAngle(this->kneeAngle + defaultLeg1KneeAngle,1000);
//     ankleServo.setAngle(-this->ankleAngle + defaultLeg1AnkleAngle,1000);
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

// void Legdebug_Task(void *pvParameters)
// {
//     LegConfig *target = (LegConfig *)pvParameters;
//     while(true)
//     {
//         if(target->)

//     }


// }
