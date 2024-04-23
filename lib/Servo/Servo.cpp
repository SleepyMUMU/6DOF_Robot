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

void LegConfig::LegInit(FSUS_Protocol INputPol, uint8_t hipServoID, uint8_t kneeServoID, uint8_t ankleServoID)
{
    this->hipServoID = hipServoID;     // 传入Hip髋关节舵机ID1
    this->kneeServoID = kneeServoID;   // 传入Knee膝关节舵机ID2
    this->ankleServoID = ankleServoID; // 传入Ankle踝关节舵机ID3

    this->protocol = INputPol;                                  // 初始化舵机串口通信协议
    this->hipServo.init(this->hipServoID, &this->protocol);     // 初始化hip髋关节舵机
    this->kneeServo.init(this->kneeServoID, &this->protocol);   // 初始化knee膝关节舵机
    this->ankleServo.init(this->ankleServoID, &this->protocol); // 初始化ankle踝关节舵机
}

// to be continued
void LegConfig::LegInit(FSUS_Protocol INput)
{
    this->protocol = INput; // 初始化舵机串口通信协议
    // this->Servo1.init(this->ServoID, &this->protocol);  // 初始化舵机1
    // this->Servo2.init(this->ServoID2, &this->protocol); // 初始化舵机2
    // this->Servo3.init(this->ServoID3, &this->protocol); // 初始化舵机3
}