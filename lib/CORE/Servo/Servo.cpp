#include "Servo.h"

LegConfig::LegConfig(HardwareSerial *serial, uint32_t ServoBaud, uint8_t ServoID, uint8_t ServoID2, uint8_t ServoID3)
{
    this->serial = serial;       // 串口ID
    this->ServoBaud = ServoBaud; // 舵机波特率
    this->ServoID = ServoID;     // 舵机ID
    this->ServoID2 = ServoID2;   // 舵机ID2
    this->ServoID3 = ServoID3;   // 舵机ID3
}

void LegConfig::LegInit()
{
    FSUS_Protocol Protocol(*(this->serial), this->ServoBaud);
    this->protocol =  Protocol; // 初始化舵机串口通信协议
    this->Servo1.init(this->ServoID, &this->protocol);                      // 初始化舵机1
    this->Servo2.init(this->ServoID2, &this->protocol);                     // 初始化舵机2
    this->Servo3.init(this->ServoID3, &this->protocol);                     // 初始化舵机3
}