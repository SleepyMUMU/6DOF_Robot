
#ifndef _SERVO_H_
#define _SERVO_H_
#include <Arduino.h>
#include "FashionStar_UartServoProtocol.h" // 串口总线舵机通信协议
#include "FashionStar_UartServo.h"         // 串口总线舵机SDK

#define G1H 1
#define G1K 2
#define G1A 3

#define G2H 4
#define G2K 5
#define G2A 6

#define G3H 7
#define G3K 8
#define G3A 9

#define G4H 10
#define G4K 11
#define G4A 12

#define G5H 13
#define G5K 14
#define G5A 15

#define G6H 16
#define G6K 17
#define G6A 18

class LegConfig
{
public:
    FSUS_Protocol protocol; // 舵机串口通信协议
    FSUS_Servo Servo1;      // 舵机对象
    FSUS_Servo Servo2;      // 舵机对象
    FSUS_Servo Servo3;      // 舵机对象
    LegConfig(HardwareSerial *serial, uint32_t ServoBaud, uint8_t ServoID, uint8_t ServoID2, uint8_t ServoID3);
    ~LegConfig();

    HardwareSerial *serial;
    uint8_t ServoID;    // 舵机ID
    uint8_t ServoID2;   // 舵机ID2
    uint8_t ServoID3;   // 舵机ID3
    uint32_t ServoBaud; // 舵机波特率
    float currentangle; // 当前角度
    float targetangle;  // 目标角度
    uint8_t runTime;    // 运行时间

    void LegInit(); // 初始化舵机
    void LegInit(FSUS_Protocol INput);
    void LegInit(FSUS_Protocol INputPol, uint8_t ServoID, uint8_t ServoID2, uint8_t ServoID3);
    void LegMove(float targetangle, uint8_t runTime); // 移动舵机
};
#endif
