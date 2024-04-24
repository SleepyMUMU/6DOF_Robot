
#ifndef _SERVO_H_
#define _SERVO_H_
#include <Arduino.h>
#include "FashionStar_UartServoProtocol.h" // 串口总线舵机通信协议
#include "FashionStar_UartServo.h"         // 串口总线舵机SDK
#include "TCPConfig.h"

/*********************Num of Servo*********************/
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
/*********************Config of Arm*********************/
#define defaultSerial Serial1
#define defaultServoBaud 115200
#define defaultServoID 1
#define defaultServoID2 2
#define defaultServoID3 3
// ..Wanning: The default<xxx> should not used in the original code

/**********************Other**********************/
#define defaultAngle 0
#define defaultTime 1000
#define NumofLeg 6

class LegConfig
{
public:
    FSUS_Protocol protocol; // 舵机串口通信协议
    FSUS_Servo hipServo;    // 舵机对象
    FSUS_Servo kneeServo;   // 舵机对象
    FSUS_Servo ankleServo;  // 舵机对象
    // LegConfig(HardwareSerial *serial, uint32_t ServoBaud, uint8_t ServoID, uint8_t ServoID2, uint8_t ServoID3);
    //~LegConfig();
    LegConfig();
    uint8_t hipServoID;   // hip髋关节舵机ID1
    uint8_t kneeServoID;  // knee膝关节舵机ID2
    uint8_t ankleServoID; // ankle舵机ID3

    void LegInit(); // 初始化舵机
    void LegInit(FSUS_Protocol INput);
    void LegInit(FSUS_Protocol INputPol, uint8_t ServoID, uint8_t ServoID2, uint8_t ServoID3);
    void LegSetAngle(FSUS_SERVO_ANGLE_T targetangle, FSUS_INTERVAL_T runTime); // 移动舵机
    uint8_t LegPing();

    uint8_t ThreeBool2Bin(bool hipServo, bool kneeServo, bool ankleServo);
    void bin2ThreeBool(uint8_t bin, bool &hipServo, bool &kneeServo, bool &ankleServo);
};
#endif
