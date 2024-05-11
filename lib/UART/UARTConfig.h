#ifndef _UARTCONFIG_H_
#define _UARTCONFIG_H_

#include <Arduino.h>
// #include <CoreSet.h>
/*串口配置、调试端口定义*/
#define DebugSerial Serial

#define ServoSerial Serial
#define ServoSerialBaud 115200
#define ServoSerialRxPin 
#define ServoSerialTxPin 

#define ServoSerial1 Serial1
#define ServoSerial1Baud 115200
#define ServoSerial1Rx 18
#define ServoSerial1Tx 17

#define ServoSerial2 Serial2
#define ServoSerial2Baud 115200
#define ServoSerial2Rx 7
#define ServoSerial2Tx 6

void UARTInit();
void DebugPrintTest(Stream *stream);
String readStringFromStream(Stream *stream);
#endif
