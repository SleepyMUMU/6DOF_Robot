#ifndef _UARTCONFIG_H_
#define _UARTCONFIG_H_
#include <Arduino.h>
//#include <CoreSet.h>
/*串口配置、调试端口定义*/
#define DebugSerial Serial

void UARTInit();
void DebugPrintTest(Stream *stream);
String readStringFromStream(Stream *stream);
#endif
 