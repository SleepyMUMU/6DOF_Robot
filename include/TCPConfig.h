#ifndef _TCPCONFIG_H_
#define _TCPCONFIG_H_

#include <Arduino.h>
// #include "WiFiClient.h"
#include <UARTConfig.h>
#include <CoreSet.h>
#include <WiFiConfig.h>
#include <WiFi.h>
/*********Server Config*********/
#define defaultServerIP \
    {                   \
        10, 3, 45, 69   \
    } // 默认服务器IP
#define MUMUServerIP  \
    {                 \
        10, 3, 45, 69 \
    } // MUMU服务器IP
#define JIAHONGServerIP \
    {                   \
        192,168,31,34   \
    } // JIAHONG服务器IP

#define MUMUServerPort 2345         // MUMU服务器端口
#define JIAHONGServerPort 2577      // JIAHONG服务器端口
#define defaultServerPort 2566      // 默认服务器端口
#define defaultServerName "Unknown" // 默认服务器名称
#define FlushTime 1000              // 默认刷新时间

class TCPConfig
{
public:

    WiFiClient TCP;
    IPAddress serverIP;
    u16_t serverPort;
    String serverName;
    String ReceiveData;
    TaskHandle_t Init_TaskHandle = NULL;
    TaskHandle_t Server_TaskHandle = NULL;
    TaskHandle_t RunTime_TaskHandle = NULL;
    TaskHandle_t Terminal_TaskHandle = NULL;
    TaskHandle_t TaskList[NumofTask];
    u_int16_t RunningNum_Task = 0;
    QueueHandle_t TCPQueue;
    QueueHandle_t LegQueue;
    TCPConfig();
    bool truncateStream = false;
    bool TCPInit();
    bool TCPInit(IPAddress serverIP, u16_t serverPort);
    // void TCPInit_Task(void *pvParam);   // Init By Task
    // void TCPServer_Task(void *pvParam); // Server By Task

private:
};
void TCPInit_Task(void *pvParam);
void TCPServer_Task(void *pvParam);
// void tcpRunTimeEnvTask(void *pvParam);
#endif // DEBUG
