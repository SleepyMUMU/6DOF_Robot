#include <TCPConfig.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <UARTConfig.h>
#include <WiFiConfig.h>
#include <CoreSet.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <FreeRTOSConfig.h>

WiFiConfig WLAN;
TCPConfig MUMU;
TCPConfig JIAHONG;

void setup()
{

    coreSetEnable();
    UARTInit();       // 初始化串口
    WLAN.WiFiInit();  // 初始化WiFi
    WLAN.OTAconfig(); // 初始化OTA

    // 生成一个消息队列，将对象MUMU和JIAHONG的指针传递给消息队列
    MUMU.TCPQueue = xQueueCreate(1, sizeof(TCPConfig *));
    JIAHONG.TCPQueue = xQueueCreate(1, sizeof(TCPConfig *));
    xQueueSend(MUMU.TCPQueue, &JIAHONG, 0);
    xQueueSend(JIAHONG.TCPQueue, &MUMU, 0);


    // 初始化TCP服务器配置
    MUMU.serverIP = MUMUServerIP;
    MUMU.serverPort = MUMUServerPort;
    MUMU.serverName = "MUMU";
    JIAHONG.serverIP = JIAHONGServerIP;
    JIAHONG.serverPort = JIAHONGServerPort;
    JIAHONG.serverName = "JIAHONG";

    xTaskCreate(OTATask, "OTA_Task", 4096, &WLAN, 1, &WLAN.OTA_TaskHandle);            // OTA任务
    xTaskCreate(TCPInit_Task, "MUMU_TCP_Init", 4096, &MUMU, 1, &MUMU.Init_TaskHandle); // TCP初始化任务,附加唤醒TCP服务器任务、TaskRunTimeEnv任务
                                                                                       // xTaskCreate(TCPInit_Task, "JIAHONG_TCP_Init", 4096, &JIAHONG, 1, &JIAHONG.Init_TaskHandle); // TCP初始化任务,附加唤醒TCP服务器任务、TaskRunTimeEnv任务
}

void loop()
{
}
