#include <TCPConfig.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <UARTConfig.h>
#include <WiFiConfig.h>
#include <CoreSet.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <FreeRTOSConfig.h>
#include <Servo.h>
#include <FashionStar_UartServoProtocol.h>
#include <FashionStar_UartServo.h>

WiFiConfig WLAN;
TCPConfig MUMU;
TCPConfig JIAHONG;
LegConfig Test;

uint8_t num = 0;
// FSUS_Protocol Le;

// FSUS_Servo G1H(1, &Le);

void setup()
{

    coreSetEnable();
    UARTInit();       // 初始化串口
    WLAN.WiFiInit();  // 初始化WiFi
    WLAN.OTAconfig(); // 初始化OTA

    // 初始化机械臂协议
    FSUS_Protocol PL1;
    Serial2.begin(115200, SERIAL_8N1, 6, 7);
    PL1.init(&Serial2, 115200, 6, 7);
    Test.LegInit(PL1, 1, 2, 3, "Leg1");

    // Test.protocol();
    // Test.LegInit(PL1, G1H, G1K, G1A);
    // MUMU.LegQueue = xQueueCreate(numofLeg, sizeof(LegConfig *));
    // xQueueSend(MUMU.LegQueue, &Test, 0);

    // 生成一个消息队列，将对象MUMU和JIAHONG的指针传递给对方消息队列
    MUMU.TCPQueue = xQueueCreate(1, sizeof(TCPConfig *));
    JIAHONG.TCPQueue = xQueueCreate(1, sizeof(TCPConfig *));

    TCPConfig *JIAHONG_Pointer = &JIAHONG;
    TCPConfig *MUMU_Pointer = &MUMU;
    xQueueSend(MUMU.TCPQueue, &JIAHONG_Pointer, portMAX_DELAY);
    xQueueSend(JIAHONG.TCPQueue, &MUMU_Pointer, portMAX_DELAY);

    // 初始化TCP服务器配置
    MUMU.serverIP = MUMUServerIP;
    MUMU.serverPort = MUMUServerPort;
    MUMU.serverName = "MUMU";
    JIAHONG.serverIP = JIAHONGServerIP;
    JIAHONG.serverPort = JIAHONGServerPort;
    JIAHONG.serverName = "JIAHONG";
    Test.LegInit(PL1, G1H, G1K, G1A, "Leg1");
    xTaskCreate(OTATask, "OTA_Task", 4096, &WLAN, 1, &WLAN.OTA_TaskHandle);                     // OTA任务
    xTaskCreate(TCPInit_Task, "MUMU_TCP_Init", 4096, &MUMU, 1, &MUMU.Init_TaskHandle);          // TCP初始化任务,附加唤醒TCP服务器任务、TaskRunTimeEnv任务
    xTaskCreate(TCPInit_Task, "JIAHONG_TCP_Init", 4096, &JIAHONG, 1, &JIAHONG.Init_TaskHandle); // TCP初始化任务,附加唤醒TCP服务器任务、TaskRunTimeEnv任务
}

void loop()
{
    Test.hipServo.setAngle(40,1000);
    Test.kneeServo.setAngle(40,1000);
    Test.ankleServo.setAngle(40,1000);
}
