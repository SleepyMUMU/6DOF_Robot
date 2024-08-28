#include <TCPConfig.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Control.h>
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
#include <Robot.h>

WiFiConfig WLAN;
TCPConfig MUMU;
TCPConfig JIAHONG;
TCPConfig Conisn;

FSUS_Protocol protocol1(&ServoSerial1, ServoSerial1Baud);
FSUS_Protocol protocol2(&ServoSerial2, ServoSerial2Baud);

LegConfig Leg1(protocol2, 1); // 1号腿对象
LegConfig Leg2(protocol2, 2); // 2号腿对象
LegConfig Leg3(protocol2, 3); // 3号腿对象
LegConfig Leg4(protocol1, 4); // 4号腿对象
LegConfig Leg5(protocol1, 5); // 5号腿对象
LegConfig Leg6(protocol1, 6); // 6号腿对象

Robot robot(Leg1, Leg2, Leg3, Leg4, Leg5, Leg6);
// Robot robot(protocol1, protocol2);
uint8_t ReciveBuffer[200]; // 接收缓冲区
float ReciveData;          // 接收数据
uint8_t Num_Start;         // 数据起始
uint8_t Num_End;           // 数据末尾
uint8_t Flag;              // 符号标志位
uint8_t DataLength;        // 数据长度

void setup()
{
  delay(500);
  coreSetEnable();
  UARTInit(); // 初始化串口
  WiFi.begin(defaultSSID, defaultPassward);

  if (WLAN.WiFiInit()) // 初始化WiFi
  {
    DebugSerial.println("WiFi Init Success");
  }

  WLAN.OTAconfig(); // 初始化OTA

  robot.SetPos(defaultPosition[4], defaultPosition[4], defaultPosition[4], defaultPosition[4], defaultPosition[4], defaultPosition[4], 2000);

  // 生成一个消息队列，将对象MUMU和JIAHONG的指针传递给对方消息队列
  Conisn.TCPQueue = xQueueCreate(1, sizeof(TCPConfig *));
  JIAHONG.TCPQueue = xQueueCreate(1, sizeof(TCPConfig *));
  // // 生成队列传递腿部信息
  // Conisn.LegQueue = xQueueCreate(numofLeg, sizeof(LegConfig *));
  // Conisn.LegQueue = xQueueCreate(numofLeg, sizeof(LegConfig *));

  TCPConfig *JIAHONG_Pointer = &JIAHONG;
  TCPConfig *Conisn_Pointer = &Conisn;
  // TCPConfig *MUMU_Pointer = &MUMU;
  // xQueueSend(JIAHONG.TCPQueue, &MUMU_Pointer, portMAX_DELAY);
  xQueueSend(Conisn.TCPQueue, &JIAHONG_Pointer, portMAX_DELAY);
  xQueueSend(JIAHONG.TCPQueue, &Conisn_Pointer, portMAX_DELAY);

  // 初始化TCP服务器配置
  // MUMU.serverIP = MUMUServerIP;
  // MUMU.serverPort = MUMUServerPort;
  // MUMU.serverName = "MUMU";
  JIAHONG.serverIP = JIAHONGServerIP;
  JIAHONG.serverPort = JIAHONGServerPort;
  JIAHONG.serverName = "JIAHONG";
  Conisn.serverIP = ConisnServerIP;
  Conisn.serverPort = ConisnServerPort;
  Conisn.serverName = "Conisn";

    xTaskCreate(OTATask, "OTA_Task", 4096, &WLAN, 1, &WLAN.OTA_TaskHandle); // OTA任务
    // xTaskCreate(TCPInit_Task, "MUMU_TCP_Init", 4096, &MUMU, 1, &MUMU.Init_TaskHandle);          // TCP初始化任务,附加唤醒TCP服务器任务、TaskRunTimeEnv任务
    xTaskCreate(TCPInit_Task, "JIAHONG_TCP_Init", 4096, &JIAHONG, 1, &JIAHONG.Init_TaskHandle); // TCP.初始化任务,附加唤醒TCP服务器任务、TaskRunTimeEnv任务
    xTaskCreate(TCPInit_Task, "Conisn_TCP_Init", 4096, &Conisn, 1, &Conisn.Init_TaskHandle);    // TCP.初始化任务,附加唤醒TCP服务器任务、TaskRunTimeEnv任务
  
}
  void loop()
  {
  }
