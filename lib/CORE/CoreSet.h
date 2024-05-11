#ifndef _CORESET_H_
#define _CORESET_H_
#define CONFIG_FREERTOS_USE_TRACE_FACILITY
#define NumofTask 10

#include <Arduino.h>
#include <../Servo/Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <../UART/UARTConfig.h>
#include <TCPConfig.h>
// #define CONFIG_FREERTOS_USE_TRACE_FACILITY
// #define configUSE_TRACE_FACILITY 1
#include <../Robtics Control/Control.h>

void coreSetEnable();
void tcpRunTimeEnvTask(void *pvParam);
bool showStateofRunningTask(TaskHandle_t TaskHandle, Stream *stream);
void RobotPingTest(void *pvParam);
size_t TaskHindBind(TaskHandle_t *pxCreatedTask, void *pvParam);
void tcpRunTimeEnvTaskCrtl(void *pvParam);
void tcpCom_Task(void *pvParam);
void showTask(void *pvParam);
#endif // _CORESET_H_