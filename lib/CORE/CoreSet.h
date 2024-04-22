#ifndef _CORESET_H_
#define _CORESET_H_
#include <Arduino.h>
#include <TCPConfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// #define CONFIG_FREERTOS_USE_TRACE_FACILITY
// #define configUSE_TRACE_FACILITY 1

void coreSetEnable();
void tcpRunTimeEnvTask(void *pvParam);
bool showStateofRunningTask(TaskHandle_t TaskHandle, Stream *stream);
void RobotPingTest(void *pvParam);
void showAllTask(void *pvParam);
#endif // _CORESET_H_