#include "CoreSet.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <FreeRTOSConfig.h>

// #define CONFIG_FREERTOS_USE_TRACE_FACILITY
// #define configUSE_TRACE_FACILITY 1

void coreSetEnable()
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector
    disableCore0WDT();
    disableCore1WDT();
}

// 创建函数 主逻辑树

void tcpRunTimeEnvTask(void *pvParam)
{
    // todo...
    TCPConfig *Target = (TCPConfig *)pvParam; // 接收对应TCPConfig对象
    while (true)
    {
        if (Target->ReceiveData.length() > 0)
        {
            if (Target->ReceiveData == "ping")
            {
                Target->TCP.println("[I][RunTime]Ping Test.");
                xTaskCreate(RobotPingTest, "RobotPingTest", 4096, &(Target->TCP), 1, NULL);
            }
            //...在这写功能逻辑
            
            else if (Target->ReceiveData == "sendmsg")  
            {
                Target->TCP.println("[I][RunTime]Send Message To Other Client.");
                

            }
            
            //ShowAllTask ... Todo
            // else if (Target->ReceiveData == "showTask")
            // {
            //     Target->TCP.println("[I][RunTime]Show All Task.");
            //     xTaskCreate(showAllTask, "ShowAllTask", 4096, NULL, 1, NULL);
            // }
            else
            {
                Target->TCP.println("[E][RunTime]Unknown Command.");
            }
            Target->ReceiveData = "";
        }
        vTaskSuspend(NULL);
        vTaskDelay(1);
    }
}

// 显示正在运行的任务
bool showStateofRunningTask(TaskHandle_t TaskHandle, Stream *stream)
{

    eTaskState eState;
    String TaskName;
    // 获取任务状态
    if (TaskHandle != NULL)
    {
        eState = eTaskGetState(TaskHandle);
        TaskName = pcTaskGetTaskName(TaskHandle);
    }
    else
    {
        stream->printf("[Task State]TaskHandle is NULL.\n");
        return false;
    }

    switch (eState)
    {
    case eReady:
        stream->printf("[Task State]Task %s is ready to run.\n", TaskName.c_str());
        break;
    case eRunning:
        stream->printf("[Task State]Task %s is currently running.\n", TaskName.c_str());
        break;
    case eBlocked:
        stream->printf("[Task State]Task %s is blocked.\n", TaskName.c_str());
        break;
    case eSuspended:
        stream->printf("[Task State]Task %s is suspended.\n", TaskName.c_str());
        break;
    case eDeleted:
        stream->printf("[Task State]Task %s has been deleted.\n", TaskName.c_str());
        break;
    default:
        stream->printf("[Task State]Unknown task state.\n");
        break;
    }
    return true;
}

void RobotPingTest(void *pvParam)
{
    Stream *stream = (Stream *)pvParam;
    DebugPrintTest(stream);
    vTaskDelete(NULL);
}

// To Fix up
//  void showAllTask(void *pvParam)
//  {
//      Stream *stream = (Stream *)pvParam;
//      TaskHandle_t *TaskList;                                            // 任务列表
//      uint8_t TaskNum = uxTaskGetNumberOfTasks();                        // 获取任务数量
//      TaskList = (TaskHandle_t *)malloc(TaskNum * sizeof(TaskHandle_t)); // 申请内存
//      if (TaskList == NULL)                                              // 内存申请失败
//      {
//          stream->println("[E][Show Task]TaskList malloc failed.");
//          vTaskDelete(NULL);
//      }
//      else
//      {
//          TaskNum = uxTaskGetSystemState((TaskStatus_t *)TaskList, TaskNum, NULL);
//          stream->printf("**************Task List**************\n");
//          stream->println("Ser.\tTaskName\tHandle");
//          for (uint8_t i = 0; i < TaskNum; i++)
//          {
//              stream->printf("%d\t%s\t%p\n", i, pcTaskGetTaskName(TaskList[i]), TaskList[i]);
//          }
//          stream->printf("[I][Task]TaskNum:%d\n", TaskNum);
//          free(TaskList);
//      }
//      vTaskDelete(NULL);
//  }
