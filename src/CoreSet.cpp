#include "CoreSet.h"
#include "Robot.h"

#define CONFIG_FREERTOS_USE_TRACE_FACILITY
#define configUSE_TRACE_FACILITY 1

TaskHandle_t straight_taskHandle;    // 直行任务句柄
TaskHandle_t back_taskHandle;        // 后退任务句柄
TaskHandle_t left_taskHandle;        // 左行任务句柄
TaskHandle_t right_taskHandle;       // 右行任务句柄
TaskHandle_t left_cross_taskHandle;  // 左十字行任务句柄
TaskHandle_t right_cross_taskHandle; // 右十字行任务句柄
TaskHandle_t changepos_taskHandle;   // 改变姿态任务句柄
TaskHandle_t up_stairs_taskHandle;
TaskHandle_t Stand_taskHandle;
TaskHandle_t Car_taskHandle;
void coreSetEnable()
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // 禁用低压检测
    disableCore0WDT();                         // 禁用核心0看门狗
    disableCore1WDT();                         // 禁用核心1看门狗
}

// 创建函数 主逻辑树

void tcpRunTimeEnvTask(void *pvParam)
{
    // todo...
    TCPConfig *Target = (TCPConfig *)pvParam; // 接收对应TCPConfig对象
    while (true)
    {
        if (!Target->truncateStream)
        {
            /* code */

            if (Target->ReceiveData.length() > 0)
            {

                if (Target->ReceiveData == "ping")
                {
                    Target->TCP.println("[I][RunTime]Ping Test.");

                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(RobotPingTest, "RobotPingTest", 4096, Target, 1, &taskHandle);
                    TaskHindBind(&taskHandle, Target);
                    Target->Terminal_TaskHandle = taskHandle;
                }
                //...在这写功能逻辑
                else if (Target->ReceiveData == "debugik")
                {
                    Target->TCP.println("[I][RunTime]Debug IK Test.");

                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(debugIK, "DebugIK", 4096, Target, 1, &taskHandle);
                    TaskHindBind(&taskHandle, Target);
                    Target->Terminal_TaskHandle = taskHandle;
                    Target->truncateStream = true;
                }

                else if (Target->ReceiveData == "showtask")
                {
                    Target->TCP.println("[I][RunTime]Show All Task.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(showTask, "ShowAllTask", 4096, Target, 1, NULL);
                    TaskHindBind(&taskHandle, Target);
                    Target->Terminal_TaskHandle = taskHandle;
                }
                else if (Target->ReceiveData == "ctrl")
                {
                    Target->TCP.println("[I][RunTime]Leg Control.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(LegCrtl_Task, "LegControl", 4096, Target, 1, &taskHandle);
                    TaskHindBind(&taskHandle, Target);
                    Target->Terminal_TaskHandle = taskHandle;
                    Target->truncateStream = true;
                }
                else if (Target->ReceiveData == "legangle")
                {
                    Target->TCP.println("[I][RunTime]Leg Angle.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(LegAngleQuery_Task, "LegAngle", 4096, Target, 1, &taskHandle);
                    TaskHindBind(&taskHandle, Target);
                    Target->Terminal_TaskHandle = taskHandle;
                    Target->truncateStream = true;
                }

                else if (Target->ReceiveData == "setangle")
                {
                    Target->TCP.println("[I][RunTime]Set Angle.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(LegSetAngle_task, "SetAngle", 4096, Target, 1, &taskHandle);
                    TaskHindBind(&taskHandle, Target);
                    Target->Terminal_TaskHandle = taskHandle;
                    Target->truncateStream = true;
                }
                else if (Target->ReceiveData == "moving")
                {
                    Target->TCP.println("[I][RunTime]Leg Moving.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(LegMoving_Task, "LegMoving", 4096, Target, 1, &taskHandle);
                    TaskHindBind(&taskHandle, Target);
                    Target->Terminal_TaskHandle = taskHandle;
                    Target->truncateStream = true;
                }
                else if (Target->ReceiveData == "w")
                {
                    if (Target->Terminal_TaskHandle != NULL)
                    {
                        Target->TCP.printf("[E][RunTime]Task %s is Running.\n", pcTaskGetTaskName(Target->Terminal_TaskHandle));
                        vTaskDelete(Target->Terminal_TaskHandle);
                    }

                    Target->TCP.println("[I][RunTime]Straight Test.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(straight_walk_task, "StraightTest", 4096, Target, 1, &straight_taskHandle);
                    // TaskHindBind(&straight_taskHandle, Target);        // 任务绑定
                    Target->Terminal_TaskHandle = straight_taskHandle; // 任务句柄绑定
                    Target->truncateStream = false;                    // 截断流
                    // Target->truncateStream = true;
                }
                else if (Target->ReceiveData == "s")
                {

                    if (Target->Terminal_TaskHandle != NULL)
                    {
                        Target->TCP.printf("[E][RunTime]Task %s is Running.\n", pcTaskGetTaskName(Target->Terminal_TaskHandle));
                        vTaskDelete(Target->Terminal_TaskHandle);
                        Target->TCP.printf("[E][RunTime]Task %s is Running.\n", pcTaskGetTaskName(Target->RunTime_TaskHandle));
                    }
                    Target->TCP.println("[I][RunTime]Back Test.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(back_walk_task, "Back Test", 4096, Target, 1, &back_taskHandle);
                    // TaskHindBind(&back_taskHandle, Target);
                    Target->Terminal_TaskHandle = back_taskHandle;
                    Target->truncateStream = false;
                }
                else if (Target->ReceiveData == "l")
                {
                    if (Target->Terminal_TaskHandle != NULL)
                    {
                        Target->TCP.printf("[E][RunTime]Task %s is Running.\n", pcTaskGetTaskName(Target->Terminal_TaskHandle));
                        vTaskDelete(Target->Terminal_TaskHandle);
                    }
                    Target->TCP.println("[I][RunTime]left Test.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(left_walk_task, "leftTest", 4096, Target, 1, &left_taskHandle);
                    // TaskHindBind(&left_taskHandle, Target);
                    Target->Terminal_TaskHandle = left_taskHandle;
                    Target->truncateStream = false;
                }
                else if (Target->ReceiveData == "a")
                {
                    if (Target->Terminal_TaskHandle != NULL)
                    {
                        Target->TCP.printf("[E][RunTime]Task %s is Running.\n", pcTaskGetTaskName(Target->Terminal_TaskHandle));
                        vTaskDelete(Target->Terminal_TaskHandle);
                    }
                    Target->TCP.println("[I][RunTime]lcross Test.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(left_cross_walk_task, "lcrossTest", 4096, Target, 1, &left_cross_taskHandle);
                    // TaskHindBind(&left_cross_taskHandle, Target);
                    Target->Terminal_TaskHandle = left_cross_taskHandle;
                    Target->truncateStream = false;
                }
                else if (Target->ReceiveData == "r")
                {
                    if (Target->Terminal_TaskHandle != NULL)
                    {
                        Target->TCP.printf("[E][RunTime]Task %s is Running.\n", pcTaskGetTaskName(Target->Terminal_TaskHandle));
                        vTaskDelete(Target->Terminal_TaskHandle);
                    }
                    Target->TCP.println("[I][RunTime]right Test.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(right_walk_task, "right_walkTest", 4096, Target, 1, &right_taskHandle);
                    // TaskHindBind(&right_taskHandle, Target);
                    Target->Terminal_TaskHandle = right_taskHandle;
                    Target->truncateStream = false;
                }
                else if (Target->ReceiveData == "d")
                {
                    if (Target->Terminal_TaskHandle != NULL)
                    {
                        Target->TCP.printf("[E][RunTime]Task %s is Running.\n", pcTaskGetTaskName(Target->Terminal_TaskHandle));
                        vTaskDelete(Target->Terminal_TaskHandle);
                    }
                    Target->TCP.println("[I][RunTime]rcross Test.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(right_cross_walk_task, "right_cross_walkTest", 4096, Target, 1, &right_cross_taskHandle);
                    // TaskHindBind(&right_cross_taskHandle, Target);
                    Target->Terminal_TaskHandle = right_cross_taskHandle;
                    Target->truncateStream = false;
                }

                else if (Target->ReceiveData == "Stand")
                {
                    if (Target->Terminal_TaskHandle != NULL)
                    {
                        Target->TCP.printf("[E][RunTime]Task %s is Running.\n", pcTaskGetTaskName(Target->Terminal_TaskHandle));
                        vTaskDelete(Target->Terminal_TaskHandle);
                    }
                    Target->TCP.println("[I][RunTime]Stand Test.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(RobotPosStand_Task, "DownPosTest", 4096, Target, 1, &Stand_taskHandle);
                    Target->Terminal_TaskHandle = Stand_taskHandle;
                    Target->truncateStream = false;
                }
                else if (Target->ReceiveData == "up")
                {
                    if (Target->Terminal_TaskHandle != NULL)
                    {
                        Target->TCP.printf("[E][RunTime]Task %s is Running.\n", pcTaskGetTaskName(Target->Terminal_TaskHandle));
                        vTaskDelete(Target->Terminal_TaskHandle);
                    }
                    Target->TCP.println("[I][RunTime]Stand Test.");
                    xTaskCreate(up_stairs_walk_task, "DownPosTest", 4096, Target, 1, &up_stairs_taskHandle);
                    Target->Terminal_TaskHandle = up_stairs_taskHandle;
                    Target->truncateStream = false;
                }
                else if (Target->ReceiveData == "car")
                {
                    if (Target->Terminal_TaskHandle != NULL)
                    {
                        Target->TCP.printf("[E][RunTime]Task %s is Running.\n", pcTaskGetTaskName(Target->Terminal_TaskHandle));
                        vTaskDelete(Target->Terminal_TaskHandle);
                    }
                    Target->TCP.println("[I][RunTime]CarPosTest Test.");
                    Target->Terminal_TaskHandle = Car_taskHandle;
                    xTaskCreate(RobotPoscar_Task, "CarPosTest", 4096, Target, 1, NULL);
                    Target->truncateStream = false;
                }
                else if (Target->ReceiveData == "postest")
                {
                    Target->TCP.println("[I][RunTime]Pos Test.");
                    xTaskCreate(RobotPos_Task, "PosTest", 4096, Target, 1, NULL);
                }
                else if (Target->ReceiveData == "sendmsg")
                {
                    Target->TCP.println("[I][RunTime]Send Message To Other Client.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(tcpCom_Task, "tcpCom_Task", 4096, Target, 1, &taskHandle);
                    TaskHindBind(&taskHandle, Target);
                    Target->Terminal_TaskHandle = taskHandle;
                    Target->truncateStream = false;
                }
                else if (Target->ReceiveData == "restart")
                {
                    Target->TCP.println("[I][RunTime]Restarting....");
                    esp_restart();
                }
                else if (Target->ReceiveData == "powerdown")
                {
                    Target->TCP.println("[I][RunTime]Leg Power Down.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(LegPowerDown_Task, "LegPowerDown", 4096, Target, 1, &taskHandle);
                    TaskHindBind(&taskHandle, Target);
                    Target->Terminal_TaskHandle = taskHandle;
                    Target->truncateStream = false;
                }

                else if (Target->ReceiveData == "ipconfig")
                {
                    Target->TCP.println("[I][ipconfig]IP Config.");
                    Target->TCP.printf("[I][ipconfig]IP:%s\n", WiFi.localIP().toString().c_str());
                    Target->TCP.printf("[I][ipconfig]Gateway:%s\n", WiFi.gatewayIP().toString().c_str());
                    Target->TCP.printf("[I][ipconfig]Subnet:%s\n", WiFi.subnetMask().toString().c_str());
                    Target->TCP.printf("[I][ipconfig]DNS:%s\n", WiFi.dnsIP().toString().c_str());
                    Target->TCP.printf("[I][ipconfig]MAC:%s\n", WiFi.macAddress().c_str());
                }
                else if (Target->ReceiveData == "ChangePos")
                {
                    if (Target->Terminal_TaskHandle != NULL)
                    {
                        Target->TCP.printf("[E][RunTime]Task %s is Running.\n", pcTaskGetTaskName(Target->Terminal_TaskHandle));
                        vTaskDelete(Target->Terminal_TaskHandle);
                    }
                    Target->TCP.println("[I][RunTime]Change Pos.");
                    TaskHandle_t taskHandle = NULL;
                    xTaskCreate(RobotChangePos_Task, "ChangePos", 4096, Target, 1, &changepos_taskHandle);
                    // TaskHindBind(&taskHandle, Target);
                    Target->Terminal_TaskHandle = changepos_taskHandle;
                    // Target->truncateStream = false;
                }
                else if (Target->ReceiveData == "stop")
                {
                    if (Target->Terminal_TaskHandle != NULL && Target->Terminal_TaskHandle != Target->RunTime_TaskHandle)
                    {
                        Target->TCP.printf("[I][RunTime]Task %s is Exit.\n", pcTaskGetTaskName(Target->Terminal_TaskHandle));
                        vTaskSuspend(Target->Terminal_TaskHandle);
                        xTaskCreate(RobotPos_Task, "RobotInit", 4096, Target, 1, NULL);
                        Target->Terminal_TaskHandle = NULL;
                        Target->truncateStream = false;
                    }
                    else
                    {
                        Target->TCP.println("[E][RunTime]No User Task is Running.");
                        Target->truncateStream = false;
                    }
                }
                else if (Target->ReceiveData == "help")
                {
                    Target->TCP.println("[I][RunTime]Help.");
                    Target->TCP.println("[I][RunTime]ping:Ping Test.");
                    Target->TCP.println("[I][RunTime]debugik:Debug IK Test.");
                    Target->TCP.println("[I][RunTime]showtask:Show All Task.");
                    Target->TCP.println("[I][RunTime]ctrl:Leg Control.");
                    Target->TCP.println("[I][RunTime]legangle:Leg Angle.");
                    Target->TCP.println("[I][RunTime]setangle:Set Angle.");
                    Target->TCP.println("[I][RunTime]moving:Leg Moving.");
                    Target->TCP.println("[I][RunTime]straight:Straight Test.");
                    Target->TCP.println("[I][RunTime]back:Back Test.");
                    Target->TCP.println("[I][RunTime]left:left Test.");
                    Target->TCP.println("[I][RunTime]lcross:lcross Test.");
                    Target->TCP.println("[I][RunTime]right:right Test.");
                    Target->TCP.println("[I][RunTime]rcross:rcross Test.");
                    Target->TCP.println("[I][RunTime]up:UpPosTest Test.");
                    Target->TCP.println("[I][RunTime]down:DownPos Test.");
                    Target->TCP.println("[I][RunTime]car:CarPosTest Test.");
                    Target->TCP.println("[I][RunTime]postest:Pos Test.");
                    Target->TCP.println("[I][RunTime]sendmsg:Send Message To Other Client.");
                    Target->TCP.println("[I][RunTime]restart:Restarting....");
                    Target->TCP.println("[I][RunTime]powerdown:Leg Power Down.");
                    Target->TCP.println("[I][RunTime]ipconfig:IP Config.");
                    Target->TCP.println("[I][RunTime]ChangePos:Change Pos.");
                }
                else
                {
                    Target->TCP.printf("[E][RunTime]Unknown Command:%s\n", Target->ReceiveData.c_str());
                }
                Target->ReceiveData = "";
            }
        }
        vTaskSuspend(NULL);
        vTaskDelay(1);
    }
}
void tcpCom_Task(void *pvParam)
{
    TCPConfig *Target = (TCPConfig *)pvParam;
    TCPConfig *COMTarget = NULL;
    xQueuePeek(Target->TCPQueue, &COMTarget, 0);
    if (COMTarget == NULL)
    {
        Target->TCP.printf("[E][COM]No COM Target.\n");
        vTaskSuspend(NULL);
    }
    else
    {
        Target->TCP.printf("[I][COM]COM Target:%s\n", COMTarget->serverName.c_str());
        if (COMTarget->TCP.connected())
        {
            Target->TCP.printf("[I][COM]Connected to %s\n", COMTarget->serverName.c_str());
        }
        else
        {
            Target->TCP.printf("[E][COM]Connect to %s failed.\n", COMTarget->serverName.c_str());
            vTaskSuspend(NULL);
        }
    }

    while (true)
    {

        if (Target->ReceiveData.length() > 0)
        {

            Target->TCP.printf("[I][COM]Send Message:%s\n", Target->ReceiveData.c_str());
            COMTarget->TCP.printf("[I][Message]%s say: %s", Target->serverName.c_str(), Target->ReceiveData);
            Target->ReceiveData = "";
            Target->truncateStream = false; // 重置截断流
            // to do ...
        }

        vTaskDelay(1);
    }
}

void TaskHindBind(TaskHandle_t *pxCreatedTask, void *pvParam) // 查询是否有任务，有则返回任务序号，无则添加任务
{
    TCPConfig *Target = (TCPConfig *)pvParam;
    bool flag = false; // 任务是否已经存在
    for (size_t i = 0; i < Target->RunningNum_Task; i++)
    {
        if (Target->TaskList[i] == *pxCreatedTask)
        {
            flag = true;

            break;
        }
    }
    if (!flag)
    {
        Target->TaskList[Target->RunningNum_Task] = *pxCreatedTask;
        Target->RunningNum_Task++;
    }
}

void tcpRunTimeEnvTaskCrtl(void *pvParam)
{
    TCPConfig *Target = (TCPConfig *)pvParam;
    while (true)
    {
        if (Target->ReceiveData == "exit")
        {
            if (Target->Terminal_TaskHandle != NULL && Target->Terminal_TaskHandle != Target->RunTime_TaskHandle)
            {
                Target->TCP.printf("[I][RunTime]Task %s is Exit.\n", pcTaskGetTaskName(Target->Terminal_TaskHandle));
                vTaskSuspend(Target->Terminal_TaskHandle);
                xTaskCreate(RobotPos_Task, "RobotInit", 4096, Target, 1, NULL);
                Target->Terminal_TaskHandle = NULL;
                Target->truncateStream = false;
            }
            else
            {
                Target->TCP.println("[E][RunTime]No User Task is Running.");
                Target->truncateStream = false;
            }
        }
    }
}

// 显示正在运行的任务状态
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
    TCPConfig *Target = (TCPConfig *)pvParam;
    Stream *stream = &Target->TCP;
    DebugPrintTest(stream);

    Target->Terminal_TaskHandle = NULL;
    vTaskSuspend(NULL);
}

// To Fix up
void showTask(void *pvParam)
{
    TCPConfig *Target = (TCPConfig *)pvParam;
    Stream *stream = &Target->TCP;

    for (size_t i = 0; i < Target->RunningNum_Task; i++)
    {
        showStateofRunningTask(Target->TaskList[i], stream);
    }
    Target->Terminal_TaskHandle = NULL;
    vTaskSuspend(NULL);
}