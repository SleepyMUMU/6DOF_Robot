#include "ROSConfig.h"

// IPAddress Server(192, 168, 31, 203); // ROS服务器的IP地址
IPAddress Server(192, 168, 48, 128); // ROS服务器的IP地址
uint16_t ServerPort = 11411;         // ROS服务器的端口
// IPAddress ROSServer{192, 168, 202, 129}; // ROS服务器的IP地址
// u16_t ROSPort = 11411;                  // ROS服务器的端口

char hello[24] = "RosServer is connected.";
unsigned long last_time = 0;
uint16_t period = 1000; // 心跳包发送周期，单位为毫秒

/* ROS配置构造函数
 *NULL
 *NULL
 */
ROSConfig::ROSConfig()
{
    this->server = Server;
    this->serverPort = ServerPort;
    this->ReceiveData = "";
}

/* ROS初始化
 *NULL
 *bool
 */
bool ROSConfig::RosInit()
{
    if (this->server[0] == 0 && this->server[1] == 0 && this->server[2] == 0 && this->server[3] == 0)
    {
        this->server = Server;
    }
    if (this->serverPort == 0)
    {
        this->serverPort = ServerPort;
    }
    while (!nh.connected()) // 未连接则循环连接
    {
        delay(10);
        // nh.getHardware()->setConnection(Server, ServerPort); // 设置 ROS 服务器地址和端口
        // nh.initNode();                                       // 初始化 ROS（机器人操作系统）节点
        // Serial.print("ROS IP=");                             // 获取本地IP
        // Serial.println(nh.getHardware()->getLocalIP());      // 获取本地IP
        // nh.subscribe(sub);                                   // 开始订阅
        // delay(10);
        if (nh.connected()) // 连接服务器
        {                   // 连接成功
            Serial.println("[ROS]Successful connected Server");
            Serial.printf("[ROS]Server IP: %d.%d.%d.%d\n", server[0], server[1], server[2], server[3]);
            return true;
        }
        else // 连接失败
        {
            Serial.println("[ROS]Fail to connected Server");
            //         // vTaskDelay(10);
            return false;
        }
    }
    return true;
}

/* ROS初始化任务
 *判断服务器连接是否超时，如果超时则重新连接
 *NULL
 *NULL
 */
void RosInit_Task(void *pvParm)
{
    ROSConfig *Target = (ROSConfig *)pvParm; // 接收对应TCPConfig对象
    while (true)
    {
        if (Target->RosInit()) // 初始化ROS连接
        {
            if (nh.connected())
            {
                Serial.println("[I][ROS]ROSInit Success."); // ROS连接成功
            }
            else
            {
                Serial.println(nh.connected());          // ROS连接失败
                Serial.println("[E][ROS]ROSInit Fail."); // ROS连接失败
            }

            xTaskCreate(RosServer_Task, "ROS_Server", 4096, Target, 1, &(Target->Ros_ServerTaskHandle)); // 创建ROS服务器任务
            // xTaskCreate(RosRunTimeEnvTask, "ROS_RunTimeEnv", 4096, Target, 1, &(Target->Ros_RunTimeTaskHandle)); // 创建ROS运行环境任务
            // xTaskCreate(tcpRunTimeEnvTaskCrtl, String(Target->serverName + "_RunEnvCrtl").c_str(), 4096, Target, 1, &(Target->Ctrl_TaskHandle)); // 创建TCP运行环境任务控制任务

            // if (showStateofRunningTask(Target->Ros_RunTimeTaskHandle, &Serial))
            // {
            //     vTaskSuspend(Target->Ros_RunTimeTaskHandle); // Suspend the RunTime_TaskHandle
            // }
            // else
            // {
            //     Serial.printf("[E][ROS Task] Server_TaskHandle is NULL.\n"); // 输出错误信息
            // }

            // if (showStateofRunningTask(Target->Ros_TaskHandleHandle, &Serial))
            // {
            //     ;
            // }
            // else
            // {
            //     Serial.printf("[E][ROS Task] Server_TaskHandle is NULL.\n"); // 输出错误信息
            // }

            // 挂起TCP运行环境任务
            vTaskSuspend(NULL); // 挂起TCP初始化任务
        }
    }
}

/*  ROS运行环境任务
 *  通过串口接收数据
 *  NULL
 *  NULL
 */
void RosRunTimeEnvTask(void *pvParam)
{
    ROSConfig *Target = (ROSConfig *)pvParam; // 接收对应TCPConfig对象
                                              // while (true)
                                              // {
                                              // if (!Target->truncateStream)
                                              // {
                                              // if (Target->ReceiveData.length() > 0)
                                              // {

    if (Target->ReceiveData == "w")
    {
        Serial.println("[I][RunTime]Straight Test.");
        TaskHandle_t taskHandle = NULL;
        xTaskCreate(straight_walk_task, "StraightTest", 4096, Target, 1, &taskHandle);
        Target->Ros_TaskHandleHandle = taskHandle;
    }
    // }
    // }
    vTaskResume(Target->Ros_ServerTaskHandle);
    vTaskDelete(Target->Ros_RunTimeTaskHandle);

    // }
}
/* ROS服务任务
 *判断服务器连接是否超时，如果超时则重新连接
 *NULL
 *NULL
 */
void RosServer_Task(void *pvParam)
{
    ROSConfig *Target = (ROSConfig *)pvParam; // 接收对应TCPConfig对象

    if (millis() - last_time > period) // 判断是否超时
    {
        last_time = millis();

        if (nh.connected())
        {
            Serial.println("[I][ROSServer]Connected to ROS Server.\n");
        }
        else
        {
            Serial.println("[E][ROS]Connect to ROS Server failed.\n");
            vTaskResume(Target->Ros_InitTaskHandle); // 唤醒ROS初始化任务
            vTaskSuspend(NULL);
        }
    }
    for (;;)
    {
        if (nh.connected())
        {
            // nh.spinOnce();
            // delay(1);
            // nh.spinOnce(); // ROS节点运行
            if (nh.connected())
            {
                // nh.spinOnce();
                Serial.println("[I][ROS Server]Received Data:" + Target->ReceiveData);                               //+ Target->ReceiveData
                                                                                                                     // vTaskResume(Target->Ros_RunTimeTaskHandle);                            // 唤醒ROS运行环境任务
                xTaskCreate(RosRunTimeEnvTask, "ROS_RunTimeEnv", 4096, Target, 1, &(Target->Ros_RunTimeTaskHandle)); // 创建ROS运行环境任务
            }
            else
            {
                Serial.println("[E][ROS Server]Connection to Server lost."); // 服务器断开连接
                Serial.println("[I][ROS Server]Reconnecting to Server.");    // 服务器断开连接
                vTaskResume(Target->Ros_InitTaskHandle);                     // 唤醒ROS初始化任务
            }
        }
        vTaskDelay(10);
        // vTaskSuspend(Target->Ros_ServerTaskHandle); // 挂起ROS服务器任务
        // vtaskD
    }
}
// /*bool showStateofRunningTask(TaskHandle_t TaskHandle, Stream *stream)
//  *显示正在运行的任务状态
//  *TaskHandle_t TaskHandle,Stream *stream
//  *bool
//  */
// bool showStateofRunningTask(TaskHandle_t TaskHandle, Stream *stream)
// {

//     eTaskState eState;
//     String TaskName;
//     // 获取任务状态
//     if (TaskHandle != NULL)
//     {
//         eState = eTaskGetState(TaskHandle);
//         TaskName = pcTaskGetTaskName(TaskHandle);
//     }
//     else
//     {
//         stream->printf("[Task State]TaskHandle is NULL.\n");
//         return false;
//     }

//     switch (eState)
//     {
//     case eReady:
//         stream->printf("[Task State]Task %s is ready to run.\n", TaskName.c_str());
//         break;
//     case eRunning:
//         stream->printf("[Task State]Task %s is currently running.\n", TaskName.c_str());
//         break;
//     case eBlocked:
//         stream->printf("[Task State]Task %s is blocked.\n", TaskName.c_str());
//         break;
//     case eSuspended:
//         stream->printf("[Task State]Task %s is suspended.\n", TaskName.c_str());
//         break;
//     case eDeleted:
//         stream->printf("[Task State]Task %s has been deleted.\n", TaskName.c_str());
//         break;
//     default:
//         stream->printf("[Task State]Unknown task state.\n");
//         break;
//     }
//     return true;
// }
