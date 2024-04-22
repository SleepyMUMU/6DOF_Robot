#include "TCPConfig.h"

TCPConfig::TCPConfig()
{
    this->serverIP = defaultServerIP;
    this->serverPort = defaultServerPort;
}

// 连接指定服务器
bool TCPConfig::TCPInit()
{
    // 初始化前判断serverIP和serverPort是否已被初始化，如果没有则使用默认值
    if (serverIP[0] == 0 && serverIP[1] == 0 && serverIP[2] == 0 && serverIP[3] == 0)
    {
        this->serverIP = defaultServerIP;
    }
    if (serverPort == 0)
    {
        this->serverPort = defaultServerPort;
    }

    while (!TCP.connected()) // 未连接则循环连接
    {
        delay(FlushTime);                                 // 等待刷新时间
        DebugSerial.println("[TCP]connecting to Server"); // 输出连接信息
        if (TCP.connect(serverIP, serverPort))            // 连接服务器
        {                                                 // 连接成功
            DebugSerial.println("[TCP]Successful connected Server");
            DebugSerial.printf("[TCP]Server IP: %d.%d.%d.%d\n", serverIP[0], serverIP[1], serverIP[2], serverIP[3]);
            return true;
        }
        else
        { // 连接失败
            DebugSerial.println("[TCP]Fail to connected Server");
            return false;
        }
    }
}

// 连接指定服务器
bool TCPConfig::TCPInit(IPAddress serverIP, u16_t serverPort)
{
    this->serverIP = serverIP;
    this->serverPort = serverPort;

    while (!TCP.connected())
    {
        delay(FlushTime);
        DebugSerial.println("[TCP]connecting Server");
        if (TCP.connect(serverIP, serverPort))
        {
            DebugSerial.println("[TCP]Successful connected Server");
            DebugSerial.printf("[TCP]Server IP: %d.%d.%d.%d\n", serverIP[0], serverIP[1], serverIP[2], serverIP[3]);
            return true;
        }
        else
        {
            DebugSerial.println("[TCP]Fail to connected Server");
            return false;
        }
    }
}

void TCPServer_Task(void *pvParam)
{
    TCPConfig *Target = (TCPConfig *)pvParam; // 接收对应TCPConfig对象
    DebugSerial.println("[TCP Server]Running now.");
    // DebugPrintTest(&(Target->TCP));
    //  showStateofRunningTask(Target->Init_TaskHandle, &(Target->TCP));
    if (Target->TCP.connected())
    {
        DebugPrintTest(&(Target->TCP));
    }
    else
    {
        DebugSerial.println("[TCP Task]TCPInit Fail."); // TCP连接失败
                                                        // DebugSerial.print("Connection failed with error code: ");
    }
    while (true)
    {
        if (Target->TCP.connected()) // 判断是否连接成功
        {
            if (Target->TCP.available()) // 判断是否有数据可读
            {
                Target->ReceiveData = readStringFromStream(&(Target->TCP)); // 读取数据
                Target->TCP.print("[I][TCP Server]Received Data:" + Target->ReceiveData);
                vTaskResume(Target->RunTime_TaskHandle); // 唤醒TCP运行环境任务
            }
        }
        else
        {
            DebugSerial.println("[E][TCP Server]Connection to server lost."); // 服务器断开连接
            Target->TCP.stop();                                               // 停止连接
            DebugSerial.println("[I][TCP Server]Reconnecting to server.");    // 重新连接
            vTaskResume(Target->Init_TaskHandle);                             // 唤醒TCP初始化任务
            //  vTaskDelete(NULL);                                       // 删除TCP服务器任务
            vTaskSuspend(NULL); // 挂起TCP服务器任务
        }
        delay(1);
    }
}

void TCPInit_Task(void *pvParam)
{
    TCPConfig *Target = (TCPConfig *)pvParam; // 接收对应TCPConfig对象
    while (true)
    {
        if (Target->TCPInit()) // 初始化TCP连接
        {

            if (Target->TCP.connected())
            {
                DebugSerial.println("[TCP Task]TCPInit Success."); // TCP连接成功
            }
            else
            {
                DebugSerial.println("[TCP Task]TCPInit Fail."); // TCP连接失败
            }

            xTaskCreate(TCPServer_Task, String(Target->serverName + "_TCP_Server").c_str(), 4096, Target, 1, &(Target->Server_TaskHandle));     // 创建TCP服务器任务
            xTaskCreate(tcpRunTimeEnvTask, String(Target->serverName + "_RunTimeEnv").c_str(), 4096, Target, 1, &(Target->RunTime_TaskHandle)); // 创建TCP运行环境任务

            if (showStateofRunningTask(Target->RunTime_TaskHandle, &(Target->TCP)))
            {
                vTaskSuspend(Target->RunTime_TaskHandle); // Suspend the RunTime_TaskHandle
            }
            else
            {
                DebugSerial.printf("[E][TCP Task]%s Server_TaskHandle is NULL.\n", Target->serverName.c_str()); // 输出错误信息
            }

            if (showStateofRunningTask(Target->Server_TaskHandle, &(Target->TCP)))
            {
                ;
            }
            else
            {
                DebugSerial.printf("[E][TCP Task]%s Server_TaskHandle is NULL.\n", Target->serverName.c_str()); // 输出错误信息
            }

            // 挂起TCP运行环境任务

            vTaskSuspend(NULL); // 挂起TCP初始化任务
        }
        else // TCP连接失败
        {
            DebugSerial.printf("[W][TCP Task]%s TCPInit Fail,Reconnect in %dms.\n", Target->serverName.c_str(), FlushTime); // 输出错误信息
        }
        delay(1);
    }
}