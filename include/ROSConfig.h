#ifndef _ROSCONFIG_H_
#define _ROSCONFIG_H_

#define FlushTime 2000 // 默认刷新时间

#include <Arduino.h>
#include <Servo.h>
#include "Robot.h"
#include <ros.h>
#include <std_msgs/String.h>

extern ros::NodeHandle nh;         // ROS节点句柄
extern std_msgs::String trans_msg; // 发布的消息
extern uint16_t period;            // 心跳包发送周期，单位为毫秒
extern IPAddress Server;        // ROS服务器的IP地址
extern uint16_t ServerPort;              // ROS服务器的端口
extern ros::Subscriber<std_msgs::String> sub;
class ROSConfig
{
public:
    TaskHandle_t Ros_InitTaskHandle = NULL;    // ROS初始化任务
    TaskHandle_t Ros_ServerTaskHandle = NULL;  // ROS服务器任务
    TaskHandle_t Ros_RunTimeTaskHandle = NULL; // ROS运行控制任务
    TaskHandle_t Ros_TaskHandleHandle = NULL;  // ROS任务
    TaskHandle_t TaskList[NumofTask];
    QueueHandle_t TCPQueue;
    QueueHandle_t LegQueue;

    u_int16_t RunningNum_Task = 0;
    IPAddress server; // ROS服务器的IP地址
    u16_t serverPort; // ROS服务器的端口
    String last_ReceiveData = "";
    String ReceiveData = "";
    bool truncateStream = false;
    ROSConfig();
    bool RosInit();
};
// 声明函数
// bool showStateofRunningTask(TaskHandle_t TaskHandle, Stream *stream);
void RosInit_Task(void *pvParam);
void RosServer_Task(void *pvParam);
void RosRunTimeEnvTask(void *pvParam);
#endif // DEBUG
