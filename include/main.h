#ifndef _MAIN_H_
#define _MAIN_H_
#include <Arduino.h>
#include <Servo.h>
#include <Robot.h>
#include <ROSConfig.h>
extern TCPConfig MUMU;
extern TCPConfig JIAHONG;
extern Robot robot;

extern ros::NodeHandle nh;         // ROS节点句柄
extern std_msgs::String trans_msg; // 发布的消息
extern uint16_t period;            // 心跳包发送周期，单位为毫秒

#endif // _MAIN_H_
