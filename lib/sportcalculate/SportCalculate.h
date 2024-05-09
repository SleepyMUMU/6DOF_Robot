#ifndef _SPORTCALCULATE_H_
#define _SPORTCALCULATE_H_

#include <Arduino.h>
#include <mymath.h>

class Rob_Pos
{
private:
    Position Wps[6];//机器人末端相对于起始端的位置
    Position Wps_default[6];//默认情况下机器人末端相对于起始端的位置
    Position Wps_Leg[6];//各个机械腿起始端相对于机器人中心的坐标
    Position Rob_position;//机器人中心坐标
    Theta theta;
public:
    void Init();
}
#endif                                                                                                                                          // DEBUG