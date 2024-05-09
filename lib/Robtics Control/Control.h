#ifndef CONTROL_H
#define CONTROL_H

#include "mymath.h"

#define PI 3.14159f
#define LEN_HtoK 84f    // 髋长度
#define LEN_KtoA 73.5f  // 大腿长度
#define LEN_AtoF 140.5f // 小腿长度

class Move_Ctl
{
private:
    Position Rps[6]; // 机器人起始坐标
    
public:
    void Init();
}

#endif