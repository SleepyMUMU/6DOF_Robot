#ifndef CONTROL_H
#define CONTROL_H
#include <TCPConfig.h>
#include <../math/mymath.h>

#define PI 3.14159f
#define LEN_HtoK 84.0f  // L1髋长度
#define LEN_KtoA 73.5f  // L2大腿长度
#define LEN_AtoF 140.5f // L3小腿长度

class Move_Ctl
{
private:
    Position3 Rps[6]; // 机器人起始坐标

public:
    void Init();
    Theta ikCaculateTest(Position3 &pos);
};

void debugIK(void *PvParameters);

#endif