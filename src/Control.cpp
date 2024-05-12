#include "Control.h"

// static Position3 fkine(Theta thetas);
// static Theta ikine(Position3 &pos);

// 角度---->弧度
float D2R(float deg)
{

    // String CMD;
    // CMD.toInt();
    return deg * PI / 180;
}

// 弧度---->角度
float R2D(float rad)
{
    return rad * 180 / PI;
}

static Position3 fkine(Theta thetas) // 正运动学 由关节角计算末端坐标
{
    Position3 position(cos(thetas.angle[0]) * (LEN_HtoK + cos(thetas.angle[1]) * LEN_KtoA + cos(thetas.angle[1] + thetas.angle[2]) * LEN_AtoF),
                       sin(thetas.angle[0]) * (LEN_HtoK + sin(thetas.angle[1]) * LEN_KtoA + sin(thetas.angle[1] + thetas.angle[2]) * LEN_AtoF),
                       -LEN_KtoA * sin(thetas.angle[1]) - LEN_AtoF * sin(thetas.angle[1] + thetas.angle[2]));

    return position;
}

int a()
{
    return 0;
}

static Theta ikine(Position3 &pos) // 逆运动学 由末端坐标计算关节角
{
    static Position3 pos1;
    static float f1, f2, Lr, alpha_r, alpha1, alpha2, alpha3;
    pos1 = pos;
    f1 = sqrt(pow(pos1.x, 2) + pow(pos1.y, 2));
    f2 = pos1.z;
    Lr = sqrt(pow(f1 - LEN_HtoK, 2) + pow(pos1.z, 2));
    alpha_r = atan2(-pos1.z, f1 - LEN_HtoK);
    alpha1 = atan2(pos1.y, pos1.x);
    alpha2 = acos((pow(Lr, 2) + pow(LEN_KtoA, 2) - pow(LEN_AtoF, 2)) / (2 * Lr * LEN_KtoA)) - atan2(f2, LEN_HtoK - f1);
    alpha3 = acos((pow(Lr, 2) - pow(LEN_KtoA, 2) - pow(LEN_AtoF, 2)) / (2 * LEN_KtoA * LEN_AtoF));
    // Theta thetas(alpha1, alpha2 - alpha_r, -(alpha2 + alpha3));
    Theta thetas(alpha1, alpha2, alpha3 - (PI / 2));
    return thetas;
}

void Move_Ctl::Init()
{
    Rps[0] = fkine(Theta(PI / 4, 0, 0));
    Rps[1] = fkine(Theta(0, 0, 0));
    Rps[2] = fkine(Theta(-PI / 4, 0, 0));
    Rps[3] = fkine(Theta(3 * PI / 4, 0, 0));
    Rps[4] = fkine(Theta(PI, 0, 0));
    Rps[5] = fkine(Theta(5 * PI / 4, 0, 0));
}

Theta Move_Ctl::ikCaculateTest(Position3 &pos)
{
    Theta thetas = ikine(pos);
    return thetas;
}

void debugIK(void *PvParameters)
{
    TCPConfig *Target = (TCPConfig *)PvParameters;

    Move_Ctl moveCtl;
    moveCtl.Init();
    Position3 pos(0, 0, 0);
    Target->TCP.println("[Debug IK]IK Debug Start.");
    Target->TCP.println("[Debug IK]Please input the position of the end point.");
    Target->TCP.println("[Debug IK]Format: x y z");
    while (true)
    {
        if (Target->ReceiveData.length() > 0)
        {
            Target->TCP.println("[Debug IK]Receive data: " + Target->ReceiveData);
            // 判断格式是否正确
            sscanf(Target->ReceiveData.c_str(), "%f %f %f", &pos.x, &pos.y, &pos.z);
            Target->TCP.println("[Debug IK]Position: " + String(pos.x) + " " + String(pos.y) + " " + String(pos.z));
            Theta thetas = moveCtl.ikCaculateTest(pos);
            Target->TCP.println("[Debug IK]Theta: hip=" + String(R2D(thetas.angle[0])) + " knee=" + String(R2D(thetas.angle[1])) + " ankle=" + String(R2D(thetas.angle[2])));
            Target->ReceiveData = "";
        }
        vTaskDelay(1);
    }
}
