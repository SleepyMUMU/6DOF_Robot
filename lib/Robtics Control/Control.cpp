#include "Control.h"

static Position fkine(Theta thetas);
static Theta ikine(Position &pos);

void Move_Ctl::Init()
{
    Rps[0] = fkine(Theta(PI/4, 0, 0));
    Rps[1] = fkine(Theta(0, 0, 0));
    Rps[2] = fkine(Theta(-PI/4, 0, 0));
    Rps[3] = fkine(Theta(3*PI/4, 0, 0));
    Rps[4] = fkine(Theta(PI, 0, 0));
    Rps[5] = fkine(Theta(5*PI/4, 0, 0));
}
static Position fkine(Theta thetas)
{
    Position position(cos(thetas.angle[0]) * (LEN_HtoK + cos(thetas.angle[1]) * LEN_KtoA + cos(thetas.angle[1] + thetas.angle[2]) * LEN_AtoF),
                      sin(thetas.angle[0]) * (LEN_HtoK + sin(thetas.angle[1]) * LEN_KtoA + sin(thetas.angle[1] + thetas.angle[2]) * LEN_AtoF),
                      -LEN_KtoA * sin(thetas.angle[1]) - LEN_AtoF * sin(thetas.angle[1] + thetas.angle[2]));

    return position;
}

static Position ikine(Position &pos)
{
    static Position pos1;
    static float f1, f2, Lr, alpha_r, alpha1, alpha2, alpha3;
    pos1 = pos;
    Lr = sqrt(pow(f1 - LEN_HtoK, 2) + pow(pos1.z, 2));
    f1 = sqrt(pow(pos1.x, 2) + pow(pos1.y, 2));
    f2 = pos1.z;
    alpha_r = atan2( -pos1.z,f1 - LEN_HtoK);
    alpha1 = atan2(pos1.y, pos1.x);
    alpha2 = acos((pow(Lr, 2) + pow(LEN_HtoK, 2) - pow(LEN_AtoF, 2)) / (2 * Lr * LEN_KtoA)) - atan2(f2, LEN_HtoK - f1);
    alpha3 = acos((pow(f1 - LEN_KtoA, 2) + pow(f2, 2) - pow(LEN_KtoA, 2) - pow(LEN_AtoF, 2)) / (2 * LEN_KtoA * LEN_AtoF));
    Theta thetas(alpha1, alpha2-alpha_r,-(alpha2+alpha3));
    return thetas;
}