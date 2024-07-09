#include <mymath.h>

//



void Position3 ::zero()
{
    x = 0;
    y = 0;
    z = 0;
}

Position3 operator+(const Position3 &pos1, const Position3 &pos2)
{
    Position3 pos;
    pos.x = pos1.x + pos2.x;
    pos.y = pos1.y + pos2.y;
    pos.z = pos1.z + pos2.z;
    return pos;
}
Position3 operator-(const Position3 &pos1, const Position3 &pos2)
{
    Position3 pos;
    pos.x = pos1.x - pos2.x;
    pos.y = pos1.y - pos2.y;
    pos.z = pos1.z - pos2.z;
    return pos;
}
Theta operator+(const Theta &theta1, const Theta &theta2)
{
    Theta theta;
    theta.angle[0] = theta1.angle[0] + theta2.angle[0];
    theta.angle[1] = theta1.angle[1] + theta2.angle[1];
    theta.angle[2] = theta1.angle[2] + theta2.angle[2];
    return theta;
}
Theta operator-(const Theta &theta1, const Theta &theta2)
{
    Theta theta;
    theta.angle[0] = theta1.angle[0] - theta2.angle[0];
    theta.angle[1] = theta1.angle[1] - theta2.angle[1];
    theta.angle[2] = theta1.angle[2] - theta2.angle[2];
    return theta;
}

Theta &Theta ::operator=(const float angle[3])
{
    this->angle[0] = angle[0];
    this->angle[1] = angle[1];
    this->angle[2] = angle[2];
    return *this;
}

Theta ::Theta(const float angle[3])
{
    this->angle[0] = angle[0];
    this->angle[1] = angle[1];
    this->angle[2] = angle[2];
}
