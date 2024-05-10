#ifndef MYMATH_H
#define MYMATH_H

#include <cmath>

// PI在Arduino中已经定义为 #define PI 3.1415926535897932384626433832795
// #define PI 3.14159f

class Theta
{
public:
    float angle[3];
    Theta(float hip = 0, float knee = 0, float ankle = 0)
    {
        this->angle[0] = hip;
        this->angle[1] = knee;
        this->angle[2] = ankle;
    }
    Theta &operator=(const float angle[3]);
    Theta(const float angle[3]);
};

Theta operator+(const Theta &theta1, const Theta &theta2);
Theta operator-(const Theta &theta1, const Theta &theta2);

class Position3
{
public:
    float x;
    float y;
    float z;
    Position3(float x = 0, float y = 0, float z = 0)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    void zero();
};

Position3 operator+(const Position3 &pos1, const Position3 &pos2);
Position3 operator-(const Position3 &pos1, const Position3 &pos2);

#endif // MYMATH_H