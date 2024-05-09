#ifndef MYMATH_H
#define MYMATH_H

#define  PI 3.14159f
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

class Position
{
public:
    float x;
    float y;
    float z;
    Position(float x = 0, float y = 0, float z = 0)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    void zero();
};

Position operator+(const Position &pos1, const Position &pos2);
Position operator-(const Position &pos1, const Position &pos2);

#endif // MYMATH_H