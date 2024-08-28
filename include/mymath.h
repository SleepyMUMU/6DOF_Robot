#ifndef MYMATH_H
#define MYMATH_H

#include <Arduino.h>
#include <math.h>
#include <iostream>
#include <cmath>
#include <vector>
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

struct Vector3
{
    double x, y, z;

    Vector3 operator-(const Vector3 &v) const
    {
        return {x - v.x, y - v.y, z - v.z};
    } // 重载减法运算符
    Vector3 operator+(const Vector3 &v) const
    {
        return Vector3{x + v.x, y + v.y, z + v.z};
    }// 重载加法运算符
     Vector3 operator*(double scalar) const {
        return Vector3{x * scalar, y * scalar, z * scalar};
    } // 重载乘法运算符
    Vector3 cross(const Vector3 &other) const
    {
        return Vector3{y * other.z - z * other.y,
                       z * other.x - x * other.z,
                       x * other.y - y * other.x};
    } // 计算叉乘

    double dot(const Vector3 &other) const
    {
        return x * other.x + y * other.y + z * other.z;
    } // 计算点乘
    double length() const
    {
        return sqrt(x * x + y * y + z * z);
    } // 计算向量长度
    Vector3 normalized() const
    {
        double len = length();
        return Vector3{x / len, y / len, z / len};
    } // 计算单位向量
}; // 定义一个三维向量结构体

Vector3 calculateArcCenter(const Vector3 &p1, const Vector3 &p2, const Vector3 &p3);
Vector3 Forward_calculateTrack(const Vector3 &p1, const Vector3 &p3, const Vector3 &centers, int step, int i);
Vector3 Invers_calculateTrack(const Vector3 &p1, const Vector3 &p3, const Vector3 &centers, int step, int i);
#endif // MYMATH_H