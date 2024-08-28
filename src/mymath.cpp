#include <mymath.h>

using namespace std; // 使用标准命名空间

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

float multiply_with_precision(float a, float b, int precision)
{
    float multiplied = a * b;
    float power = powf(10, precision);
    return roundf(multiplied * power) / power;
}

// 计算空间中三点的圆弧轨迹圆心坐标
Vector3 calculateArcCenter(const Vector3 &p1, const Vector3 &p2, const Vector3 &p3)
{

    // 计算三个向量
    Vector3 p2p3 = p3 - p2; // a
    Vector3 p1p3 = p3 - p1; // b
    Vector3 p1p2 = p2 - p1; // c

    // 计算向量的模
    double a = p2p3.length();
    double b = p1p3.length();
    double c = p1p2.length();

    // 计算内接三角形半周长
    double l = (a + b + c) / 2;

    // 计算轨迹圆半径
    double r = a * b * c / 4 / sqrt(l * (l - a) * (l - b) * (l - c));

    // 圆弧所在轨迹圆心
    double b1 = a * a * (b * b + c * c - a * a) / double(1000000);
    double b2 = b * b * (a * a + c * c - b * b) / double(1000000);
    double b3 = c * c * (a * a + b * b - c * c) / double(1000000);

    Vector3 a1, a2, a3;
    a1.x = p1.x / double(1000);
    a1.y = p1.y / double(1000);
    a1.z = p1.z / double(1000);
    a2.x = p2.x / double(1000);
    a2.y = p2.y / double(1000);
    a2.z = p2.z / double(1000);
    a3.x = p3.x / double(1000);
    a3.y = p3.y / double(1000);
    a3.z = p3.z / double(1000);

    double P1[3][3] = {{a1.x, a1.y, a1.z}, {a2.x, a2.y, a2.z}, {a3.x, a3.y, a3.z}};
    double P2[3][1] = {{b1}, {b2}, {b3}};
    double P3[3][1] = {{P1[0][0] * P2[0][0] + P1[1][0] * P2[1][0] + P1[2][0] * P2[2][0]},
                       {P1[0][1] * P2[0][0] + P1[1][1] * P2[1][0] + P1[2][1] * P2[2][0]},
                       {P1[0][2] * P2[0][0] + P1[1][2] * P2[1][0] + P1[2][2] * P2[2][0]}};

    Vector3 center;
    center.x = (P3[0][0] / (b1 + b2 + b3)) * double(1000);
    center.y = (P3[1][0] / (b1 + b2 + b3) * double(1000));
    center.z = (P3[2][0] / (b1 + b2 + b3)) * double(1000);

    return center;
}

Vector3 Forward_calculateTrack(const Vector3 &p1, const Vector3 &p3, const Vector3 &centers, int step, int i)
{
    // 计算旋转轴 n 和旋转角度 theta
    Vector3 v_S, v_E, v_Snom, v_enom;
    // 起始向量坐标表示
    v_S.x = p1.x - centers.x;
    v_S.y = p1.y - centers.y;
    v_S.z = p1.z - centers.z;
    // 终点向量坐标表示
    v_E.x = p3.x - centers.x;
    v_E.y = p3.y - centers.y;
    v_E.z = p3.z - centers.z;

    v_Snom = v_S.normalized(); // 起始向量单位化
    v_enom = v_E.normalized(); // 终点向量单位化

    // 旋转轴表示
    Vector3 Rotation_axis;
    Rotation_axis.x = (v_Snom.y * v_enom.z - v_Snom.z * v_enom.y) / sqrt(v_Snom.x * v_Snom.x + v_Snom.y * v_Snom.y + v_Snom.z * v_Snom.z) / sqrt(v_enom.x * v_enom.x + v_enom.y * v_enom.y + v_enom.z * v_enom.z);
    Rotation_axis.y = (v_Snom.z * v_enom.x - v_Snom.x * v_enom.z) / sqrt(v_Snom.x * v_Snom.x + v_Snom.y * v_Snom.y + v_Snom.z * v_Snom.z) / sqrt(v_enom.x * v_enom.x + v_enom.y * v_enom.y + v_enom.z * v_enom.z);
    Rotation_axis.z = (v_Snom.x * v_enom.y - v_Snom.y * v_enom.x) / sqrt(v_Snom.x * v_Snom.x + v_Snom.y * v_Snom.y + v_Snom.z * v_Snom.z) / sqrt(v_enom.x * v_enom.x + v_enom.y * v_enom.y + v_enom.z * v_enom.z);
    // 只有是单位超球上的旋转才是单纯的旋转，否则是旋转加缩放
    Rotation_axis = Rotation_axis.normalized();
    // Serial.println(" ");
    // 计算夹角
    double theta = acos(v_Snom.dot(v_enom)); // 起始向量与终点向量的夹角
    double thera_per = theta / step;         // 每一步的夹角
    double theta_current = thera_per * i;    // 当前步的夹角
    // Serial.println(thera_per,4);
    // Serial.println(theta_current,4);
    // 计算旋转矩阵
    double a = cos(theta_current / 2);
    double b = -Rotation_axis.x * sin(theta_current / 2);
    double c = -Rotation_axis.y * sin(theta_current / 2);
    double d = -Rotation_axis.z * sin(theta_current / 2);

    // 计算旋转矩阵
    double Rotation_matrix[3][3];
    Rotation_matrix[0][0] = a * a + b * b - c * c - d * d;
    Rotation_matrix[0][1] = 2 * (b * c + a * d);
    Rotation_matrix[0][2] = 2 * (b * d - a * c);
    Rotation_matrix[1][0] = 2 * (b * c - a * d);
    Rotation_matrix[1][1] = a * a + c * c - b * b - d * d;
    Rotation_matrix[1][2] = 2 * (c * d + a * b);
    Rotation_matrix[2][0] = 2 * (b * d + a * c);
    Rotation_matrix[2][1] = 2 * (c * d - a * b);
    Rotation_matrix[2][2] = a * a + d * d - b * b - c * c;

    // Serial.print(Rotation_matrix[0][0], 4);Serial.print(" ");Serial.print(Rotation_matrix[0][1], 4);Serial.print(" ");Serial.println(Rotation_matrix[0][2], 4);
    // Serial.print(Rotation_matrix[1][0], 4);Serial.print(" ");Serial.print(Rotation_matrix[1][1], 4);Serial.print(" ");Serial.println(Rotation_matrix[1][2], 4);
    // Serial.print(Rotation_matrix[2][0], 4);Serial.print(" ");Serial.print(Rotation_matrix[2][1], 4);Serial.print(" ");Serial.println(Rotation_matrix[2][2], 4);
    // Serial.println(" ");
    Vector3 v_Current;
    v_Current.x = Rotation_matrix[0][0] * v_S.x + Rotation_matrix[0][1] * v_S.y + Rotation_matrix[0][2] * v_S.z;
    v_Current.y = Rotation_matrix[1][0] * v_S.x + Rotation_matrix[1][1] * v_S.y + Rotation_matrix[1][2] * v_S.z;
    v_Current.z = Rotation_matrix[2][0] * v_S.x + Rotation_matrix[2][1] * v_S.y + Rotation_matrix[2][2] * v_S.z;

    Vector3 nowposition;
    nowposition.x = centers.x + v_Current.x;
    nowposition.y = centers.y + v_Current.y;
    nowposition.z = centers.z + v_Current.z;

    // Serial.println(nowposition.x);
    // Serial.println(nowposition.y);
    // Serial.println(nowposition.z);
    return nowposition;
}

Vector3 Invers_calculateTrack(const Vector3 &p1, const Vector3 &p3, const Vector3 &centers, int step, int i)
{
    // 计算旋转轴 n 和旋转角度 theta
    Vector3 v_S, v_E, v_Snom, v_enom;
    // 起始向量坐标表示
    v_S.x = p1.x - centers.x;
    v_S.y = p1.y - centers.y;
    v_S.z = p1.z - centers.z;
    // 终点向量坐标表示
    v_E.x = p3.x - centers.x;
    v_E.y = p3.y - centers.y;
    v_E.z = p3.z - centers.z;

    v_Snom = v_S.normalized(); // 起始向量单位化
    v_enom = v_E.normalized(); // 终点向量单位化

    // 旋转轴表示
    Vector3 Rotation_axis;
    Rotation_axis.x = (v_Snom.y * v_enom.z - v_Snom.z * v_enom.y) / sqrt(v_Snom.x * v_Snom.x + v_Snom.y * v_Snom.y + v_Snom.z * v_Snom.z) / sqrt(v_enom.x * v_enom.x + v_enom.y * v_enom.y + v_enom.z * v_enom.z);
    Rotation_axis.y = (v_Snom.z * v_enom.x - v_Snom.x * v_enom.z) / sqrt(v_Snom.x * v_Snom.x + v_Snom.y * v_Snom.y + v_Snom.z * v_Snom.z) / sqrt(v_enom.x * v_enom.x + v_enom.y * v_enom.y + v_enom.z * v_enom.z);
    Rotation_axis.z = (v_Snom.x * v_enom.y - v_Snom.y * v_enom.x) / sqrt(v_Snom.x * v_Snom.x + v_Snom.y * v_Snom.y + v_Snom.z * v_Snom.z) / sqrt(v_enom.x * v_enom.x + v_enom.y * v_enom.y + v_enom.z * v_enom.z);
    // 只有是单位超球上的旋转才是单纯的旋转，否则是旋转加缩放
    Rotation_axis = Rotation_axis.normalized();
    // Serial.println(" ");
    // 计算夹角
    double theta = acos(v_Snom.dot(v_enom));      // 起始向量与终点向量的夹角
    double thera_per = theta / step;              // 每一步的夹角
    double theta_current = theta - thera_per * i; // 当前步的夹角
    // Serial.println(thera_per,4);
    // Serial.println(theta_current,4);
    // 计算旋转矩阵
    double a = cos(theta_current / 2);
    double b = -Rotation_axis.x * sin(theta_current / 2);
    double c = -Rotation_axis.y * sin(theta_current / 2);
    double d = -Rotation_axis.z * sin(theta_current / 2);

    // 计算旋转矩阵
    double Rotation_matrix[3][3];
    Rotation_matrix[0][0] = a * a + b * b - c * c - d * d;
    Rotation_matrix[0][1] = 2 * (b * c + a * d);
    Rotation_matrix[0][2] = 2 * (b * d - a * c);
    Rotation_matrix[1][0] = 2 * (b * c - a * d);
    Rotation_matrix[1][1] = a * a + c * c - b * b - d * d;
    Rotation_matrix[1][2] = 2 * (c * d + a * b);
    Rotation_matrix[2][0] = 2 * (b * d + a * c);
    Rotation_matrix[2][1] = 2 * (c * d - a * b);
    Rotation_matrix[2][2] = a * a + d * d - b * b - c * c;

    // Serial.print(Rotation_matrix[0][0], 4);Serial.print(" ");Serial.print(Rotation_matrix[0][1], 4);Serial.print(" ");Serial.println(Rotation_matrix[0][2], 4);
    // Serial.print(Rotation_matrix[1][0], 4);Serial.print(" ");Serial.print(Rotation_matrix[1][1], 4);Serial.print(" ");Serial.println(Rotation_matrix[1][2], 4);
    // Serial.print(Rotation_matrix[2][0], 4);Serial.print(" ");Serial.print(Rotation_matrix[2][1], 4);Serial.print(" ");Serial.println(Rotation_matrix[2][2], 4);
    // Serial.println(" ");
    Vector3 v_Current;
    v_Current.x = Rotation_matrix[0][0] * v_S.x + Rotation_matrix[0][1] * v_S.y + Rotation_matrix[0][2] * v_S.z;
    v_Current.y = Rotation_matrix[1][0] * v_S.x + Rotation_matrix[1][1] * v_S.y + Rotation_matrix[1][2] * v_S.z;
    v_Current.z = Rotation_matrix[2][0] * v_S.x + Rotation_matrix[2][1] * v_S.y + Rotation_matrix[2][2] * v_S.z;

    Vector3 nowposition;
    nowposition.x = centers.x + v_Current.x;
    nowposition.y = centers.y + v_Current.y;
    nowposition.z = centers.z + v_Current.z;

    // Serial.println(nowposition.x);
    // Serial.println(nowposition.y);
    // Serial.println(nowposition.z);
    return nowposition;
}
// float vector_S_x = start.x - center.x;
// float vector_S_y = start.y - center.y;
// float vector_S_z = start.z - center.z;
// // 终点到圆心的向量
// float vector_E_x = end.x - center.x;
// float vector_E_y = end.y - center.y;
// float vector_E_z = end.z - center.z;

// 计算两向量点积
// float dot = vector_S_x * vector_E_x + vector_S_y * vector_E_y + vector_S_z * vector_E_z;
// // 计算两向量的模
// float mod_S = sqrt(vector_S_x * vector_S_x + vector_S_y * vector_S_y + vector_S_z * vector_S_z);
// float mod_E = sqrt(vector_E_x * vector_E_x + vector_E_y * vector_E_y + vector_E_z * vector_E_z);

// // 计算穿过圆心的单位旋转向量
// float vector_R_x = (vector_S_y * vector_E_z - vector_S_z * vector_E_y) / sqrt(vector_S_x * vector_S_x + vector_S_y * vector_S_y + vector_S_z * vector_S_z) / sqrt(vector_E_x * vector_E_x + vector_E_y * vector_E_y + vector_E_z * vector_E_z);
// float vector_R_y = (vector_S_z * vector_E_x - vector_S_x * vector_E_z) / sqrt(vector_S_x * vector_S_x + vector_S_y * vector_S_y + vector_S_z * vector_S_z) / sqrt(vector_E_x * vector_E_x + vector_E_y * vector_E_y + vector_E_z * vector_E_z);
// float vector_R_z = (vector_S_x * vector_E_y - vector_S_y * vector_E_x) / sqrt(vector_S_x * vector_S_x + vector_S_y * vector_S_y + vector_S_z * vector_S_z) / sqrt(vector_E_x * vector_E_x + vector_E_y * vector_E_y + vector_E_z * vector_E_z);

// // 计算夹角
// std::vector<Vector3> calculateTrack(const Vector3 &p1, const Vector3 &p2, const Vector3 &p3, uint8_t step, uint8_t i)
// {
//     std::vector<Vector3> centers = calculateArcCenter(p1, p2, p3);

//     // 计算旋转轴 n 和旋转角度 theta
//     Vector3 v1 = p1 - centers;

//     Vector3 n = calculateArcCenter()

//         float vector_S_x = start.x - center.x;
//     float vector_S_y = start.y - center.y;
//     float vector_S_z = start.z - center.z;
//     // 终点到圆心的向量
//     float vector_E_x = end.x - center.x;
//     float vector_E_y = end.y - center.y;
//     float vector_E_z = end.z - center.z;

//     // 计算两向量点积
//     float dot = vector_S_x * vector_E_x + vector_S_y * vector_E_y + vector_S_z * vector_E_z;
//     // 计算两向量的模
//     float mod_S = sqrt(vector_S_x * vector_S_x + vector_S_y * vector_S_y + vector_S_z * vector_S_z);
//     float mod_E = sqrt(vector_E_x * vector_E_x + vector_E_y * vector_E_y + vector_E_z * vector_E_z);

//     // 计算穿过圆心的单位旋转向量
//     float vector_R_x = (vector_S_y * vector_E_z - vector_S_z * vector_E_y) / sqrt(vector_S_x * vector_S_x + vector_S_y * vector_S_y + vector_S_z * vector_S_z) / sqrt(vector_E_x * vector_E_x + vector_E_y * vector_E_y + vector_E_z * vector_E_z);
//     float vector_R_y = (vector_S_z * vector_E_x - vector_S_x * vector_E_z) / sqrt(vector_S_x * vector_S_x + vector_S_y * vector_S_y + vector_S_z * vector_S_z) / sqrt(vector_E_x * vector_E_x + vector_E_y * vector_E_y + vector_E_z * vector_E_z);
//     float vector_R_z = (vector_S_x * vector_E_y - vector_S_y * vector_E_x) / sqrt(vector_S_x * vector_S_x + vector_S_y * vector_S_y + vector_S_z * vector_S_z) / sqrt(vector_E_x * vector_E_x + vector_E_y * vector_E_y + vector_E_z * vector_E_z);

//     // 计算夹角
//     float angle = acos(dot / mod_S / mod_E);

//     // 拆分夹角
//     float angle_i = angle / step * t;
//     // 绕旋转向量旋转
//     float Rotation[3][3] = {
//         {cos(angle_i) + vector_R_x * vector_R_x * (1 - cos(angle_i)), vector_R_x * vector_R_y * (1 - cos(angle_i)) - vector_R_z * sin(angle_i), vector_R_x * vector_R_z * (1 - cos(angle_i)) + vector_R_y * sin(angle_i)},
//         {vector_R_y * vector_R_x * (1 - cos(angle_i)) + vector_R_z * sin(angle_i), cos(angle_i) + vector_R_y * vector_R_y * (1 - cos(angle_i)), vector_R_y * vector_R_z * (1 - cos(angle_i)) - vector_R_x * sin(angle_i)},
//         {vector_R_z * vector_R_x * (1 - cos(angle_i)) - vector_R_y * sin(angle_i), vector_R_z * vector_R_y * (1 - cos(angle_i)) + vector_R_x * sin(angle_i), cos(angle_i) + vector_R_z * vector_R_z * (1 - cos(angle_i))}};
//     // 计算旋转后的向量
//     float vector_T_x = Rotation[0][0] * vector_S_x + Rotation[0][1] * vector_S_y + Rotation[0][2] * vector_S_z;
//     float vector_T_y = Rotation[1][0] * vector_S_x + Rotation[1][1] * vector_S_y + Rotation[1][2] * vector_S_z;
//     float vector_T_z = Rotation[2][0] * vector_S_x + Rotation[2][1] * vector_S_y + Rotation[2][2] * vector_S_z;

//     // 计算旋转后的点
//     nowposition.x = center.x + vector_T_x;
//     nowposition.y = center.y + vector_T_y;
//     nowposition.z = center.z + vector_T_z;
// }

// int main()
// {
//     Point3D p1 = {1.0, 2.0, 3.0};
//     Point3D p2 = {4.0, 5.0, 6.0};
//     Point3D p3 = {7.0, 8.0, 9.0};

//     std::vector<double> center = calculateArcCenter(p1, p2, p3);

//     std::cout << "Arc center: (" << center[0] << ", " << center[1] << ", " << center[2] << ")" << std::endl;

//     return 0;
// }