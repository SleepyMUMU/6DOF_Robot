#include <math.h>

void Circular_track(float p_start[3], float p_mid[3], float p_final[3], int step, float *center, float *rotation_axis, float *theta_per)
{
    // 计算所取圆弧在轨迹圆的半径
    float a = sqrt(pow(p_final[0] - p_mid[0], 2) + pow(p_final[1] - p_mid[1], 2) + pow(p_final[2] - p_mid[2], 2));       // 内接三角形边长a
    float b = sqrt(pow(p_final[0] - p_start[0], 2) + pow(p_final[1] - p_start[1], 2) + pow(p_final[2] - p_start[2], 2)); // 内接三角形边长b
    float c = sqrt(pow(p_mid[0] - p_start[0], 2) + pow(p_mid[1] - p_start[1], 2) + pow(p_mid[2] - p_start[2], 2));       // 内接三角形边长c
    float l = (a + b + c) / 2;                                                                                           // 内接三角形半周长
    float r = a * b * c / 4 / sqrt(l * (l - a) * (l - b) * (l - c));                                                     // 轨迹圆半径

    // 求所取圆弧所在平面参数
    float solution[3];
    float matrix[9] = {
        p_start[0], p_start[1], p_start[2],
        p_mid[0], p_mid[1], p_mid[2],
        p_final[0], p_final[1], p_final[2]};
    float rhs[3] = {1, 1, 1};

    // 求解 Ax = b 得到 solutionm,Cramer
    float det = matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7]) -
                 matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6]) +
                 matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);
    solution[0] = (rhs[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7]) -
                   rhs[1] * (matrix[1] * matrix[8] - matrix[2] * matrix[7]) +
                   rhs[2] * (matrix[1] * matrix[5] - matrix[2] * matrix[4])) /
                  det;
    solution[1] = (matrix[0] * rhs[1] - matrix[1] * rhs[0] + matrix[2] * (rhs[0] * matrix[7] - rhs[1] * matrix[6])) / det;
    solution[2] = (matrix[0] * (rhs[2] * matrix[4] - rhs[1] * matrix[5]) -
                   matrix[1] * (rhs[2] * matrix[3] - rhs[0] * matrix[5]) +
                   matrix[2] * (rhs[1] * matrix[3] - rhs[0] * matrix[4])) /
                  det;

    // 求取圆弧所在轨迹圆圆心
    float b1 = a * (b + c - a);
    float b2 = b * (a + c - b);
    float b3 = c * (a + b - c);
    center[0] = (p_start[0] * b1 + p_mid[0] * b2 + p_final[0] * b3) / (b1 + b2 + b3);
    center[1] = (p_start[1] * b1 + p_mid[1] * b2 + p_final[1] * b3) / (b1 + b2 + b3);
    center[2] = (p_start[2] * b1 + p_mid[2] * b2 + p_final[2] * b3) / (b1 + b2 + b3);

    // 求旋转轴
    float vector_start[3] = {p_start[0] - center[0], p_start[1] - center[1], p_start[2] - center[2]};
    float vector_final[3] = {p_final[0] - center[0], p_final[1] - center[1], p_final[2] - center[2]};
    rotation_axis[0] = vector_start[1] * vector_final[2] - vector_start[2] * vector_final[1];
    rotation_axis[1] = vector_start[2] * vector_final[0] - vector_start[0] * vector_final[2];
    rotation_axis[2] = vector_start[0] * vector_final[1] - vector_start[1] * vector_final[0];

    // 计算圆弧角度
    float dot_product = (vector_start[0] * vector_final[0] + vector_start[1] * vector_final[1] + vector_start[2] * vector_final[2]) /
                         (sqrt(vector_start[0] * vector_start[0] + vector_start[1] * vector_start[1] + vector_start[2] * vector_start[2]) *
                          sqrt(vector_final[0] * vector_final[0] + vector_final[1] * vector_final[1] + vector_final[2] * vector_final[2]));
    *theta_per = acos(dot_product) / step;              
}
// 定义旋转矩阵函数
float rotation_matrix(float axis[3], float theta)
{
    axis[0] = axis[0] / sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    axis[1] = axis[1] / sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    axis[2] = axis[2] / sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    theta = theta;
    float a = cos(theta / 2);
    float bcd[3] = {axis[0] * sin(theta / 2), axis[1] * sin(theta / 2), axis[2] * sin(theta / 2)};
    float b = bcd[0];
    float c = bcd[1];
    float d = bcd[2];
    float aa = pow(a, 2);
    float bb = pow(b, 2);
    float cc = pow(c, 2);
    float dd = pow(d, 2);
    float ab = a * b;
    float ac = a * c;
    float ad = a * d;
    float bc = b * c;
    float bd = b * d;
    float cd = c * d;

    float r_matrix[3][3] =
        {
            {aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)},
            {2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)},
            {2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc}};

    return r_matrix[3][3];
}
