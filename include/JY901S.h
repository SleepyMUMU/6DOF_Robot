#ifndef _JY901S_H_
#define _JY901S_H_
#include <Wire.h> // Include the Wire library for I2C
#include <wit_c_sdk.h>
#include <Arduino.h>
#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80

extern char cmd[10];
extern const uint32_t c_uiBaud[8];
extern float fAcc[3], fGyro[3], fAngle[3];


#define default_Speend 400000
class JY901S
{

public:
    JY901S()
    {
        this->IIC_Speend = default_Speend;
    }
    JY901S(uint32_t Speend)
    {
        this->IIC_Speend = Speend;
    }
    ~JY901S()
    {
    }

    void JY901S_Init();
    float GetAccX();

private:
    uint32_t IIC_Speend;    // IIC速度
    WitI2cWrite write_func; // IIC写函数
    WitI2cRead read_func;   // IIC读函数
};

extern JY901S gyroscope;
void GetAcc_task(void *pvParameters);
#endif