#include "Servo.h"
#include "Robot.h"
#include "Track.h"
FSUS_SERVO_ANGLE_T defaultAngleArray[7][3] = {
    {0, 0, 0},
    {defaultLeg1HipAngle, defaultLeg1KneeAngle, defaultLeg1AnkleAngle},
    {defaultLeg2HipAngle, defaultLeg2KneeAngle, defaultLeg2AnkleAngle},
    {defaultLeg3HipAngle, defaultLeg3KneeAngle, defaultLeg3AnkleAngle},
    {defaultLeg4HipAngle, defaultLeg4KneeAngle, defaultLeg4AnkleAngle},
    {defaultLeg5HipAngle, defaultLeg5KneeAngle, defaultLeg5AnkleAngle},
    {defaultLeg6HipAngle, defaultLeg6KneeAngle, defaultLeg6AnkleAngle}};

u8_t defaultLegServoSerial[7][3] = {
    {0, 0, 0},
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9},
    {10, 11, 12},
    {13, 14, 15},
    {16, 17, 18}};

float debugAngle[5][3] = {

    {288.5, 0, 36.75},
    {136.4, 78.75, -140.8},
    {157.5, 0, -140.8},
};
#define PosDownSer 0

float left_position[3][3] = {
    {235.5, -136, 15.53},    // 0
    {78.75, -136.4, -140.8}, // 1
    {157.5, 0, -140.8}       // 2
};
float right_position[3][3] = {
    {235.5, 136, 15.53},    // 0
    {78.75, 136.4, -140.8}, // 1
    {157.5, 0, -140.8}      // 2
};
float midleft_posistion[3][3] = {
    {276.8, 0, 51.97}, // 0
    {279.4, 0, -70.4}, // 1
    {50.35, 0, -58.28} // 2
};
float midright_position[3][3] = {
    {279.4, 0, -70.4}, // 0
    {276.8, 0, 51.97}, // 1
    {50.35, 0, -58.28} // 2
};
float moveforward_position[3][3] = {
    {233.5, 134.8, -33.65}, // 0
    {111.4, 111.4, -140.8}, // 1
    {157.5, 0, -140.8}      // 2
};

float movebackward_position[3][3] = {
    {233.5, -134.8, -33.65}, // 0
    {111.4, -111.4, -140.8}, // 1
    {157.5, 0, -140.8}       // 2
};
// 螃蟹步
float Front_left_position[3][3] = {
    {195.7, -195.7, 51.97}, // 0
    {182.8, -182.8, -71.5}, // 1
    {46.37, -46.37, -69.96} // 2
};
float Front_right_position[3][3] = {
    {195.7, -195.7, 51.97},  // 0
    {46.37, -46.37, -69.96}, // 1
    {182.8, -182.8, -71.5}   // 2
};
float Mid_left_position[3][3] = {
    {276.8, 0, 51.97},  // 0
    {218.1, 0, -158.7}, // 1
    {65.57, 0, -69.96}  // 2
};
float Mid_right_position[3][3] = {
    {276.8, 0, 51.97},  // 0
    {65.57, 0, -69.96}, // 1
    {218.1, 0, -158.7}  // 2
};
float Back_left_position[3][3] = {
    {195.7, 195.7, 51.97}, // 0
    {182.8, 182.8, -71.5}, // 1
    {46.37, 46.37, -69.96} // 2
};
float Back_right_position[3][3] = {
    {195.7, 195.7, 51.97},  // 0
    {46.37, 46.37, -69.96}, // 1
    {182.8, 182.8, -71.5}   // 2
};

u8_t AddedNumofLeg = 0;
QueueHandle_t LegQueue[numofLeg]; // 腿部队列
LegConfig::LegConfig()
{
}
LegConfig::LegConfig(FSUS_Protocol INputPol, u8_t LegSer)
{
    this->legSer = LegSer;                                           // 传入机械臂序号
    this->hipServoID = defaultLegServoSerial[LegSer][0];             // 传入Hip髋关节舵机ID1
    this->kneeServoID = defaultLegServoSerial[LegSer][1];            // 传入Knee膝关节舵机ID2
    this->ankleServoID = defaultLegServoSerial[LegSer][2];           // 传入Ankle踝关节舵机ID3
    this->defaultHipAngle = defaultAngleArray[LegSer][0];            // 传入默认Hip髋关节角度
    this->defaultKneeAngle = defaultAngleArray[LegSer][1];           // 传入默认Knee膝关节角度
    this->defaultAnkleAngle = defaultAngleArray[LegSer][2];          // 传入默认Ankle踝关节角度
    this->protocol = INputPol;                                       // 初始化舵机串口通信协议
    this->hipServo.init(this->hipServoID, &this->protocol);          // 初始化hip髋关节舵机
    this->kneeServo.init(this->kneeServoID, &this->protocol);        // 初始化knee膝关节舵机
    this->ankleServo.init(this->ankleServoID, &this->protocol);      // 初始化ankle踝关节舵机
    LegQueue[AddedNumofLeg] = xQueueCreate(1, sizeof(LegConfig *));  // 创建一个消息队列
    LegConfig *LegPointer = this;                                    // 将LegConfig对象的指针传递给消息队列
    xQueueSend(LegQueue[AddedNumofLeg], &LegPointer, portMAX_DELAY); // 发送消息队列
    AddedNumofLeg++;
}
LegConfig::~LegConfig()
{
}

LegConfig::LegConfig(FSUS_Protocol INputPol, uint8_t hipServoID, uint8_t kneeServoID, uint8_t ankleServoID)
{
    this->protocol = INputPol; // 初始化舵机串口通信协议

    this->LegServoID[0] = hipServoID;   // 传入Hip髋关节舵机ID
    this->LegServoID[1] = kneeServoID;  // 传入Knee膝关节舵机ID
    this->LegServoID[2] = ankleServoID; // 传入Ankle踝关节舵机ID

    this->LegServo[0].init(this->LegServoID[0], &this->protocol); // 初始化hip髋关节舵机
    this->LegServo[1].init(this->LegServoID[1], &this->protocol); // 初始化knee膝关节舵机
    this->LegServo[2].init(this->LegServoID[2], &this->protocol); // 初始化ankle踝关节舵机
}
void LegConfig::LegInit(FSUS_Protocol INputPol, uint8_t hipServoID, uint8_t kneeServoID, uint8_t ankleServoID)
{
    this->hipServoID = hipServoID;     // 传入Hip髋关节舵机ID1
    this->kneeServoID = kneeServoID;   // 传入Knee膝关节舵机ID2
    this->ankleServoID = ankleServoID; // 传入Ankle踝关节舵机ID3
    // this->LegName = String(defaultLegName + String("#") + int(AddedNumofLeg)); // 传入LegName
    this->protocol = INputPol;                                       // 初始化舵机串口通信协议
    this->hipServo.init(this->hipServoID, &this->protocol);          // 初始化hip髋关节舵机
    this->kneeServo.init(this->kneeServoID, &this->protocol);        // 初始化knee膝关节舵机
    this->ankleServo.init(this->ankleServoID, &this->protocol);      // 初始化ankle踝关节舵机
    LegQueue[AddedNumofLeg] = xQueueCreate(1, sizeof(LegConfig *));  // 创建一个消息队列
    LegConfig *LegPointer = this;                                    // 将LegConfig对象的指针传递给消息队列
    xQueueSend(LegQueue[AddedNumofLeg], &LegPointer, portMAX_DELAY); // 发送消息队列
    AddedNumofLeg++;                                                 // 增加机械臂数量
}
void LegConfig::LegPowerDown()
{
    this->hipServo.setTorque(false);   // hip髋关节舵机关闭阻尼
    this->kneeServo.setTorque(false);  // knee膝关节舵机关闭阻尼
    this->ankleServo.setTorque(false); // ankle踝关节舵机关闭阻尼
}
void LegConfig::LegInit()
{
    LegSetAngle(defaultHipAngle, defaultKneeAngle, defaultAnkleAngle, defaultRunTime);
    delay(defaultRunTime);
}
void LegConfig::LegInit(FSUS_Protocol INputPol, u8_t LegSer)
{
    this->legSer = LegSer;                                           // 传入机械臂序号
    this->hipServoID = defaultLegServoSerial[LegSer][0];             // 传入Hip髋关节舵机ID1
    this->kneeServoID = defaultLegServoSerial[LegSer][1];            // 传入Knee膝关节舵机ID2
    this->ankleServoID = defaultLegServoSerial[LegSer][2];           // 传入Ankle踝关节舵机ID3
    this->defaultHipAngle = defaultAngleArray[LegSer][0];            // 传入默认Hip髋关节角度
    this->defaultKneeAngle = defaultAngleArray[LegSer][1];           // 传入默认Knee膝关节角度
    this->defaultAnkleAngle = defaultAngleArray[LegSer][2];          // 传入默认Ankle踝关节角度
    this->protocol = INputPol;                                       // 初始化舵机串口通信协议
    this->hipServo.init(this->hipServoID, &this->protocol);          // 初始化hip髋关节舵机
    this->kneeServo.init(this->kneeServoID, &this->protocol);        // 初始化knee膝关节舵机
    this->ankleServo.init(this->ankleServoID, &this->protocol);      // 初始化ankle踝关节舵机
    LegQueue[AddedNumofLeg] = xQueueCreate(1, sizeof(LegConfig *));  // 创建一个消息队列
    LegConfig *LegPointer = this;                                    // 将LegConfig对象的指针传递给消息队列
    xQueueSend(LegQueue[AddedNumofLeg], &LegPointer, portMAX_DELAY); // 发送消息队列
    AddedNumofLeg++;
    LegSetAngle(defaultHipAngle, defaultKneeAngle, defaultAnkleAngle, defaultRunTime);
}
void LegConfig::LegSetAngle(FSUS_SERVO_ANGLE_T hipAngle, FSUS_SERVO_ANGLE_T kneeAngle, FSUS_SERVO_ANGLE_T ankleAngle, FSUS_INTERVAL_T runTime)
{
    this->hipServo.setAngle(hipAngle, runTime);     // 移动hip髋关节舵机
    this->kneeServo.setAngle(kneeAngle, runTime);   // 移动knee膝关节舵机
    this->ankleServo.setAngle(ankleAngle, runTime); // 移动ankle踝关节舵机
}

void LegConfig::LegSetHipAngle(FSUS_SERVO_ANGLE_T targetangle, FSUS_INTERVAL_T runTime)
{
    this->hipServo.setAngle(targetangle, runTime); // 移动hip髋关节舵机
    delay(runTime);                                // 延时
}

void LegConfig::LegSetKneeAngle(FSUS_SERVO_ANGLE_T targetangle, FSUS_INTERVAL_T runTime)
{
    this->kneeServo.setAngle(targetangle, runTime); // 移动knee膝关节舵机
    delay(runTime);                                 // 延时
}

void LegConfig::LegSetAnkleAngle(FSUS_SERVO_ANGLE_T targetangle, FSUS_INTERVAL_T runTime)
{
    this->ankleServo.setAngle(targetangle, runTime); // 移动ankle踝关节舵机
    delay(runTime);                                  // 延时
}
// void LegConfig::LegSetAngle(FSUS_SERVO_ANGLE_T targetangle, FSUS_INTERVAL_T runTime)
// {
//     this->hipServo.setAngle(targetangle, runTime);   // 移动hip髋关节舵机
//     this->kneeServo.setAngle(targetangle, runTime);  // 移动knee膝关节舵机
//     this->ankleServo.setAngle(targetangle, runTime); // 移动ankle踝关节舵机
//     delay(runTime);                                  // 延时
// }

uint8_t LegConfig::LegPing()
{
    // 因为ping返回的是bool值，所以我希望将这三个bool值以二进制的形式返回给调用者，以便调用者知道ping的结果
    return ThreeBool2Bin(this->hipServo.ping(), this->kneeServo.ping(), this->ankleServo.ping());
}

uint8_t LegConfig::ThreeBool2Bin(bool hipServo, bool kneeServo, bool ankleServo) // 三个bool值转换为一个二进制值
{
    return (hipServo << 2) | (kneeServo << 1) | ankleServo;
}

// bin2ThreeBool
void LegConfig::bin2ThreeBool(uint8_t bin, bool &hipServo, bool &kneeServo, bool &ankleServo)
{
    hipServo = (bin >> 2) & 0x01;
    kneeServo = (bin >> 1) & 0x01;
    ankleServo = bin & 0x01;
}

void LegConfig::fkine(FSUS_SERVO_ANGLE_T hipAngle, FSUS_SERVO_ANGLE_T kneeAngle, FSUS_SERVO_ANGLE_T ankleAngle, float &x, float &y, float &z)
{

    // 弧度制
    float hipAngleRad = hipAngle * PI / 180;
    float kneeAngleRad = kneeAngle * PI / 180;
    float ankleAngleRad = ankleAngle * PI / 180;

    // 机械臂的正运动学解算
    x = L1 * cos(hipAngleRad) + L2 * cos(kneeAngleRad) * cos(hipAngleRad) + L3 * cos(kneeAngleRad + ankleAngleRad) * cos(hipAngleRad);
    y = L1 * sin(hipAngleRad) + L2 * sin(kneeAngleRad) * sin(hipAngleRad) + L3 * sin(kneeAngleRad + ankleAngleRad) * sin(hipAngleRad);
    z = -L3 * sin(kneeAngleRad + ankleAngleRad) - L2 * sin(kneeAngleRad);
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
void LegConfig::SetDampMode()
{
    this->hipServo.setDamping(defaultPower);
    this->kneeServo.setDamping(defaultPower);
    this->ankleServo.setDamping(defaultPower);
}
void LegConfig::SetDampMode(FSUS_POWER_T Power)
{
    this->hipServo.setDamping(Power);
    this->kneeServo.setDamping(Power);
    this->ankleServo.setDamping(Power);
}
void LegConfig::ikine(Position3 &pos)
{
    float f1, f2, Lr, alpha_r, alpha1, alpha2, alpha3;
    f1 = sqrt(pow(pos.x, 2) + pow(pos.y, 2));
    f2 = pos.z;
    Lr = sqrt(pow(f1 - LEN_HtoK, 2) + pow(pos.z, 2));
    alpha_r = atan2(-pos.z, f1 - LEN_HtoK);
    alpha1 = atan2(pos.y, pos.x);
    alpha2 = acos((pow(Lr, 2) + pow(LEN_KtoA, 2) - pow(LEN_AtoF, 2)) / (2 * Lr * LEN_KtoA)) - atan2(f2, LEN_HtoK - f1);
    alpha3 = acos((pow(Lr, 2) - pow(LEN_KtoA, 2) - pow(LEN_AtoF, 2)) / (2 * LEN_KtoA * LEN_AtoF));
    this->hipAngle = R2D(alpha1);
    this->kneeAngle = R2D(alpha2);
    this->ankleAngle = R2D(alpha3 - (PI / 2));
}
/*运动学逆解*/
void LegConfig::ikine(float x, float y, float z)
{

    float f1, f2, Lr, alpha_r, alpha1, alpha2, alpha3;
    f1 = sqrt(pow(x, 2) + pow(y, 2));
    f2 = z;
    Lr = sqrt(pow(f1 - LEN_HtoK, 2) + pow(z, 2));
    alpha_r = atan2(-z, f1 - LEN_HtoK);
    alpha1 = atan2(y, x);
    alpha2 = acos((pow(Lr, 2) + pow(LEN_KtoA, 2) - pow(LEN_AtoF, 2)) / (2 * Lr * LEN_KtoA)) - atan2(f2, LEN_HtoK - f1);
    alpha3 = acos((pow(Lr, 2) - pow(LEN_KtoA, 2) - pow(LEN_AtoF, 2)) / (2 * LEN_KtoA * LEN_AtoF));
    if (z > 0)
    {
        alpha2 = alpha2 + PI;
        this->kneeAngle = R2D(alpha2);
    }
    else if (z < 0)
    {
        alpha2 = alpha2 - PI;
        this->kneeAngle = R2D(alpha2);
    }
    else
    {
        this->kneeAngle = R2D(alpha2);
    }
    this->hipAngle = R2D(alpha1);
    // this->kneeAngle = R2D(alpha2);
    this->ankleAngle = -R2D(alpha3 - PI / 2);
}

void LegConfig::LegMoving(float x, float y, float z, FSUS_INTERVAL_T intertval)
{
    ikine(x, y, z);
    hipServo.setAngle(this->hipAngle + defaultHipAngle, intertval);
    kneeServo.setAngle(this->kneeAngle + defaultKneeAngle, intertval);
    ankleServo.setAngle(-this->ankleAngle + defaultAnkleAngle, intertval);
}
void LegConfig::LegMoving()
{
    int i;                  // Declare the variable "i"
    for (i = 0; i < 3; i++) // Fix the for loop condition
    {
        ikine(debugAngle[i][0], debugAngle[i][1], debugAngle[i][2]);
        hipServo.setAngle(this->hipAngle + defaultHipAngle, servoDefaultTime);
        kneeServo.setAngle(this->kneeAngle + defaultKneeAngle, servoDefaultTime);
        ankleServo.setAngle(-this->ankleAngle + defaultAnkleAngle, servoDefaultTime);
        delay(2000);
    }
}
void LegConfig::LegMoving(float x, float y, float z)
{

    ikine(x, y, z);
    hipServo.setAngle(this->hipAngle + defaultHipAngle, servoDefaultTime);
    kneeServo.setAngle(this->kneeAngle + defaultKneeAngle, servoDefaultTime);
    ankleServo.setAngle(-this->ankleAngle + defaultAnkleAngle, servoDefaultTime);
}
void LegSetAngle_task(void *pvParameters)
{
    TCPConfig *Target = (TCPConfig *)pvParameters; // 接收对应TCPConfig对象
    Target->TCP.println("[LegSetAngle]Please enter the Serial Number of the Leg you want to control.");
    while (1)
    {
        if (Target->ReceiveData != "")
        {
            Target->TCP.printf("[LegSetAngle]The Serial Number of the Leg you want to control is %s.\n", Target->ReceiveData.c_str());
            int LegNum = Target->ReceiveData.toInt() - 1;
            Target->ReceiveData = "";
            if (LegNum < AddedNumofLeg)
            {
                LegConfig *TargetLeg;
                xQueuePeek(LegQueue[LegNum], &TargetLeg, portMAX_DELAY);
                Target->TCP.println("[LegSetAngle]Please enter the Hip,Knee,Ankle of the Leg you want to control.");
                while (1)
                {
                    if (Target->ReceiveData != "")
                    {
                        float Hip, Knee, Ankle;
                        sscanf(Target->ReceiveData.c_str(), "%f %f %f", &Hip, &Knee, &Ankle);
                        Target->TCP.printf("[LegSetAngle]The Hip,Knee,Ankle of the Leg you want to control is %f,%f,%f.\n", Hip, Knee, Ankle);
                        TargetLeg->LegSetAngle(Hip, Knee, Ankle, defaultRunTime);
                        Target->TCP.println("[LegSetAngle]The Leg is moving.");
                        Target->ReceiveData = "";
                        Target->Terminal_TaskHandle = NULL;
                        Target->truncateStream = false;
                        vTaskDelete(NULL);
                    }
                    vTaskDelay(1);
                }
            }
            else
            {
                Target->TCP.println("[LegSetAngle]The Serial Number is out of range.");
                Target->TCP.println("[LegSetAngle]Please enter the Serial Number of the Leg you want to control.");
            }
            Target->ReceiveData = "";
        }
        vTaskDelay(1);
    }
    vTaskDelete(NULL);
}
void LegCrtl_Task(void *pvParameters)
{
    bool LegCrtlFlag = true;
    TCPConfig *Target = (TCPConfig *)pvParameters; // 接收对应TCPConfig对象
    Target->TCP.println("[LegCrtl]Please enter the Serial Number of the Leg you want to control.");
    while (LegCrtlFlag)
    {
        if (Target->ReceiveData != "")
        {
            Target->TCP.printf("[LegCrtl]The Serial Number of the Leg you want to control is %s.\n", Target->ReceiveData.c_str());
            int LegNum = Target->ReceiveData.toInt() - 1;
            Target->ReceiveData = "";
            if (LegNum < AddedNumofLeg)
            {
                LegConfig *TargetLeg;
                Target->TCP.println("[LegCrtl]Loading the date of the leg...");
                xQueuePeek(LegQueue[LegNum], &TargetLeg, portMAX_DELAY);
                Target->TCP.println("[LegCrtl]Please enter the x,y,z of the Leg you want to control.");

                while (LegCrtlFlag)
                {
                    if (Target->ReceiveData != "")
                    {
                        float x, y, z;
                        sscanf(Target->ReceiveData.c_str(), "%f %f %f", &x, &y, &z);
                        Target->TCP.printf("[LegCrtl]The x,y,z of the Leg you want to control is %f,%f,%f.\n", x, y, z);
                        TargetLeg->LegMoving(x, y, z);
                        Target->TCP.printf("[LegCrtl]The Leg is moving to Angle:%f,%f,%f.\n", TargetLeg->hipAngle, TargetLeg->kneeAngle, TargetLeg->ankleAngle);
                        Target->TCP.println("");
                        Target->ReceiveData = "";
                        LegCrtlFlag = false;
                        Target->Terminal_TaskHandle = NULL;
                        Target->truncateStream = false;
                        break;
                    }
                    vTaskDelay(1);
                }
            }
            else
            {
                Target->TCP.println("[LegCrtl]The Serial Number is out of range.");
                Target->TCP.println("[LegCrtl]Please enter the Serial Number of the Leg you want to control.");
            }
            Target->ReceiveData = "";
        }
        vTaskDelay(1);
    }
    vTaskDelete(NULL);
}
void LegPing_Task(void *pvParameters)
{
    LegConfig *Target = (LegConfig *)pvParameters; // 接收对应LegConfig对象
    while (1)
    {
        uint8_t pingResult = Target->LegPing();
        bool hipServo, kneeServo, ankleServo;
        Target->bin2ThreeBool(pingResult, hipServo, kneeServo, ankleServo);
        if (hipServo)
        {
            DebugSerial.println("Hip Servo is online");
        }
        else
        {
            DebugSerial.println("Hip Servo is offline");
        }

        if (kneeServo)
        {
            DebugSerial.println("Knee Servo is online");
        }
        else
        {
            DebugSerial.println("Knee Servo is offline");
        }

        if (ankleServo)
        {
            DebugSerial.println("Ankle Servo is online");
        }
        else
        {
            DebugSerial.println("Ankle Servo is offline");
        }

        if (hipServo && kneeServo && ankleServo)
        {
            Target->LegSetHipAngle(defaultLeg1HipAngle, 2);
            Target->LegSetKneeAngle(defaultLeg1KneeAngle, 2);
            Target->LegSetAnkleAngle(defaultLeg1AnkleAngle, 2);
        }
        else
        {
            DebugSerial.println("Some Servo is offline");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
float *center;
float *rotation_axis;
float *theta_per;
float Start[3] = {127.4, -92.58, -140.8};
float Mid[3] = {127.4, 0, -60.8};
float End[3] = {127.4, 92.58, -140.8};
float matrix_current[3][3];

void straight()
{

    int16_t DSD = 200; // 每点间隔
    float T = 2;       // 周期
    float step = 200;  // 步长
    float H = 100;     // 步高
    float t = 0;
    float Position_Swing[3];
    float Position_Support[3];

    while (t <= T)
    {

        if (t <= T / 2)
        {
            Position_Swing[0] = 127.4;
            Position_Swing[1] = 6 * step * pow(t, 5) - 15 * step * pow(t, 4) + 10 * step * pow(t, 3) - step / 2;
            Position_Swing[2] = -64 * H * pow(t, 6) + 192 * H * pow(t, 5) - 192 * H * pow(t, 4) + 64 * H * pow(t, 3) - 140.8;

            Position_Support[0] = 127.4;
            Position_Support[1] = -6 * step * pow(t + 1, 5) + 45 * step * pow(t + 1, 4) - 130 * step * pow(t + 1, 3) + 180 * step * pow(t + 1, 2) - 120 * step * (t + 1) + 63 * step / 2;
            Position_Support[2] = -140.8;
        }
        else if (t <= T && t > T / 2)
        {
            Position_Support[0] = 127.4;
            Position_Support[1] = 6 * step * pow(t - 1, 5) - 15 * step * pow(t - 1, 4) + 10 * step * pow(t - 1, 3) - step / 2;
            Position_Support[2] = -64 * H * pow(t - 1, 6) + 192 * H * pow(t - 1, 5) - 192 * H * pow(t - 1, 4) + 64 * H * pow(t - 1, 3) - 140.8;

            Position_Swing[0] = 127.4;
            Position_Swing[1] = -6 * step * pow(t, 5) + 45 * step * pow(t, 4) - 130 * step * pow(t, 3) + 180 * step * pow(t, 2) - 120 * step * t + 63 * step / 2;
            Position_Swing[2] = -140.8;
        }
        else
        {
        }; // do nothing
        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
                break;

            case 1:
                TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
                // case 1:
                //     TargetLeg->LegMoving(x2, y2, z2);
                break;
            case 2:
                TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
                break;
            case 3:
                TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
                break;
            // case 3:
            //     TargetLeg->LegMoving(x2, y2, z2);
            //     break;
            case 4:
                TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
                break;
            case 5:
                TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
                // case 5:
                //     TargetLeg->LegMoving(x2, y2, z2);
                break;
            default:
                break;
            }
        }
        delay(DSD);
        t += 0.2;
    }
}
// int16_t T = 2000;                 // 周期
// int16_t DSD = 500;                // 每点间隔
// uint8_t flag1 = 0, flag2 = 0;     // 两组腿位置标志位
// uint8_t B_flag1 = 0, B_flag2 = 0; // 摆动标志位 1为摆动 0为支撑
// float x1 = movebackward_position[2][0], y1 = movebackward_position[2][1], z1 = movebackward_position[2][2], x2 = movebackward_position[2][0], y2 = movebackward_position[2][1], z2 = movebackward_position[2][2];
// int16_t t = 0;
// while (t <= T)
// {
//     if (t <= T / 2)
//     {
//         // 机械腿组1摆动相
//         x1 = movebackward_position[flag1][0];
//         y1 = movebackward_position[flag1][1];
//         z1 = movebackward_position[flag1][2];
//         if (flag1 == 2)
//         {
//             x2 = movebackward_position[2][0];
//             y2 = movebackward_position[2][1];
//             z2 = movebackward_position[2][2];
//         }
//         flag1++;
//         flag1 %= 3;
//     }
//     if (t > T / 2 && t <= T)
//     {
//         // 机械腿组2摆动相
//         x2 = movebackward_position[flag2][0];
//         y2 = movebackward_position[flag2][1];
//         z2 = movebackward_position[flag2][2];
//         if (flag2 == 2)
//         {
//             x1 = movebackward_position[2][0];
//             y1 = movebackward_position[2][1];
//             z1 = movebackward_position[2][2];
//         }
//         flag2++;
//         flag2 %= 3;
//     }

//     for (size_t i = 0; i < AddedNumofLeg; i++)
//     {
//         LegConfig *TargetLeg;
//         xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
//         switch (i)
//         {
//         case 0:
//             TargetLeg->LegMoving(x1, y1, z1 - 50);
//             break;

//         case 1:
//             TargetLeg->LegMoving(157.5, 0, -140.8);
//             // case 1:
//             //     TargetLeg->LegMoving(x2, y2, z2);
//             break;
//         case 2:
//             TargetLeg->LegMoving(x1, y1, z1 - 50);
//             break;
//         case 3:
//             TargetLeg->LegMoving(157.5, 0, -140.8);
//             break;
//         // case 3:
//         //     TargetLeg->LegMoving(x2, y2, z2);
//         //     break;
//         case 4:
//             TargetLeg->LegMoving(x1, y1, z1 - 50);
//             break;
//         case 5:
//             TargetLeg->LegMoving(157.5, 0, -140.8);
//             // case 5:
//             //     TargetLeg->LegMoving(x2, y2, z2);
//             break;
//         default:
//             break;
//         }
//     }

//     delay(DSD);
//     t += DSD;
// }
//}
// void straight()
// {
//     int16_t T = 2000;                 // 周期
//     int16_t DSD = 500;                // 每点间隔
//     uint8_t flag1 = 0, flag2 = 0;     // 两组腿位置标志位
//     uint8_t B_flag1 = 0, B_flag2 = 0; // 摆动标志位 1为摆动 0为支撑
//     float x1 = moveforward_position[2][0], y1 = moveforward_position[2][1], z1 = moveforward_position[2][2], x2 = moveforward_position[2][0], y2 = moveforward_position[2][1], z2 = moveforward_position[2][2];
//     int16_t t = 0;
//     while (t <= T)
//     {
//         if (t <= T / 2)
//         {
//             // 机械腿组1摆动相
//             x1 = moveforward_position[flag1][0];
//             y1 = moveforward_position[flag1][1];
//             z1 = moveforward_position[flag1][2];
//             if (flag1 == 2)
//             {
//                 x2 = moveforward_position[2][0];
//                 y2 = moveforward_position[2][1];
//                 z2 = moveforward_position[2][2];
//             }
//             flag1++;
//             flag1 %= 3;
//         }
//         if (t > T / 2 && t <= T)
//         {
//             // 机械腿组2摆动相
//             x2 = moveforward_position[flag2][0];
//             y2 = moveforward_position[flag2][1];
//             z2 = moveforward_position[flag2][2];
//             if (flag2 == 2)
//             {
//                 x1 = moveforward_position[2][0];
//                 y1 = moveforward_position[2][1];
//                 z1 = moveforward_position[2][2];
//             }
//             flag2++;
//             flag2 %= 3;
//         }

//         for (size_t i = 0; i < AddedNumofLeg; i++)
//         {
//             LegConfig *TargetLeg;
//             xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
//             switch (i)
//             {
//             case 0:
//                 TargetLeg->LegMoving(x1, y1, z1);
//                 break;

//             case 1:
//                 TargetLeg->LegMoving(x2, y2, z2);
//                 break;
//             case 2:
//                 TargetLeg->LegMoving(x1, y1, z1);
//                 break;
//             case 3:
//                 TargetLeg->LegMoving(x2, y2, z2);
//                 break;
//             case 4:
//                 TargetLeg->LegMoving(x1, y1, z1);
//                 break;
//             case 5:
//                 TargetLeg->LegMoving(x2, y2, z2);
//                 break;
//             default:
//                 break;
//             }
//         }

//         delay(DSD);
//         t += DSD;
//     }
// }
void back()
{
    int16_t DSD = 200; // 每点间隔
    float T = 2;       // 周期
    float step = 200;  // 步长
    float H = 100;     // 步高
    float t = 0;
    float Position_Swing[3];
    float Position_Support[3];

    while (t <= T)
    {

        if (t <= T / 2)
        {
            Position_Swing[0] = 127.4;
            Position_Swing[1] = -6 * step * pow(t + 1, 5) + 45 * step * pow(t + 1, 4) - 130 * step * pow(t + 1, 3) + 180 * step * pow(t + 1, 2) - 120 * step * (t + 1) + 63 * step / 2;
            Position_Swing[2] = -64 * H * pow(t, 6) + 192 * H * pow(t, 5) - 192 * H * pow(t, 4) + 64 * H * pow(t, 3) - 140.8;

            Position_Support[0] = 127.4;
            Position_Support[1] = 6 * step * pow(t, 5) - 15 * step * pow(t, 4) + 10 * step * pow(t, 3) - step / 2;
            Position_Support[2] = -140.8;
        }
        else if (t <= T && t > T / 2)
        {
            Position_Support[0] = 127.4;
            Position_Support[1] = -6 * step * pow(t, 5) + 45 * step * pow(t, 4) - 130 * step * pow(t, 3) + 180 * step * pow(t, 2) - 120 * step * t + 63 * step / 2;
            Position_Support[2] = -64 * H * pow(t - 1, 6) + 192 * H * pow(t - 1, 5) - 192 * H * pow(t - 1, 4) + 64 * H * pow(t - 1, 3) - 140.8;

            Position_Swing[0] = 127.4;
            Position_Swing[1] = 6 * step * pow(t - 1, 5) - 15 * step * pow(t - 1, 4) + 10 * step * pow(t - 1, 3) - step / 2;
            Position_Swing[2] = -140.8;
        }
        else
        {
        }; // do nothing
        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
                break;

            case 1:
                TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
                // case 1:
                //     TargetLeg->LegMoving(x2, y2, z2);
                break;
            case 2:
                TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
                break;
            case 3:
                TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
                break;
            // case 3:
            //     TargetLeg->LegMoving(x2, y2, z2);
            //     break;
            case 4:
                TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
                break;
            case 5:
                TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
                // case 5:
                //     TargetLeg->LegMoving(x2, y2, z2);
                break;
            default:
                break;
            }
        }
        delay(DSD);
        t += 0.2;
    }
}
void back_walk_Task(void *pvParameters)
{
    while (1)
    {
        back();
        vTaskDelay(1);
    }
}

void straight_walk_task(void *pvParameters)
{
    while (1)
    {
        straight();

        vTaskDelay(1);
    }
}
void back_walk_task(void *pvParameters)
{
    while (1)
    {
        back();

        vTaskDelay(1);
    }
}
void left()
{
    int16_t T = 2000;             // 周期
    int16_t DSD = 500;            // 每点间隔
    uint8_t flag1 = 0, flag2 = 0; // 两组腿位置标志位
    int16_t t = 0;
    float x1 = left_position[2][0], y1 = left_position[2][1], z1 = left_position[2][2];
    float x2 = left_position[2][0], y2 = left_position[2][1], z2 = left_position[2][2];
    float x3 = left_position[2][0], y3 = left_position[2][1], z3 = left_position[2][2];
    float x4 = left_position[2][0], y4 = left_position[2][1], z4 = left_position[2][2];
    while (t <= T)
    {
        if (t <= T / 2)
        {
            // 机械腿组1摆动相
            x1 = left_position[flag1][0];
            y1 = left_position[flag1][1];
            z1 = left_position[flag1][2];

            x2 = right_position[flag1][0];
            y2 = right_position[flag1][1];
            z2 = right_position[flag1][2];
            if (flag1 == 2)
            {
                x3 = right_position[2][0];
                y3 = right_position[2][1];
                z3 = right_position[2][2];

                x4 = right_position[2][0];
                y4 = right_position[2][1];
                z4 = right_position[2][2];
            }

            flag1++;
            flag1 %= 3;
        }
        if (t > T / 2 && t <= T)
        {
            // 机械腿组2摆动相
            x3 = right_position[flag2][0];
            y3 = right_position[flag2][1];
            z3 = right_position[flag2][2];

            x4 = left_position[flag2][0];
            y4 = left_position[flag2][1];
            z4 = left_position[flag2][2];
            if (flag2 == 2)
            {
                x1 = left_position[2][0];
                y1 = left_position[2][1];
                z1 = left_position[2][2];

                x2 = right_position[2][0];
                y2 = right_position[2][1];
                z2 = right_position[2][2];
            }

            flag2++;
            flag2 %= 3;
        }

        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegMoving(x3, y3, z3);
                break;

            case 1:
                TargetLeg->LegMoving(x2, y2, z2);
                break;
            case 2:
                TargetLeg->LegMoving(x3, y3, z3);
                break;
            case 3:
                TargetLeg->LegMoving(x1, y1, z1);
                break;
            case 4:
                TargetLeg->LegMoving(x4, y4, z4);
                break;
            case 5:
                TargetLeg->LegMoving(x1, y1, z1);
                break;
            default:
                break;
            }
        }

        delay(DSD);
        t += DSD;
    }
}
void right()
{
    int16_t T = 2000;             // 周期
    int16_t DSD = 500;            // 每点间隔
    uint8_t flag1 = 0, flag2 = 0; // 两组腿位置标志位
    int16_t t = 0;
    float x1 = left_position[2][0], y1 = left_position[2][1], z1 = left_position[2][2];
    float x2 = left_position[2][0], y2 = left_position[2][1], z2 = left_position[2][2];
    float x3 = left_position[2][0], y3 = left_position[2][1], z3 = left_position[2][2];
    float x4 = left_position[2][0], y4 = left_position[2][1], z4 = left_position[2][2];
    while (t <= T)
    {
        if (t <= T / 2)
        {
            // 机械腿组1摆动相
            x1 = right_position[flag1][0];
            y1 = right_position[flag1][1];
            z1 = right_position[flag1][2];

            x2 = left_position[flag1][0];
            y2 = left_position[flag1][1];
            z2 = left_position[flag1][2];
            if (flag1 == 2)
            {
                x3 = left_position[2][0];
                y3 = left_position[2][1];
                z3 = left_position[2][2];

                x4 = left_position[2][0];
                y4 = left_position[2][1];
                z4 = left_position[2][2];
            }

            flag1++;
            flag1 %= 3;
        }
        if (t > T / 2 && t <= T)
        {
            // 机械腿组2摆动相
            x3 = left_position[flag2][0];
            y3 = left_position[flag2][1];
            z3 = left_position[flag2][2];

            x4 = right_position[flag2][0];
            y4 = right_position[flag2][1];
            z4 = right_position[flag2][2];
            if (flag2 == 2)
            {
                x1 = right_position[2][0];
                y1 = right_position[2][1];
                z1 = right_position[2][2];

                x2 = left_position[2][0];
                y2 = left_position[2][1];
                z2 = left_position[2][2];
            }

            flag2++;
            flag2 %= 3;
        }

        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegMoving(x3, y3, z3);
                break;

            case 1:
                TargetLeg->LegMoving(x2, y2, z2);
                break;
            case 2:
                TargetLeg->LegMoving(x3, y3, z3);
                break;
            case 3:
                TargetLeg->LegMoving(x1, y1, z1);
                break;
            case 4:
                TargetLeg->LegMoving(x4, y4, z4);
                break;
            case 5:
                TargetLeg->LegMoving(x1, y1, z1);
                break;
            default:
                break;
            }
        }

        delay(DSD);
        t += DSD;
    }
}
void left_walk_task(void *pvParameters)
{
    while (1)
    {
        left();
        vTaskDelay(1);
    }
}

void right_walk_task(void *pvParameters)
{
    while (1)
    {
        right();
        vTaskDelay(1);
    }
}
// work
void left_cross(void)
{
    int16_t T = 2000;             // 周期
    int16_t DSD = 500;            // 每点间隔
    uint8_t flag1 = 0, flag2 = 0; // 两组腿位置标志位
    int16_t t = 0;
    float x1 = Front_left_position[2][0], y1 = Front_left_position[2][1], z1 = Front_left_position[2][2];
    float x2 = Mid_left_position[2][0], y2 = Mid_left_position[2][1], z2 = Mid_left_position[2][2];
    float x3 = Back_left_position[2][0], y3 = Back_left_position[2][1], z3 = Back_left_position[2][2];
    float x4 = Front_right_position[2][0], y4 = Front_right_position[2][1], z4 = Front_right_position[2][2];
    float x5 = Mid_right_position[2][0], y5 = Mid_right_position[2][1], z5 = Mid_right_position[2][2];
    float x6 = Back_right_position[2][0], y6 = Back_right_position[2][1], z6 = Back_right_position[2][2];
    while (t <= T)
    {
        if (t <= T / 2)
        {
            // 机械腿组1摆动相
            x1 = Front_left_position[flag1][0];
            y1 = Front_left_position[flag1][1];
            z1 = Front_left_position[flag1][2];

            x3 = Back_left_position[flag1][0];
            y3 = Back_left_position[flag1][1];
            z3 = Back_left_position[flag1][2];

            x5 = Mid_right_position[flag1][0];
            y5 = Mid_right_position[flag1][1];
            z5 = Mid_right_position[flag1][2];
            if (flag1 == 2)
            {
                x2 = Mid_left_position[2][0];
                y2 = Mid_left_position[2][1];
                z2 = Mid_left_position[2][2];

                x4 = Front_right_position[2][0];
                y4 = Front_right_position[2][1];
                z4 = Front_right_position[2][2];

                x6 = Back_right_position[2][0];
                y6 = Back_right_position[2][1];
                z6 = Back_right_position[2][2];
            }

            flag1++;
            flag1 %= 3;
        }
        if (t > T / 2 && t <= T)
        {
            // 机械腿组2摆动相
            x2 = Mid_left_position[flag2][0];
            y2 = Mid_left_position[flag2][1];
            z2 = Mid_left_position[flag2][2];

            x4 = Front_right_position[flag2][0];
            y4 = Front_right_position[flag2][1];
            z4 = Front_right_position[flag2][2];

            x6 = Back_right_position[flag2][0];
            y6 = Back_right_position[flag2][1];
            z6 = Back_right_position[flag2][2];

            if (flag2 == 2)
            {
                x1 = Front_left_position[2][0];
                y1 = Front_left_position[2][1];
                z1 = Front_left_position[2][2];

                x3 = Back_left_position[2][0];
                y3 = Back_left_position[2][1];
                z3 = Back_left_position[2][2];

                x5 = Mid_right_position[2][0];
                y5 = Mid_right_position[2][1];
                z5 = Mid_right_position[2][2];
            }

            flag2++;
            flag2 %= 3;
        }

        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegMoving(x1, y1, z1);
                break;

            case 1:
                TargetLeg->LegMoving(x2, y2, z2);
                break;
            case 2:
                TargetLeg->LegMoving(x3, y3, z3);
                break;
            case 3:
                TargetLeg->LegMoving(x4, y4, z4);
                break;
            case 4:
                TargetLeg->LegMoving(x5, y5, z5);
                break;
            case 5:
                TargetLeg->LegMoving(x6, y6, z6);
                break;
            default:
                break;
            }
        }

        delay(DSD);
        t += DSD;
    }
}
void left_cross_walk_task(void *pvParameters)
{
    while (1)
    {
        left_cross();
        vTaskDelay(1);
    }
}

void right_cross(void)
{
    int16_t DSD = 500; // 每点间隔
    float T = 2;       // 周期
    float step = 100;  // 步长
    float H = 60;      // 步高
    float t = 0;
    float Position_Swing_L[3];
    float Position_Swing_R[3];
    float Position_Support_L[3];
    float Position_Support_R[3];

    while (t <= T)
    {

        if (t <= T / 2)
        {
            Position_Swing_L[0] = 6 * step * pow(t, 5) - 15 * step * pow(t, 4) + 10 * step * pow(t, 3) + 108.4; // 0~step
            Position_Swing_L[1] = 0;
            Position_Swing_L[2] = -9000 * pow(t, 7) + (29100 - 64 * H) * pow(t, 6) + (192 * H - 33750) * pow(t, 5) + (16425 - 192 * H) * pow(t, 4) + (64 * H - 2850) * pow(t, 3) - 104.8; // 76.6; //(-30 + H) * pow(t, 5) - (60 - 2 * H) * pow(t, 4) + (-30 + H) * pow(t, 3) - 76.6; // 0~H

            Position_Swing_R[0] = 6 * step * pow(1 - t, 5) - 15 * step * pow(1 - t, 4) + 10 * step * pow(1 - t, 3) + 108.4; // step~0
            Position_Swing_R[1] = 0;
            Position_Swing_R[2] = -9000 * pow(1 - t, 7) + (29100 - 64 * H) * pow(1 - t, 6) + (192 * H - 33750) * pow(1 - t, 5) + (16425 - 192 * H) * pow(1 - t, 4) + (64 * H - 2850) * pow(1 - t, 3) - 104.8; // //(-30 + H) * pow(1 - t, 3) - (60 - 2 * H) * pow(1 - t, 4) + (-30 + H) * pow(1 - t, 5) - 76.6; // H~0

            Position_Support_L[0] = -6 * step * pow(t + 1, 5) + 45 * step * pow(t + 1, 4) - 130 * step * pow(t + 1, 3) + 180 * step * pow(t + 1, 2) - 120 * step * (t + 1) + 32 * step + 108.4; // step~0
            Position_Support_L[1] = 0;
            Position_Support_L[2] = -75 + 75 * pow(t, 6) - 76.6; // H~0

            Position_Support_R[0] = 6 * step * pow(t, 5) - 15 * step * pow(t, 4) + 10 * step * pow(t, 3) + 108.4; // 0~step-6 * step * pow(2 - t, 5) + 45 * step * pow(2 - t, 4) - 130 * step * pow(2 - t, 3) + 180 * step * pow(2 - t, 2) - 120 * step * (2 - t) + 32 * step + 108.4; // 0~step
            Position_Support_R[1] = 0;
            Position_Support_R[2] = -75 + 75 * pow(1 - t, 6) - 76.6; //(-50) * pow(t, 3) + (100) * pow(t, 4) + (-50) * pow(t, 5) - 76.6; // 0~H
        }
        else if (t <= T && t > T / 2)
        {
            Position_Support_R[0] = -6 * step * pow(t, 5) + 45 * step * pow(t, 4) - 130 * step * pow(t, 3) + 180 * step * pow(t, 2) - 120 * step * t + 32 * step + 108.4; // step~0
            Position_Support_R[1] = 0;
            Position_Support_R[2] = -9000 * pow(2 - t, 7) + (29100 - 64 * H) * pow(2 - t, 6) + (192 * H - 33750) * pow(2 - t, 5) + (16425 - 192 * H) * pow(2 - t, 4) + (64 * H - 2850) * pow(2 - t, 3) - 104.8; //// H~0

            Position_Support_L[0] = 6 * step * pow(t - 1, 5) - 15 * step * pow(t - 1, 4) + 10 * step * pow(t - 1, 3) + 108.4; // 0~step
            Position_Support_L[1] = 0;
            Position_Support_L[2] = -9000 * pow(t - 1, 7) + (29100 - 64 * H) * pow(t - 1, 6) + (192 * H - 33750) * pow(t - 1, 5) + (16425 - 192 * H) * pow(t - 1, 4) + (64 * H - 2850) * pow(t - 1, 3) - 104.8; //// 0~H

            Position_Swing_L[0] = -6 * step * pow(t, 5) + 45 * step * pow(t, 4) - 130 * step * pow(t, 3) + 180 * step * pow(t, 2) - 120 * step * t + 32 * step + 108.4; // step~0
            Position_Swing_L[1] = 0;
            Position_Swing_L[2] = -75 + 75 * pow(t - 1, 6) - 76.6; //-75 + 50 * pow(t - 1, 3) + (-100) * pow(t - 1, 4) + (75) * pow(t - 1, 5) - 76.6; // H~0

            Position_Swing_R[0] = 6 * step * pow(t - 1, 5) - 15 * step * pow(t - 1, 4) + 10 * step * pow(t - 1, 3) + 108.4; // 0~step
            Position_Swing_R[1] = 0;
            Position_Swing_R[2] = -75 + 75 * pow(2 - t, 6) - 76.6; //(-50) * pow(t - 1, 3) + (100) * pow(t - 1, 4) + (-50) * pow(t - 1, 5) - 76.6; // 0~H
        }
        else
        {
        }; // do nothing
        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegMoving(Position_Swing_L[0], Position_Swing_L[1], Position_Swing_L[2]);
                break;

            case 1:
                TargetLeg->LegMoving(Position_Support_L[0], Position_Support_L[1], Position_Support_L[2]);
                // case 1:
                //     TargetLeg->LegMoving(x2, y2, z2);
                break;
            case 2:
                TargetLeg->LegMoving(Position_Swing_L[0], Position_Swing_L[1], Position_Swing_L[2]);
                break;
            case 3:
                TargetLeg->LegMoving(Position_Support_R[0], Position_Support_R[1], Position_Support_R[2]);
                break;
            // case 3:
            //     TargetLeg->LegMoving(x2, y2, z2);
            //     break;
            case 4:
                TargetLeg->LegMoving(Position_Swing_R[0], Position_Swing_R[1], Position_Swing_R[2]);
                break;
            case 5:
                TargetLeg->LegMoving(Position_Support_R[0], Position_Support_R[1], Position_Support_R[2]);
                // case 5:
                //     TargetLeg->LegMoving(x2, y2, z2);
                break;
            default:
                break;
            }
        }
        delay(DSD);
        t += 0.5;
    }
    // int16_t T = 2000;             // 周期
    // int16_t DSD = 500;            // 每点间隔
    // uint8_t flag1 = 0, flag2 = 0; // 两组腿位置标志位
    // int16_t t = 0;
    // float x1 = Front_right_position[2][0], y1 = Front_right_position[2][1], z1 = Front_right_position[2][2];
    // float x2 = Mid_left_position[2][0], y2 = Mid_left_position[2][1], z2 = Mid_left_position[2][2];
    // float x3 = Back_right_position[2][0], y3 = Back_right_position[2][1], z3 = Back_right_position[2][2];
    // float x4 = Front_left_position[2][0], y4 = Front_left_position[2][1], z4 = Front_left_position[2][2];
    // float x5 = Mid_right_position[2][0], y5 = Front_left_position[2][1], z5 = Front_left_position[2][2];
    // float x6 = Back_left_position[2][0], y6 = Back_left_position[2][1], z6 = Back_left_position[2][2];
    // while (t <= T)
    // {
    //     if (t <= T / 2)
    //     {
    //         // 机械腿组1摆动相
    //         x4 = Front_left_position[flag1][0];
    //         y4 = Front_left_position[flag1][1];
    //         z4 = Front_left_position[flag1][2];

    //         x6 = Back_left_position[flag1][0];
    //         y6 = Back_left_position[flag1][1];
    //         z6 = Back_left_position[flag1][2];

    //         x2 = Mid_right_position[flag1][0];
    //         y2 = Mid_right_position[flag1][1];
    //         z2 = Mid_right_position[flag1][2];
    //         if (flag1 == 2)
    //         {
    //             x1 = Front_right_position[2][0];
    //             y1 = Front_right_position[2][1];
    //             z1 = Front_right_position[2][2];

    //             x3 = Back_right_position[2][0];
    //             y3 = Back_right_position[2][1];
    //             z3 = Back_right_position[2][2];

    //             x5 = Mid_left_position[2][0];
    //             y5 = Mid_left_position[2][1];
    //             z5 = Mid_left_position[2][2];
    //         }

    //         flag1++;
    //         flag1 %= 3;
    //     }
    //     if (t > T / 2 && t <= T)
    //     {
    //         // 机械腿组2摆动相

    //         x1 = Front_right_position[flag2][0];
    //         y1 = Front_right_position[flag2][1];
    //         z1 = Front_right_position[flag2][2];

    //         x3 = Back_right_position[flag2][0];
    //         y3 = Back_right_position[flag2][1];
    //         z3 = Back_right_position[flag2][2];

    //         x5 = Mid_left_position[flag2][0];
    //         y5 = Mid_left_position[flag2][1];
    //         z5 = Mid_left_position[flag2][2];

    //         if (flag2 == 2)
    //         {
    //             x4 = Front_left_position[2][0];
    //             y4 = Front_left_position[2][1];
    //             z4 = Front_left_position[2][2];

    //             x2 = Mid_right_position[2][0];
    //             y2 = Mid_right_position[2][1];
    //             z2 = Mid_right_position[2][2];

    //             x6 = Back_left_position[2][0];
    //             y6 = Back_left_position[2][1];
    //             z6 = Back_left_position[2][2];
    //         }

    //         flag2++;
    //         flag2 %= 3;
    //     }

    //     for (size_t i = 0; i < AddedNumofLeg; i++)
    //     {
    //         LegConfig *TargetLeg;
    //         xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
    //         switch (i)
    //         {
    //         case 0:
    //             TargetLeg->LegMoving(x1, y1, z1);
    //             break;

    //         case 1:
    //             TargetLeg->LegMoving(x2, y2, z2);
    //             break;
    //         case 2:
    //             TargetLeg->LegMoving(x3, y3, z3);
    //             break;
    //         case 3:
    //             TargetLeg->LegMoving(x4, y4, z4);
    //             break;
    //         case 4:
    //             TargetLeg->LegMoving(x5, y5, z5);
    //             break;
    //         case 5:
    //             TargetLeg->LegMoving(x6, y6, z6);
    //             break;
    //         default:
    //             break;
    //         }
    //     }

    //     delay(DSD);
    //     t += DSD;
    // }
}
void right_cross_walk_task(void *pvParameters)
{
    while (1)
    {
        right_cross();
        vTaskDelay(1);
    }
}
void LegAngleQuery_Task(void *pvParameters)
{
    TCPConfig *Target = (TCPConfig *)pvParameters; // 接收对应LegConfig对象
    for (size_t i = 0; i < AddedNumofLeg; i++)
    {
        LegConfig *TargetLeg;
        xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
        TargetLeg->SetDampMode();
    }
    Target->TCP.println("[Leg Power]All Leg is Set to Damp Mode.");
    Target->TCP.println("[LegAngleQuery]choose the mode you want to query:1.raw angle 2.real angle 3.both");
    u8_t mode = 1;

    while (1)
    {
        if (Target->ReceiveData != "")
        {
            mode = Target->ReceiveData.toInt();
            Target->ReceiveData = "";
            Target->TCP.println("[LegAngleQuery]Please enter the Serial Number of the Leg you want to Query.");
            while (Target->ReceiveData == "")
                ;
            if (mode == 1)
            {
                Target->TCP.printf("[LegAngleQuery]The Serial Number of the Leg you want to Query is %s.\n", Target->ReceiveData.c_str());
                int LegNum = Target->ReceiveData.toInt() - 1;
                if (LegNum < AddedNumofLeg)
                {
                    LegConfig *TargetLeg;
                    xQueuePeek(LegQueue[LegNum], &TargetLeg, portMAX_DELAY);
                    while (1)
                    {
                        Target->TCP.printf("[LegAngleQuery]The Raw angle is %f,%f,%f.\n", TargetLeg->hipServo.queryAngle() - defaultAngleArray[LegNum][0], TargetLeg->kneeServo.queryAngle() - defaultAngleArray[LegNum][1], TargetLeg->ankleServo.queryAngle() - defaultAngleArray[LegNum][2]);
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                    }
                }
            }
            else if (mode == 2)
            {
                Target->TCP.printf("[LegAngleQuery]The Serial Number of the Leg you want to Query is %s.\n", Target->ReceiveData.c_str());
                int LegNum = Target->ReceiveData.toInt() - 1;
                if (LegNum < AddedNumofLeg)
                {
                    LegConfig *TargetLeg;
                    xQueuePeek(LegQueue[LegNum], &TargetLeg, portMAX_DELAY);
                    while (1)
                    {
                        Target->TCP.printf("[LegAngleQuery]The real Angle is %f,%f,%f.\n", TargetLeg->hipServo.queryAngle(), TargetLeg->kneeServo.queryAngle(), TargetLeg->ankleServo.queryAngle());
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                    }
                }
            }
            else if (mode == 3)
            {
                Target->TCP.printf("[LegAngleQuery]The Serial Number of the Leg you want to Query is %s.\n", Target->ReceiveData.c_str());
                int LegNum = Target->ReceiveData.toInt() - 1;
                if (LegNum < AddedNumofLeg)
                {
                    LegConfig *TargetLeg;
                    xQueuePeek(LegQueue[LegNum], &TargetLeg, portMAX_DELAY);
                    while (1)
                    {
                        Target->TCP.printf("[LegAngleQuery]The Raw Angle is %f,%f,%f.\n", TargetLeg->hipServo.queryAngle() - defaultAngleArray[LegNum][0], TargetLeg->kneeServo.queryAngle() - defaultAngleArray[LegNum][1], TargetLeg->ankleServo.queryAngle() - defaultAngleArray[LegNum][2]);
                        Target->TCP.printf("[LegAngleQuery]The real Angle is %f,%f,%f.\n", TargetLeg->hipServo.queryAngle(), TargetLeg->kneeServo.queryAngle(), TargetLeg->ankleServo.queryAngle());
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                    }
                }
            }
            else
            {
                Target->TCP.println("[LegAngleQuery]The mode you choose is out of range.");
                Target->TCP.println("[LegAngleQuery]Please enter the Serial Number of the Leg you want to Query.");
            }
            Target->ReceiveData = "";
        }
        vTaskDelay(1);
    }
}
void LegPowerDown_Task(void *pvParameters)
{
    TCPConfig *Target = (TCPConfig *)pvParameters; // 接收对应LegConfig对象

    Target->TCP.println("[LegPowerDown]Please enter the Serial Number of the Leg you want to PowerDown.");
    while (1)
    {
        if (Target->ReceiveData != "")
        {
            Target->TCP.printf("[LegPowerDown]The Serial Number of the Leg you want to PowerDown is %s.\n", Target->ReceiveData.c_str());
            int LegNum = Target->ReceiveData.toInt() - 1;
            if (Target->ReceiveData == "all")
            {
                for (size_t i = 0; i < AddedNumofLeg; i++)
                {
                    LegConfig *TargetLeg;
                    xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
                    TargetLeg->LegPowerDown();
                    Target->TCP.printf("[LegPowerDown]Leg %d is PowerDown.\n", i);
                }
                Target->Terminal_TaskHandle = NULL;
                Target->truncateStream = false;
                vTaskDelete(NULL);
            }
            else if (LegNum < AddedNumofLeg)
            {
                LegConfig *TargetLeg;
                xQueuePeek(LegQueue[LegNum], &TargetLeg, portMAX_DELAY);
                TargetLeg->LegPowerDown();
                Target->TCP.printf("[LegPowerDown]Leg %d is PowerDown.\n", LegNum + 1);
                Target->Terminal_TaskHandle = NULL;
                Target->truncateStream = false;
                vTaskDelete(NULL);
            }

            else
            {
                Target->TCP.println("[LegPowerDown]The Serial Number is out of range.");
                Target->TCP.println("[LegPowerDown]Please enter the Serial Number of the Leg you want to PowerDown.");
            }
            Target->ReceiveData = "";
        }
        vTaskDelay(1);
    }
}
void LegMoving_Task(void *pvParameters)
{
    TCPConfig *Target = (TCPConfig *)pvParameters; // 接收对应LegConfig对象
    Target->TCP.println("[LegMoving]Please enter the Serial Number of the Leg you want to Moving.");
    while (1)
    {
        if (Target->ReceiveData != "")
        {
            Target->TCP.printf("[LegMoving]The Serial Number of the Leg you want to Moving is %s.\n", Target->ReceiveData.c_str());
            int LegNum = Target->ReceiveData.toInt() - 1;
            Target->ReceiveData = "";
            if (LegNum < AddedNumofLeg)
            {
                LegConfig *TargetLeg;
                xQueuePeek(LegQueue[LegNum], &TargetLeg, portMAX_DELAY);
                Target->TCP.printf("loop? y/n\n");
                while (Target->ReceiveData == "")
                    ;

                if (Target->ReceiveData == "y")
                {
                    while (1)
                    {
                        TargetLeg->LegMoving();
                    }
                }
                else if (Target->ReceiveData == "n")
                {
                    TargetLeg->LegMoving();
                }

                vTaskDelete(NULL);
            }
            else
            {
                Target->TCP.println("[LegMoving]The Serial Number is out of range.");
                Target->TCP.println("[LegMoving]Please enter the Serial Number of the Leg you want to Moving.");
            }
            Target->ReceiveData = "";
        }
        vTaskDelay(1);
    }
}