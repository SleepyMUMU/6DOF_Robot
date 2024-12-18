#include "Servo.h"
#include "main.h"

bool ControlExit_Flag = false;
// uint16_t servoDefaultTime = 200;
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

// float matrix_current[3][3];

// void straight()
// {

//     int16_t DSD = 200; // 每点间隔
//     float T = 2;       // 周期
//     float step = 200;  // 步长
//     float H = 100;     // 步高
//     float t = 0;
//     float Position_Swing[3];
//     float Position_Support[3];

//     while (t <= T)
//     {

//         if (t <= T / 2)
//         {
//             Position_Swing[0] = vector_Stand[0][0];
//             Position_Swing[1] = 6 * step * pow(t, 5) - 15 * step * pow(t, 4) + 10 * step * pow(t, 3) - step / 2 + vector_Stand[0][1];
//             Position_Swing[2] = vector_Stand[0][2] + (-64) * H * pow(t, 6) + 192 * H * pow(t, 5) - 192 * H * pow(t, 4) + 64 * H * pow(t, 3);

//             Position_Support[0] = vector_Stand[0][0];
//             Position_Support[1] = (-6) * step * pow(t + 1, 5) + 45 * step * pow(t + 1, 4) - 130 * step * pow(t + 1, 3) + 180 * step * pow(t + 1, 2) - 120 * step * (t + 1) + 63 * step / 2;
//             Position_Support[2] = vector_Stand[0][2];
//         }
//         else if (t <= T && t > T / 2)
//         {
//             Position_Support[0] = vector_Stand[0][0];
//             Position_Support[1] = 6 * step * pow(t - 1, 5) - 15 * step * pow(t - 1, 4) + 10 * step * pow(t - 1, 3) - step / 2 + vector_Stand[0][1];
//             Position_Support[2] = vector_Stand[0][2] + (-64) * H * pow(t - 1, 6) + 192 * H * pow(t - 1, 5) - 192 * H * pow(t - 1, 4) + 64 * H * pow(t - 1, 3);

//             Position_Swing[0] = vector_Stand[0][0];
//             Position_Swing[1] = (-6) * step * pow(t, 5) + 45 * step * pow(t, 4) - 130 * step * pow(t, 3) + 180 * step * pow(t, 2) - 120 * step * t + 63 * step / 2;
//             Position_Swing[2] = vector_Stand[0][2];
//         }
//         else
//         {
//         }; // do nothing
//         for (size_t i = 0; i < AddedNumofLeg; i++)
//         {
//             LegConfig *TargetLeg;
//             xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
//             switch (i)
//             {
//             case 0:
//                 TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
//                 break;

//             case 1:
//                 TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
//                 // case 1:
//                 //     TargetLeg->LegMoving(x2, y2, z2);
//                 break;
//             case 2:
//                 TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
//                 break;
//             case 3:
//                 TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
//                 break;
//             // case 3:
//             //     TargetLeg->LegMoving(x2, y2, z2);
//             //     break;
//             case 4:
//                 TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
//                 break;
//             case 5:
//                 TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
//                 // case 5:
//                 //     TargetLeg->LegMoving(x2, y2, z2);
//                 break;
//             default:
//                 break;
//             }
//         }
//         delay(DSD);
//         t += 0.2;
//     }
// }

// void straight()
// {

//     int16_t DSD = 200; // 每点间隔
//     float T = 2;       // 周期
//     float step = 200;  // 步长
//     float H = 100;     // 步高
//     float t = 0;
//     float Position_Swing[3];
//     float Position_Support[3];

//     while (t <= T)
//     {

//         if (t <= T / 2)
//         {
//             Position_Swing[0] = 127.4;
//             Position_Swing[1] = 6 * step * pow(t, 5) - 15 * step * pow(t, 4) + 10 * step * pow(t, 3) - step / 2;
//             Position_Swing[2] = -64 * H * pow(t, 6) + 192 * H * pow(t, 5) - 192 * H * pow(t, 4) + 64 * H * pow(t, 3) - 140.8;

//             Position_Support[0] = 127.4;
//             Position_Support[1] = -6 * step * pow(t + 1, 5) + 45 * step * pow(t + 1, 4) - 130 * step * pow(t + 1, 3) + 180 * step * pow(t + 1, 2) - 120 * step * (t + 1) + 63 * step / 2;
//             Position_Support[2] = -140.8;
//         }
//         else if (t <= T && t > T / 2)
//         {
//             Position_Support[0] = 127.4;
//             Position_Support[1] = 6 * step * pow(t - 1, 5) - 15 * step * pow(t - 1, 4) + 10 * step * pow(t - 1, 3) - step / 2;
//             Position_Support[2] = -64 * H * pow(t - 1, 6) + 192 * H * pow(t - 1, 5) - 192 * H * pow(t - 1, 4) + 64 * H * pow(t - 1, 3) - 140.8;

//             Position_Swing[0] = 127.4;
//             Position_Swing[1] = -6 * step * pow(t, 5) + 45 * step * pow(t, 4) - 130 * step * pow(t, 3) + 180 * step * pow(t, 2) - 120 * step * t + 63 * step / 2;
//             Position_Swing[2] = -140.8;
//         }
//         else
//         {
//         }; // do nothing
//         for (size_t i = 0; i < AddedNumofLeg; i++)
//         {
//             LegConfig *TargetLeg;
//             xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
//             switch (i)
//             {
//             case 0:
//                 TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
//                 break;

//             case 1:
//                 TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
//                 // case 1:
//                 //     TargetLeg->LegMoving(x2, y2, z2);
//                 break;
//             case 2:
//                 TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
//                 break;
//             case 3:
//                 TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
//                 break;
//             // case 3:
//             //     TargetLeg->LegMoving(x2, y2, z2);
//             //     break;
//             case 4:
//                 TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
//                 break;
//             case 5:
//                 TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
//                 // case 5:
//                 //     TargetLeg->LegMoving(x2, y2, z2);
//                 break;
//             default:
//                 break;
//             }
//         }
//         delay(DSD);
//         t += 0.2;
//     }
// }

void up_slope()
{

    int16_t DSD = 120;      // 每点间隔
    float T = 2;            // 周期
    float Length = 80;      // 步长
    float H = Length - 0.1; // 步高
    float step = 10;        // 插值次数
    float t = 0;            // 时间标志位
    int flag = 0;           // 插值标志位

    float Position_Swing[3];                                        // 摆动相位置暂存
    float Position_Support[3];                                      // 支撑相位置暂存
    Vector3 p1 = {vector_Stand[0][0], Length, vector_Stand[0][2]};  // 相对当前位置向量圆弧起始点
    Vector3 p2 = {vector_Stand[0][0], 0, vector_Stand[0][2] + H};   // 相对当前位置向量圆弧中间点
    Vector3 p3 = {vector_Stand[0][0], -Length, vector_Stand[0][2]}; // 相对当前位置向量圆弧终点

    Vector3 center;        // 圆弧圆心暂存
    Vector3 position;      // 位置暂存
    float Swinging[10][3]; // 摆动相位置
    center = calculateArcCenter(p1, p2, p3);
    for (int i = 0; i < 10; i++)
    {
        position = Invers_calculateTrack(p1, p3, center, step, i);

        for (int j = 0; j < 3; j++)
        {
            Swinging[i][0] = float(position.x);
            Swinging[i][1] = float(position.y);
            Swinging[i][2] = float(position.z);
        }
    }

    while (t < T)
    {

        if (t < T / 2)
        {
            // 左1，左3，右2摆动 y方向运动Length~-Length z方向运动0~H
            Position_Swing[0] = Swinging[flag][0];
            Position_Swing[1] = Swinging[flag][1];
            Position_Swing[2] = Swinging[flag][2];

            // 右1，右3，左2支撑 y方向运动-Length~0.8*Length
            Position_Support[0] = vector_Stand[0][0];
            Position_Support[1] = -2 * Length * t + Length;
            Position_Support[2] = vector_Stand[0][2];
        }
        else if (t < T && t >= T / 2)
        {

            if (flag >= step)
            {
                flag = 0;
            }
            Position_Support[0] = Swinging[flag][0];
            Position_Support[1] = Swinging[flag][1];
            Position_Support[2] = Swinging[flag][2];
            // Serial.println(Position_Swing[0]);
            // Serial.println("");
            // Serial.println(Position_Swing[1]);
            // Serial.println("");
            // Serial.println(Position_Swing[2]);
            // Serial.println("");

            Position_Swing[0] = vector_Stand[0][0];
            Position_Swing[1] = -2 * Length * (t - T / 2) + Length;
            Position_Swing[2] = vector_Stand[0][2];
            // Serial.println(Position_Support[0]);
            // Serial.println("");
            // Serial.println(Position_Support[1]);
            // Serial.println("");
            // Serial.println(Position_Support[2]);
            // Serial.println("");
        }
        else
        {
            t += 0.2;
            continue;

        }; // do nothing
        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2] + 44);
                break;

            case 1:
                TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2] + 22);
                break;
            case 2:
                TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
                break;
            case 3:
                TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2] + 44);
                break;

            case 4:
                TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2] + 22);
                break;
            case 5:
                TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
                break;
            default:
                break;
            }
        }

        delay(DSD);
        t += 0.2;
        flag += 2;
    }
}
void UpSlope_task(void *pvParameters)
{
    while (1)
    {
        up_slope();
        vTaskDelay(1);
    }
}
/*爬坡适应步态*/
void w_straight()
{
    int16_t DSD = 120;      // 每点间隔
    float T = 2;            // 周期
    float Length = 80;      // 步长
    float H = Length - 0.1; // 步高
    float step = 10;        // 插值次数
    float t = 0;            // 时间标志位
    int flag = 0;           // 插值标志位

    float Position_Swing[3];                                        // 摆动相位置暂存
    float Position_Support[3];                                      // 支撑相位置暂存
    Vector3 p1 = {vector_Stand[0][0], Length, vector_Stand[0][2]};  // 相对当前位置向量圆弧起始点
    Vector3 p2 = {vector_Stand[0][0], 0, vector_Stand[0][2] + H};   // 相对当前位置向量圆弧中间点
    Vector3 p3 = {vector_Stand[0][0], -Length, vector_Stand[0][2]}; // 相对当前位置向量圆弧终点

    Vector3 center;        // 圆弧圆心暂存
    Vector3 position;      // 位置暂存
    float Swinging[10][3]; // 摆动相位置
    center = calculateArcCenter(p1, p2, p3);
    for (int i = 0; i < 10; i++)
    {
        position = Invers_calculateTrack(p1, p3, center, step, i);

        for (int j = 0; j < 3; j++)
        {
            Swinging[i][0] = float(position.x);
            Swinging[i][1] = float(position.y);
            Swinging[i][2] = float(position.z);
        }
    }

    while (t < T)
    {

        if (t < T / 2)
        {
            // 左1，左3，右2摆动 y方向运动Length~-Length z方向运动0~H
            Position_Swing[0] = Swinging[flag][0];
            Position_Swing[1] = Swinging[flag][1];
            Position_Swing[2] = Swinging[flag][2];

            // 右1，右3，左2支撑 y方向运动-Length~0.8*Length
            Position_Support[0] = vector_Stand[0][0];
            Position_Support[1] = -2 * Length * t + Length;
            Position_Support[2] = vector_Stand[0][2];
        }
        else if (t < T && t >= T / 2)
        {

            if (flag >= step)
            {
                flag = 0;
            }
            Position_Support[0] = Swinging[flag][0];
            Position_Support[1] = Swinging[flag][1];
            Position_Support[2] = Swinging[flag][2];
            // Serial.println(Position_Swing[0]);
            // Serial.println("");
            // Serial.println(Position_Swing[1]);
            // Serial.println("");
            // Serial.println(Position_Swing[2]);
            // Serial.println("");

            Position_Swing[0] = vector_Stand[0][0];
            Position_Swing[1] = -2 * Length * (t - T / 2) + Length;
            Position_Swing[2] = vector_Stand[0][2];
            // Serial.println(Position_Support[0]);
            // Serial.println("");
            // Serial.println(Position_Support[1]);
            // Serial.println("");
            // Serial.println(Position_Support[2]);
            // Serial.println("");
        }
        else
        {
            t += 0.2;
            continue;

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
                break;
            case 2:
                TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2] - 30);
                break;
            case 3:
                TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
                break;

            case 4:
                TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
                break;
            case 5:
                TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2] - 30);
                break;
            default:
                break;
            }
        }

        delay(DSD);
        t += 0.2;
        flag += 2;
    }
}
/*侧向爬坡*/
// void w_straight()
// {
//     int16_t DSD = 120;      // 每点间隔
//     float T = 2;            // 周期
//     float Length = 80;      // 步长
//     float H = Length - 0.1; // 步高
//     float step = 10;        // 插值次数
//     float t = 0;            // 时间标志位
//     int flag = 0;           // 插值标志位

//     float Position_Swing[3];                                        // 摆动相位置暂存
//     float Position_Support[3];                                      // 支撑相位置暂存
//     Vector3 p1 = {vector_Stand[0][0], Length, vector_Stand[0][2]};  // 相对当前位置向量圆弧起始点
//     Vector3 p2 = {vector_Stand[0][0], 0, vector_Stand[0][2] + H};   // 相对当前位置向量圆弧中间点
//     Vector3 p3 = {vector_Stand[0][0], -Length, vector_Stand[0][2]}; // 相对当前位置向量圆弧终点

//     Vector3 center;        // 圆弧圆心暂存
//     Vector3 position;      // 位置暂存
//     float Swinging[10][3]; // 摆动相位置
//     center = calculateArcCenter(p1, p2, p3);
//     for (int i = 0; i < 10; i++)
//     {
//         position = Invers_calculateTrack(p1, p3, center, step, i);

//         for (int j = 0; j < 3; j++)
//         {
//             Swinging[i][0] = float(position.x);
//             Swinging[i][1] = float(position.y);
//             Swinging[i][2] = float(position.z);
//         }
//     }

//     while (t < T)
//     {

//         if (t < T / 2)
//         {
//             // 左1，左3，右2摆动 y方向运动Length~-Length z方向运动0~H
//             Position_Swing[0] = Swinging[flag][0];
//             Position_Swing[1] = Swinging[flag][1];
//             Position_Swing[2] = Swinging[flag][2];

//             // 右1，右3，左2支撑 y方向运动-Length~0.8*Length
//             Position_Support[0] = vector_Stand[0][0];
//             Position_Support[1] = -2 * Length * t + Length;
//             Position_Support[2] = vector_Stand[0][2];
//         }
//         else if (t < T && t >= T / 2)
//         {

//             if (flag >= step)
//             {
//                 flag = 0;
//             }
//             Position_Support[0] = Swinging[flag][0];
//             Position_Support[1] = Swinging[flag][1];
//             Position_Support[2] = Swinging[flag][2];
//             // Serial.println(Position_Swing[0]);
//             // Serial.println("");
//             // Serial.println(Position_Swing[1]);
//             // Serial.println("");
//             // Serial.println(Position_Swing[2]);
//             // Serial.println("");

//             Position_Swing[0] = vector_Stand[0][0];
//             Position_Swing[1] = -2 * Length * (t - T / 2) + Length;
//             Position_Swing[2] = vector_Stand[0][2];
//             // Serial.println(Position_Support[0]);
//             // Serial.println("");
//             // Serial.println(Position_Support[1]);
//             // Serial.println("");
//             // Serial.println(Position_Support[2]);
//             // Serial.println("");
//         }
//         else
//         {
//             t += 0.2;
//             continue;

//         }; // do nothing
//         for (size_t i = 0; i < AddedNumofLeg; i++)
//         {
//             LegConfig *TargetLeg;
//             xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
//             switch (i)
//             {
//             case 0:
//                 TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]+25);
//                 break;

//             case 1:
//                 TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]+25);
//                 break;
//             case 2:
//                 TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]+25);
//                 break;
//             case 3:
//                 TargetLeg->LegMoving(Position_Support[0] , Position_Support[1], Position_Support[2]-25);
//                 break;

//             case 4:
//                 TargetLeg->LegMoving(Position_Swing[0] , Position_Swing[1], Position_Swing[2]-25);
//                 break;
//             case 5:
//                 TargetLeg->LegMoving(Position_Support[0] , Position_Support[1], Position_Support[2]-25);
//                 break;
//             default:
//                 break;
//             }
//         }
//         delay(DSD);
//         t += 0.2;
//         flag += 2;
//     }
// }
/*下台阶*/
// void w_straight()
// {
//     int16_t DSD = 300;      // 每点间隔
//     float T = 2;            // 周期
//     float Length = 80;      // 步长
//     float H = Length - 0.1; //
//     float step = 10;        // 插值次数
//     float t = 0;            // 时间标志位
//     int flag = 0;           // 插值标志位

//     float Position_Swing[3];   // 摆动相位置暂存
//     float Position_Support[3]; // 支撑相位置暂存
//     float forward_Position[3];
//     float back_Position[3];
//     Vector3 p1 = {vector_Stand[0][0], Length, vector_Stand[0][2]};  // 相对当前位置向量圆弧起始点
//     Vector3 p2 = {vector_Stand[0][0], 0, vector_Stand[0][2] + H};   // 相对当前位置向量圆弧中间点
//     Vector3 p3 = {vector_Stand[0][0], -Length, vector_Stand[0][2]}; // 相对当前位置向量圆弧终点

//     Vector3 center;        // 圆弧圆心暂存
//     Vector3 position;      // 位置暂存
//     float Swinging[10][3]; // 摆动相位置
//     center = calculateArcCenter(p1, p2, p3);
//     for (int i = 0; i < 10; i++)
//     {
//         position = Invers_calculateTrack(p1, p3, center, step, i);

//         for (int j = 0; j < 3; j++)
//         {
//             Swinging[i][0] = float(position.x);
//             Swinging[i][1] = float(position.y);
//             Swinging[i][2] = float(position.z);
//         }
//     }

//     while (t < T)
//     {

//         if (t < T / 2)
//         {
//             // 左1，左3，右2摆动 y方向运动Length~-Length z方向运动0~H
//             Position_Swing[0] = Swinging[flag][0];
//             Position_Swing[1] = Swinging[flag][1];
//             Position_Swing[2] = Swinging[flag][2];

//             // 右1，右3，左2支撑 y方向运动-Length~0.8*Length
//             Position_Support[0] = vector_Stand[0][0];
//             Position_Support[1] = -2 * Length * t + Length;
//             Position_Support[2] = vector_Stand[0][2];
//         }
//         else if (t < T && t >= T / 2)
//         {

//             if (flag >= step)
//             {
//                 flag = 0;
//             }
//             Position_Support[0] = Swinging[flag][0];
//             Position_Support[1] = Swinging[flag][1];
//             Position_Support[2] = Swinging[flag][2];

//             Position_Swing[0] = vector_Stand[0][0];
//             Position_Swing[1] = -2 * Length * (t - T / 2) + Length;
//             Position_Swing[2] = vector_Stand[0][2];
//         }
//         else
//         {
//             t += 0.2;
//             continue;

//         }; // do nothing
//         for (size_t i = 0; i < AddedNumofLeg; i++)
//         {
//             LegConfig *TargetLeg;
//             xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
//             switch (i)
//             {
//             case 0:
//                 TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2] - 45);
//                 break;

//             case 1:
//                 TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2] + 35);
//                 break;
//             case 2:
//                 TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2] + 40);
//                 break;
//             case 3:
//                 TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2] - 45);
//                 break;

//             case 4:
//                 TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2] + 35);
//                 break;
//             case 5:
//                 TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2] + 40);
//                 break;
//             default:
//                 break;
//             }
//         }
//         delay(DSD);
//         t += 0.2;
//         flag += 2;
//     }
// }
void straight()
{
    int16_t DSD = 120;      // 每点间隔
    float T = 2;            // 周期
    float Length = 50;      // 步长
    float H = Length - 0.1; // 步高
    float step = 10;        // 插值次数
    float t = 0;            // 时间标志位
    int flag = 0;           // 插值标志位

    float Position_Swing[3];                                        // 摆动相位置暂存
    float Position_Support[3];                                      // 支撑相位置暂存
    Vector3 p1 = {vector_Stand[0][0], Length, vector_Stand[0][2]};  // 相对当前位置向量圆弧起始点
    Vector3 p2 = {vector_Stand[0][0], 0, vector_Stand[0][2] + H};   // 相对当前位置向量圆弧中间点
    Vector3 p3 = {vector_Stand[0][0], -Length, vector_Stand[0][2]}; // 相对当前位置向量圆弧终点

    Vector3 center;        // 圆弧圆心暂存
    Vector3 position;      // 位置暂存
    float Swinging[10][3]; // 摆动相位置
    center = calculateArcCenter(p1, p2, p3);
    for (int i = 0; i < 10; i++)
    {
        position = Invers_calculateTrack(p1, p3, center, step, i);

        for (int j = 0; j < 3; j++)
        {
            Swinging[i][0] = float(position.x);
            Swinging[i][1] = float(position.y);
            Swinging[i][2] = float(position.z);
        }
    }

    while (t < T)
    {

        if (t < T / 2)
        {
            // 左1，左3，右2摆动 y方向运动Length~-Length z方向运动0~H
            Position_Swing[0] = Swinging[flag][0];
            Position_Swing[1] = Swinging[flag][1];
            Position_Swing[2] = Swinging[flag][2];

            // 右1，右3，左2支撑 y方向运动-Length~0.8*Length
            Position_Support[0] = vector_Stand[0][0];
            Position_Support[1] = -2 * Length * t + Length;
            Position_Support[2] = vector_Stand[0][2];
        }
        else if (t < T && t >= T / 2)
        {

            if (flag >= step)
            {
                flag = 0;
            }
            Position_Support[0] = Swinging[flag][0];
            Position_Support[1] = Swinging[flag][1];
            Position_Support[2] = Swinging[flag][2];
            // Serial.println(Position_Swing[0]);
            // Serial.println("");
            // Serial.println(Position_Swing[1]);
            // Serial.println("");
            // Serial.println(Position_Swing[2]);
            // Serial.println("");

            Position_Swing[0] = vector_Stand[0][0];
            Position_Swing[1] = -2 * Length * (t - T / 2) + Length;
            Position_Swing[2] = vector_Stand[0][2];
            // Serial.println(Position_Support[0]);
            // Serial.println("");
            // Serial.println(Position_Support[1]);
            // Serial.println("");
            // Serial.println(Position_Support[2]);
            // Serial.println("");
        }
        else
        {
            t += 0.2;
            continue;

        }; // do nothing
        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegMoving(Position_Swing[0] * cos(3.14 / 4) + 30, Position_Swing[1], Position_Swing[2] + 20);
                break;

            case 1:
                TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2] + 20);
                break;
            case 2:
                TargetLeg->LegMoving(Position_Swing[0] * cos(3.14 / 4) + 30, Position_Swing[1], Position_Swing[2] + 20);
                break;
            case 3:
                TargetLeg->LegMoving(Position_Support[0] * cos(3.14 / 4) + 30, Position_Support[1], Position_Support[2] + 20);
                break;

            case 4:
                TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2] + 20);
                break;
            case 5:
                TargetLeg->LegMoving(Position_Support[0] * cos(3.14 / 4) + 30, Position_Support[1], Position_Support[2] + 20);
                break;
            default:
                break;
            }
        }

        delay(DSD);
        t += 0.2;
        flag += 2;
    }
    ControlExit_Flag = true;
}

void back()
{
    int16_t DSD = 120;      // 每点间隔
    float T = 2;            // 周期
    float Length = 60;      // 步长
    float H = Length - 0.1; // 步高
    float step = 10;        // 插值次数
    float t = 0;            // 时间标志位
    int flag = 0;           // 插值标志位

    float Position_Swing[3];                                        // 摆动相位置暂存
    float Position_Support[3];                                      // 支撑相位置暂存
    Vector3 p1 = {vector_Stand[0][0], Length, vector_Stand[0][2]};  // 相对当前位置向量圆弧起始点
    Vector3 p2 = {vector_Stand[0][0], 0, vector_Stand[0][2] + H};   // 相对当前位置向量圆弧中间点
    Vector3 p3 = {vector_Stand[0][0], -Length, vector_Stand[0][2]}; // 相对当前位置向量圆弧终点

    Vector3 center;        // 圆弧圆心暂存
    Vector3 position;      // 位置暂存
    float Swinging[10][3]; // 摆动相位置
    center = calculateArcCenter(p1, p2, p3);
    for (int i = 0; i < 10; i++)
    {
        position = Forward_calculateTrack(p1, p3, center, step, i);

        for (int j = 0; j < 3; j++)
        {
            Swinging[i][0] = float(position.x);
            Swinging[i][1] = float(position.y);
            Swinging[i][2] = float(position.z);
        }
    }

    while (t < T)
    {

        if (t < T / 2)
        {
            // 左1，左3，右2摆动 y方向运动Length~-Length z方向运动0~H
            Position_Swing[0] = Swinging[flag][0];
            Position_Swing[1] = Swinging[flag][1];
            Position_Swing[2] = Swinging[flag][2];
            // 右1，右3，左2支撑 y方向运动-Length~0.8*Length
            Position_Support[0] = vector_Stand[0][0];
            Position_Support[1] = 2 * Length * t - Length;
            Position_Support[2] = vector_Stand[0][2];
        }
        else if (t < T && t >= T / 2)
        {

            if (flag >= step)
            {
                flag = 0;
            }
            Position_Support[0] = Swinging[flag][0];
            Position_Support[1] = Swinging[flag][1];
            Position_Support[2] = Swinging[flag][2];

            Position_Swing[0] = vector_Stand[0][0];
            Position_Swing[1] = 2 * Length * (t - T / 2) - Length;
            Position_Swing[2] = vector_Stand[0][2];
        }
        else
        {
            t += 0.2;
            continue;

        }; // do nothing
        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegMoving(Position_Support[0] - 25, Position_Support[1], Position_Support[2]);
                break;

            case 1:
                TargetLeg->LegMoving(Position_Swing[0] - 25, Position_Swing[1], Position_Swing[2]);
                break;
            case 2:
                TargetLeg->LegMoving(Position_Support[0] - 25, Position_Support[1], Position_Support[2]);
                break;
            case 3:
                TargetLeg->LegMoving(Position_Swing[0] - 25, Position_Swing[1], Position_Swing[2]);
                break;

            case 4:
                TargetLeg->LegMoving(Position_Support[0], Position_Support[1], Position_Support[2]);
                break;
            case 5:
                TargetLeg->LegMoving(Position_Swing[0], Position_Swing[1], Position_Swing[2]);
                break;
            default:
                break;
            }
        }

        delay(DSD);
        t += 0.2;
        flag += 2;
    }
    ControlExit_Flag = true;
}

void straight_walk_task(void *pvParameters)
{
    // ROSConfig *Target = (ROSConfig *)pvParameters; // 接收对应LegConfig对象
    // while (1)
    // {
    straight();
    if (ControlExit_Flag)
    {
        Serial.println("[ControlExit]Control_Task is Over");
        vTaskResume(BLEServer_TaskHandle);
        ControlExit_Flag = false;
    }

    vTaskDelete(NULL);
}
void w_straight_walk_task(void *pvParameters)
{
    ROSConfig *Target = (ROSConfig *)pvParameters; // 接收对应LegConfig对象
    w_straight();
    // vTaskDelete(Target->Ros_TaskHandleHandle);
}
void back_walk_task(void *pvParameters)
{
    back();
    if (ControlExit_Flag)
    {
        Serial.println("[ControlExit]Control_Task is Over");
        vTaskResume(BLEServer_TaskHandle);
        ControlExit_Flag = false;
    }

    vTaskDelete(NULL);
}

void left()
{
    //(H<S/2)圆弧轨迹限制条件
    int16_t DSD = 120;      // 每点间隔
    float T = 2;            // 周期
    float Length = 60;      // 步长
    float H = Length - 0.1; // 步高
    float step = 10;        // 插值次数
    float t = 0;            // 时间标志位
    int flag = 0;           // 插值标志位

    float Left_Swing[3];                                            // 运动左侧摆动相位置暂存
    float Left_Support[3];                                          // 运动左侧支撑相位置暂存
    float Right_Swing[3];                                           // 运动右侧摆动相位置暂存
    float Right_Support[3];                                         // 运动右侧支撑相位置暂存
    Vector3 p1 = {vector_Stand[0][0], -Length, vector_Stand[0][2]}; // 相对当前位置向量圆弧起始点
    Vector3 p2 = {vector_Stand[0][0], 0, vector_Stand[0][2] + H};   // 相对当前位置向量圆弧中间点
    Vector3 p3 = {vector_Stand[0][0], Length, vector_Stand[0][2]};  // 相对当前位置向量圆弧终点

    Vector3 center;         // 圆弧圆心暂存
    Vector3 left_position;  // 位置暂存
    Vector3 right_position; // 位置暂存
    float left[10][3];
    float right[10][3];
    center = calculateArcCenter(p1, p2, p3);
    for (int i = 0; i < 10; i++)
    {
        left_position = Forward_calculateTrack(p1, p3, center, step, i);
        right_position = Invers_calculateTrack(p1, p3, center, step, i);

        for (int j = 0; j < 3; j++)
        {
            left[i][0] = float(left_position.x);
            left[i][1] = float(left_position.y);
            left[i][2] = float(left_position.z);

            right[i][0] = float(right_position.x);
            right[i][1] = float(right_position.y);
            right[i][2] = float(right_position.z);
        }
    }

    while (t < T)
    {

        if (t < T / 2)
        {

            Right_Swing[0] = left[flag][0];
            Right_Swing[1] = left[flag][1];
            Right_Swing[2] = left[flag][2];

            Left_Swing[0] = right[flag][0];
            Left_Swing[1] = right[flag][1];
            Left_Swing[2] = right[flag][2];

            Right_Support[0] = vector_Stand[0][0];
            Right_Support[1] = -2 * Length * t + Length;
            // Right_Support[1] = -Length;
            Right_Support[2] = vector_Stand[0][2];

            Left_Support[0] = vector_Stand[0][0];
            Left_Support[1] = 2 * Length * t - Length;
            // Left_Support[1] = Length;
            Left_Support[2] = vector_Stand[0][2];
        }
        else if (t < T && t >= T / 2)
        {
            if (flag >= step)
            {
                flag = 0;
            }

            Right_Support[0] = left[flag][0];
            Right_Support[1] = left[flag][1];
            Right_Support[2] = left[flag][2];

            Left_Support[0] = right[flag][0];
            Left_Support[1] = right[flag][1];
            Left_Support[2] = right[flag][2];

            Right_Swing[0] = vector_Stand[0][0];
            Right_Swing[1] = -2 * Length * (t - T / 2) + Length;
            // Right_Swing[1] = Length;
            Right_Swing[2] = vector_Stand[0][2];

            Left_Swing[0] = vector_Stand[0][0];
            Left_Swing[1] = 2 * Length * (t - T / 2) - Length;
            // Left_Swing[1] = -Length;
            Left_Swing[2] = vector_Stand[0][2];
        }
        else
        {
            t += 0.2;
            continue;
        }; // do nothing
        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegMoving(Left_Swing[0], Left_Swing[1], Left_Swing[2]);
                break;

            case 1:
                TargetLeg->LegMoving(Left_Support[0], Left_Support[1], Left_Support[2]);
                break;
            case 2:
                TargetLeg->LegMoving(Left_Swing[0], Left_Swing[1], Left_Swing[2]);
                break;
            case 3:
                TargetLeg->LegMoving(Right_Support[0], Right_Support[1], Right_Support[2]);
                break;

            case 4:
                TargetLeg->LegMoving(Right_Swing[0], Right_Swing[1], Right_Swing[2]);
                break;
            case 5:
                TargetLeg->LegMoving(Right_Support[0], Right_Support[1], Right_Support[2]);
                break;
            default:
                break;
            }
        }
        delay(DSD);
        t += 0.2;
        flag += 2;
    }
    ControlExit_Flag = true;
}

void right()
{
    //(H<S/2)圆弧轨迹限制条件
    int16_t DSD = 120;      // 每点间隔
    float T = 2;            // 周期
    float Length = 60;      // 步长
    float H = Length - 0.1; // 步高
    float step = 10;        // 插值次数
    float t = 0;            // 时间标志位
    int flag = 0;           // 插值标志位

    float Left_Swing[3];                                            // 运动左侧摆动相位置暂存
    float Left_Support[3];                                          // 运动左侧支撑相位置暂存
    float Right_Swing[3];                                           // 运动右侧摆动相位置暂存
    float Right_Support[3];                                         // 运动右侧支撑相位置暂存
    Vector3 p1 = {vector_Stand[0][0], -Length, vector_Stand[0][2]}; // 相对当前位置向量圆弧起始点
    Vector3 p2 = {vector_Stand[0][0], 0, vector_Stand[0][2] + H};   // 相对当前位置向量圆弧中间点
    Vector3 p3 = {vector_Stand[0][0], Length, vector_Stand[0][2]};  // 相对当前位置向量圆弧终点

    Vector3 center;         // 圆弧圆心暂存
    Vector3 left_position;  // 位置暂存
    Vector3 right_position; // 位置暂存
    float left[10][3];
    float right[10][3];
    center = calculateArcCenter(p1, p2, p3);
    for (int i = 0; i < 10; i++)
    {
        left_position = Forward_calculateTrack(p1, p3, center, step, i);
        right_position = Invers_calculateTrack(p1, p3, center, step, i);

        for (int j = 0; j < 3; j++)
        {
            left[i][0] = float(left_position.x);
            left[i][1] = float(left_position.y);
            left[i][2] = float(left_position.z);

            right[i][0] = float(right_position.x);
            right[i][1] = float(right_position.y);
            right[i][2] = float(right_position.z);
        }
    }

    while (t < T)
    {

        if (t < T / 2)
        {

            Right_Swing[0] = right[flag][0];
            Right_Swing[1] = right[flag][1];
            Right_Swing[2] = right[flag][2];

            Left_Swing[0] = left[flag][0];
            Left_Swing[1] = left[flag][1];
            Left_Swing[2] = left[flag][2];

            Right_Support[0] = vector_Stand[0][0];
            Right_Support[1] = 2 * Length * t - Length;
            // Right_Support[1] = -Length;
            Right_Support[2] = vector_Stand[0][2];

            Left_Support[0] = vector_Stand[0][0];
            Left_Support[1] = -2 * Length * t + Length;
            // Left_Support[1] = Length;
            Left_Support[2] = vector_Stand[0][2];
        }
        else if (t < T && t >= T / 2)
        {
            if (flag >= step)
            {
                flag = 0;
            }

            Right_Support[0] = right[flag][0];
            Right_Support[1] = right[flag][1];
            Right_Support[2] = right[flag][2];

            Left_Support[0] = left[flag][0];
            Left_Support[1] = left[flag][1];
            Left_Support[2] = left[flag][2];

            Right_Swing[0] = vector_Stand[0][0];
            Right_Swing[1] = 2 * Length * (t - T / 2) - Length;
            // Right_Swing[1] = Length;
            Right_Swing[2] = vector_Stand[0][2];

            Left_Swing[0] = vector_Stand[0][0];
            Left_Swing[1] = -2 * Length * (t - T / 2) + Length;
            // Left_Swing[1] = -Length;
            Left_Swing[2] = vector_Stand[0][2];
        }
        else
        {
            t += 0.2;
            continue;
        }; // do nothing
        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegMoving(Left_Support[0], Left_Support[1], Left_Support[2]);
                break;

            case 1:
                TargetLeg->LegMoving(Left_Swing[0], Left_Swing[1], Left_Swing[2]);
                break;
            case 2:
                TargetLeg->LegMoving(Left_Support[0], Left_Support[1], Left_Support[2]);
                break;
            case 3:
                TargetLeg->LegMoving(Left_Swing[0], Left_Swing[1], Left_Swing[2]);
                break;

            case 4:
                TargetLeg->LegMoving(Left_Support[0], Left_Support[1], Left_Support[2]);
                break;
            case 5:
                TargetLeg->LegMoving(Left_Swing[0], Left_Swing[1], Left_Swing[2]);
                break;
            default:
                break;
            }
        }
        delay(DSD);
        t += 0.2;
        flag += 2;
    }
    ControlExit_Flag = true;
}
void left_walk_task(void *pvParameters)
{
    left();
    if (ControlExit_Flag)
    {
        Serial.println("[ControlExit]Control_Task is Over");
        vTaskResume(BLEServer_TaskHandle);
        ControlExit_Flag = false;
    }
    vTaskDelete(NULL);
}

void right_walk_task(void *pvParameters)
{
    right();
    if (ControlExit_Flag)
    {
        Serial.println("[ControlExit]Control_Task is Over");
        vTaskResume(BLEServer_TaskHandle);
        ControlExit_Flag = false;
    }

    vTaskDelete(NULL);
}
// work
void left_cross()
{
    int16_t DSD = 120;      // 每点间隔
    float T = 2;            // 周期
    float Length = 60;      // 步长
    float H = Length - 0.1; // 步高
    float step = 10;        // 插值次数
    float t = 0;            // 时间标志位
    int flag = 0;           // 插值标志位

    float Left_Swing[3];    // 运动左侧摆动相位置暂存
    float Left_Support[3];  // 运动左侧支撑相位置暂存
    float Right_Swing[3];   // 运动右侧摆动相位置暂存
    float Right_Support[3]; // 运动右侧支撑相位置暂存

    Vector3 p1 = {vector_Stand[0][0] + Length, 0, vector_Stand[0][2]}; // 相对当前位置向量圆弧起始点
    Vector3 p2 = {vector_Stand[0][0], 0, vector_Stand[0][2] + H};      // 相对当前位置向量圆弧中间点
    Vector3 p3 = {vector_Stand[0][0] - Length, 0, vector_Stand[0][2]}; // 相对当前位置向量圆弧终点
    Vector3 center;                                                    // 圆弧圆心暂存
    Vector3 left_position;                                             // 位置暂存
    Vector3 right_position;                                            // 位置暂存
    float left[10][3];
    float right[10][3];
    center = calculateArcCenter(p1, p2, p3);
    for (int i = 0; i < 10; i++)
    {
        left_position = Invers_calculateTrack(p1, p3, center, step, i);
        right_position = Forward_calculateTrack(p1, p3, center, step, i);

        for (int j = 0; j < 3; j++)
        {
            left[i][0] = left_position.x;
            left[i][1] = left_position.y;
            left[i][2] = left_position.z;

            right[i][0] = right_position.x;
            right[i][1] = right_position.y;
            right[i][2] = right_position.z;
        }
    }

    while (t < T)
    {

        if (t < T / 2)
        {
            // 左1，左3摆动
            Left_Swing[0] = left[flag][0];
            Left_Swing[1] = left[flag][1];
            Left_Swing[2] = left[flag][2];
            // 右2摆动
            Right_Swing[0] = right[flag][0];
            Right_Swing[1] = right[flag][1];
            Right_Swing[2] = right[flag][2];
            // 左2支撑
            Left_Support[0] = (-2 * Length) * t + Length + vector_Stand[0][0];
            Left_Support[1] = 0;
            Left_Support[2] = vector_Stand[0][2];
            // 右1,右3支撑
            Right_Support[0] = 2 * Length * t - Length + vector_Stand[0][0];
            Right_Support[1] = 0;
            Right_Support[2] = vector_Stand[0][2];
        }
        else if (t < T && t >= T / 2)
        {
            if (flag >= step)
            {
                flag = 0;
            }
            // 左2摆动
            Left_Support[0] = left[flag][0];
            Left_Support[1] = left[flag][1];
            Left_Support[2] = left[flag][2];
            // 右1，右3摆动
            Right_Support[0] = right[flag][0];
            Right_Support[1] = right[flag][1];
            Right_Support[2] = right[flag][2];
            // 左1，左3支撑
            Left_Swing[0] = -2 * Length * (t - T / 2) + Length + vector_Stand[0][0];
            Left_Swing[1] = 0;
            Left_Swing[2] = vector_Stand[0][2];
            // 右2支撑
            Right_Swing[0] = 2 * Length * (t - T / 2) - Length + vector_Stand[0][0];
            Right_Swing[1] = 0;
            Right_Swing[2] = vector_Stand[0][2];
        }
        else
        {
            t += 0.2;
            continue;
        }; // do nothing
        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegMoving(Left_Swing[0], Left_Swing[1], Left_Swing[2]);
                break;

            case 1:
                TargetLeg->LegMoving(Left_Support[0], Left_Support[1], Left_Support[2]);
                break;
            case 2:
                TargetLeg->LegMoving(Left_Swing[0], Left_Swing[1], Left_Swing[2]);
                break;
            case 3:
                TargetLeg->LegMoving(Right_Support[0], Right_Support[1], Right_Support[2]);
                break;

            case 4:
                TargetLeg->LegMoving(Right_Swing[0], Right_Swing[1], Right_Swing[2]);
                break;
            case 5:
                TargetLeg->LegMoving(Right_Support[0], Right_Support[1], Right_Support[2]);
                break;
            default:
                break;
            }
        }
        delay(DSD);
        t += 0.2;
        flag += 2;
    }
    ControlExit_Flag = true;
}

void left_cross_walk_task(void *pvParameters)
{
    left_cross();
    if (ControlExit_Flag)
    {
        Serial.println("[ControlExit]Control_Task is Over");
        vTaskResume(BLEServer_TaskHandle);
        ControlExit_Flag = false;
    }
    vTaskDelete(NULL);
}

void right_cross()
{
    int16_t DSD = 120;      // 每点间隔
    float T = 2;            // 周期
    float Length = 60;      // 步长
    float H = Length - 0.1; // 步高
    float step = 4;         // 插值次数
    float t = 0;            // 时间标志位
    int flag = 0;           // 插值标志位

    float Left_Swing[3];    // 运动左侧摆动相位置暂存
    float Left_Support[3];  // 运动左侧支撑相位置暂存
    float Right_Swing[3];   // 运动右侧摆动相位置暂存
    float Right_Support[3]; // 运动右侧支撑相位置暂存

    Vector3 p1 = {vector_Stand[0][0] + Length, 0, vector_Stand[0][2]}; // 相对当前位置向量圆弧起始点
    Vector3 p2 = {vector_Stand[0][0], 0, vector_Stand[0][2] + H};      // 相对当前位置向量圆弧中间点
    Vector3 p3 = {vector_Stand[0][0] - Length, 0, vector_Stand[0][2]}; // 相对当前位置向量圆弧终点

    Vector3 center;         // 圆弧圆心暂存
    Vector3 left_position;  // 位置暂存
    Vector3 right_position; // 位置暂存
    float left[5][3];
    float right[5][3];
    center = calculateArcCenter(p1, p2, p3);
    for (int i = 0; i < 5; i++)
    {
        left_position = Invers_calculateTrack(p1, p3, center, step, i);
        right_position = Forward_calculateTrack(p1, p3, center, step, i);

        for (int j = 0; j < 3; j++)
        {
            left[i][0] = left_position.x;
            left[i][1] = left_position.y;
            left[i][2] = left_position.z;

            right[i][0] = right_position.x;
            right[i][1] = right_position.y;
            right[i][2] = right_position.z;
        }
    }

    while (t < T)
    {

        if (t < T / 2)
        {
            // 左1，左3摆动
            Right_Swing[0] = left[flag][0];
            Right_Swing[1] = left[flag][1];
            Right_Swing[2] = left[flag][2];
            // 右2摆动
            Left_Swing[0] = right[flag][0];
            Left_Swing[1] = right[flag][1];
            Left_Swing[2] = right[flag][2];
            // 左2支撑
            Right_Support[0] = (-2 * Length) * t + Length + vector_Stand[0][0];
            Right_Support[1] = 0;
            Right_Support[2] = vector_Stand[0][2];
            // 右1,右3支撑
            Left_Support[0] = 2 * Length * t - Length + vector_Stand[0][0];
            Left_Support[1] = 0;
            Left_Support[2] = vector_Stand[0][2];
        }
        else if (t < T && t > T / 2)
        {
            if (flag > 4)
            {
                flag = 0;
            }
            // 左2摆动
            Right_Support[0] = left[flag][0];
            Right_Support[1] = left[flag][1];
            Right_Support[2] = left[flag][2];
            // 右1，右3摆动
            Left_Support[0] = right[flag][0];
            Left_Support[1] = right[flag][1];
            Left_Support[2] = right[flag][2];
            // 左1，左3支撑
            Right_Swing[0] = -2 * Length * (t - T / 2) + Length + vector_Stand[0][0];
            Right_Swing[1] = 0;
            Right_Swing[2] = vector_Stand[0][2];
            // 右2支撑
            Left_Swing[0] = 2 * Length * (t - T / 2) - Length + vector_Stand[0][0];
            Left_Swing[1] = 0;
            Left_Swing[2] = vector_Stand[0][2];
        }
        else
        {
            t += 0.2;
            continue;
        }; // do nothing
        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegMoving(Left_Swing[0], Left_Swing[1], Left_Swing[2]);
                break;

            case 1:
                TargetLeg->LegMoving(Left_Support[0], Left_Support[1], Left_Support[2]);
                break;
            case 2:
                TargetLeg->LegMoving(Left_Swing[0], Left_Swing[1], Left_Swing[2]);
                break;
            case 3:
                TargetLeg->LegMoving(Right_Support[0], Right_Support[1], Right_Support[2]);
                break;

            case 4:
                TargetLeg->LegMoving(Right_Swing[0], Right_Swing[1], Right_Swing[2]);
                break;
            case 5:
                TargetLeg->LegMoving(Right_Support[0], Right_Support[1], Right_Support[2]);
                break;
            default:
                break;
            }
        }
        delay(DSD);
        t += 0.2;
        flag++;
    }
    ControlExit_Flag = true;
}
void right_cross_walk_task(void *pvParameters)
{
    right_cross();
    if (ControlExit_Flag)
    {
        Serial.println("[ControlExit]Control_Task is Over");
        vTaskResume(BLEServer_TaskHandle);
        ControlExit_Flag = false;
    }
    vTaskDelete(NULL);
}
FSUS_SERVO_ANGLE_T S_angle_group1[4][3] = {{-45, 35, 15},
                                           {-45, 0, 0},
                                           {0, 0, 0},
                                           {0, 0, 0}};
FSUS_SERVO_ANGLE_T S_angle_group2[4][3] = {{0, -55, -35},
                                           {0, -55, -35},
                                           {0, -55, -15},
                                           {0, 0, 0}};
FSUS_SERVO_ANGLE_T S_angle_group3[4][3] = {{45, 35, 15},
                                           {45, 0, 0},
                                           {0, 0, 0},
                                           {0, 0, 0}};
void stand()
{
    for (int j = 0; j < 4; j++)
    {
        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegSetAngle(S_angle_group1[j][0] + defaultLeg1HipAngle, S_angle_group1[j][1] + defaultLeg1KneeAngle, S_angle_group1[j][2] + defaultLeg1AnkleAngle, 1000);
                break;
            case 1:
                TargetLeg->LegSetAngle(S_angle_group2[j][0] + defaultLeg2HipAngle, S_angle_group2[j][1] + defaultLeg2KneeAngle, S_angle_group2[j][2] + defaultLeg2AnkleAngle, 1000);
                break;
            case 2:
                TargetLeg->LegSetAngle(S_angle_group3[j][0] + defaultLeg3HipAngle, S_angle_group3[j][1] + defaultLeg3KneeAngle, S_angle_group3[j][2] + defaultLeg3AnkleAngle, 1000);
                break;
            case 3:
                TargetLeg->LegSetAngle(S_angle_group1[j][0] + defaultLeg4HipAngle, S_angle_group1[j][1] + defaultLeg4KneeAngle, S_angle_group1[j][2] + defaultLeg4AnkleAngle, 1000);
                break;

            case 4:
                TargetLeg->LegSetAngle(S_angle_group2[j][0] + defaultLeg5HipAngle, S_angle_group2[j][1] + defaultLeg5KneeAngle, S_angle_group2[j][2] + defaultLeg5AnkleAngle, 1000);
                break;
            case 5:
                TargetLeg->LegSetAngle(S_angle_group3[j][0] + defaultLeg6HipAngle, S_angle_group3[j][1] + defaultLeg6KneeAngle, S_angle_group3[j][2] + defaultLeg6AnkleAngle, 1000);
                break;
            default:
                break;
            }
        }
        vTaskDelay(500);
    }
    // for (uint8_t j = 0; j < 2; j++)
    // {
    //     if (j == 0)
    //     {
    //         for (size_t i = 0; i < AddedNumofLeg; i++)
    //         {
    //             LegConfig *TargetLeg;
    //             xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
    //             switch (i)
    //             {
    //             case 0:
    //                 TargetLeg->LegSetAngle(defaultLeg1HipAngle, -30 + defaultLeg1KneeAngle, -30 + defaultLeg1AnkleAngle, 1500);
    //                 break;
    //             case 1:
    //                 TargetLeg->LegSetAngle(defaultLeg2HipAngle, -30 + defaultLeg2KneeAngle, -30 + defaultLeg2AnkleAngle, 1500);
    //                 break;
    //             case 2:
    //                 TargetLeg->LegSetAngle(defaultLeg3HipAngle, -30 + defaultLeg3KneeAngle, -30 + defaultLeg3AnkleAngle, 1500);
    //                 break;
    //             case 3:
    //                 TargetLeg->LegSetAngle(defaultLeg4HipAngle, -30 + defaultLeg4KneeAngle, -30 + defaultLeg4AnkleAngle, 1500);
    //                 break;

    //             case 4:
    //                 TargetLeg->LegSetAngle(defaultLeg5HipAngle, -30 + defaultLeg5KneeAngle, -30 + defaultLeg5AnkleAngle, 1500);
    //                 break;
    //             case 5:
    //                 TargetLeg->LegSetAngle(defaultLeg6HipAngle, -30 + defaultLeg6KneeAngle, -30 + defaultLeg6AnkleAngle, 1500);
    //                 break;
    //             default:
    //                 break;
    //             }
    //         }
    //     }
    //     else
    //     {
    //         for (size_t i = 0; i < AddedNumofLeg; i++)
    //         {
    //             LegConfig *TargetLeg;
    //             xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
    //             switch (i)
    //             {
    //             case 0:
    //                 TargetLeg->LegSetAngle(defaultLeg1HipAngle, defaultLeg1KneeAngle, defaultLeg1AnkleAngle, 1000);
    //                 break;
    //             case 1:
    //                 TargetLeg->LegSetAngle(defaultLeg2HipAngle, defaultLeg2KneeAngle, defaultLeg2AnkleAngle, 1000);
    //                 break;
    //             case 2:
    //                 TargetLeg->LegSetAngle(defaultLeg3HipAngle, defaultLeg3KneeAngle, defaultLeg3AnkleAngle, 1000);
    //                 break;
    //             case 3:
    //                 TargetLeg->LegSetAngle(defaultLeg4HipAngle, defaultLeg4KneeAngle, defaultLeg4AnkleAngle, 1000);
    //                 break;

    //             case 4:
    //                 TargetLeg->LegSetAngle(defaultLeg5HipAngle, defaultLeg5KneeAngle, defaultLeg4AnkleAngle, 1000);
    //                 break;
    //             case 5:
    //                 TargetLeg->LegSetAngle(defaultLeg6HipAngle, defaultLeg6KneeAngle, defaultLeg6AnkleAngle, 1000);
    //                 break;
    //             default:
    //                 break;
    //             }
    //         }
    //     }
    // }
    ControlExit_Flag = true;
}
void Stand_Task(void *pvParameters)
{
    stand();
    if (ControlExit_Flag)
    {
        vTaskResume(BLEServer_TaskHandle);
    }
    vTaskDelete(NULL);
}
void car()
{

    for (uint8_t flag = 0; flag < 4; flag++)
    {
        if (flag == 0)
        {
            for (size_t i = 0; i < AddedNumofLeg; i++)
            {
                LegConfig *TargetLeg;
                xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
                switch (i)
                {
                case 0:
                    TargetLeg->LegSetAngle(defaultLeg1HipAngle, defaultLeg1KneeAngle, defaultLeg1AnkleAngle, 1000);
                    break;
                case 1:
                    TargetLeg->LegSetAngle(defaultLeg2HipAngle, defaultLeg2KneeAngle, defaultLeg2AnkleAngle, 1000);
                    break;
                case 2:
                    TargetLeg->LegSetAngle(defaultLeg3HipAngle, defaultLeg3KneeAngle, defaultLeg3AnkleAngle, 1000);
                    break;
                case 3:
                    TargetLeg->LegSetAngle(defaultLeg4HipAngle, defaultLeg4KneeAngle, defaultLeg4AnkleAngle, 1000);
                    break;

                case 4:
                    TargetLeg->LegSetAngle(defaultLeg5HipAngle, defaultLeg5KneeAngle, defaultLeg5AnkleAngle, 1000);
                    break;
                case 5:
                    TargetLeg->LegSetAngle(defaultLeg6HipAngle, defaultLeg6KneeAngle, defaultLeg6AnkleAngle, 1000);
                    break;
                default:
                    break;
                }
            }
            // vTaskDelay(500);
        }
        if (flag == 1)
        {
            for (size_t i = 0; i < AddedNumofLeg; i++)
            {
                LegConfig *TargetLeg;
                xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
                switch (i)
                {
                case 0:
                    TargetLeg->LegSetAngle(-45 + defaultLeg1HipAngle, defaultLeg1KneeAngle, defaultLeg1AnkleAngle, 1000);
                    break;
                case 1:
                    TargetLeg->LegSetAngle(0 + defaultLeg2HipAngle, defaultLeg2KneeAngle, defaultLeg2AnkleAngle, 1000);
                    break;
                case 2:
                    TargetLeg->LegSetAngle(45 + defaultLeg3HipAngle, defaultLeg3KneeAngle, defaultLeg3AnkleAngle, 1000);
                    break;
                case 3:
                    TargetLeg->LegSetAngle(-45 + defaultLeg4HipAngle, defaultLeg4KneeAngle, defaultLeg4AnkleAngle, 1000);
                    break;

                case 4:
                    TargetLeg->LegSetAngle(0 + defaultLeg5HipAngle, defaultLeg5KneeAngle, defaultLeg5AnkleAngle, 1000);
                    break;
                case 5:
                    TargetLeg->LegSetAngle(45 + defaultLeg6HipAngle, defaultLeg6KneeAngle, defaultLeg6AnkleAngle, 1000);
                    break;
                default:
                    break;
                }
            }
            // vTaskDelay(500);
        }
        if (flag == 2)
        {
            for (size_t i = 0; i < AddedNumofLeg; i++)
            {
                LegConfig *TargetLeg;
                xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
                switch (i)
                {
                case 0:
                    TargetLeg->LegSetAngle(-45 + defaultLeg1HipAngle, -55 + defaultLeg1KneeAngle, -35 + defaultLeg1AnkleAngle, 2000);
                    break;
                case 1:
                    TargetLeg->LegSetAngle(0 + defaultLeg2HipAngle, -35 + defaultLeg2KneeAngle, 50 + defaultLeg2AnkleAngle, 2000);
                    break;
                case 2:
                    TargetLeg->LegSetAngle(45 + defaultLeg3HipAngle, -55 + defaultLeg3KneeAngle, -35 + defaultLeg3AnkleAngle, 2000);
                    break;
                case 3:
                    TargetLeg->LegSetAngle(-45 + defaultLeg4HipAngle, -55 + defaultLeg4KneeAngle, -35 + defaultLeg4AnkleAngle, 2000);
                    break;

                case 4:
                    TargetLeg->LegSetAngle(0 + defaultLeg5HipAngle, -35 + defaultLeg5KneeAngle, 50 + defaultLeg5AnkleAngle, 2000);
                    break;
                case 5:
                    TargetLeg->LegSetAngle(45 + defaultLeg6HipAngle, -55 + defaultLeg6KneeAngle, -35 + defaultLeg6AnkleAngle, 2000);
                    break;
                default:
                    break;
                }
            }
            // vTaskDelay(500);
        }
        if (flag == 3)
        {

            for (size_t i = 0; i < AddedNumofLeg; i++)
            {
                LegConfig *TargetLeg;
                xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
                switch (i)
                {
                case 0:
                    TargetLeg->LegSetAngle(-45 + defaultLeg1HipAngle, -65 + defaultLeg1KneeAngle, 30 + defaultLeg1AnkleAngle, 1000);
                    break;
                case 1:
                    TargetLeg->LegSetAngle(0 + defaultLeg2HipAngle, -65 + defaultLeg2KneeAngle, 25 + defaultLeg2AnkleAngle, 1000);
                    break;
                case 2:
                    TargetLeg->LegSetAngle(45 + defaultLeg3HipAngle, -65 + defaultLeg3KneeAngle, 30 + defaultLeg3AnkleAngle, 1000);
                    break;
                case 3:
                    TargetLeg->LegSetAngle(-45 + defaultLeg4HipAngle, -65 + defaultLeg4KneeAngle, 30 + defaultLeg4AnkleAngle, 1000);
                    break;

                case 4:
                    TargetLeg->LegSetAngle(0 + defaultLeg5HipAngle, -65 + defaultLeg5KneeAngle, 25 + defaultLeg5AnkleAngle, 1000);
                    break;
                case 5:
                    TargetLeg->LegSetAngle(45 + defaultLeg6HipAngle, -65 + defaultLeg6KneeAngle, 30 + defaultLeg6AnkleAngle, 1000);
                    break;
                default:
                    break;
                }
            }
        }
    }
    ControlExit_Flag = true;
}
void car_Task(void *pvParameters)
{

    car();
    if (ControlExit_Flag = true)
    {
        vTaskResume(BLEServer_TaskHandle);
        ControlExit_Flag = false;
    }
    vTaskDelete(NULL);
}
FSUS_SERVO_ANGLE_T angle_group1[4][3] = {{45, 0, 0},
                                         {45, 35, 15},
                                         {45, -65, 30},
                                         {-45, -65, 30}};
FSUS_SERVO_ANGLE_T angle_group2[4][3] = {{0, 0, 0},
                                         {0, 35, 30},
                                         {0, 65, 50},
                                         {0, 65, 50}};
FSUS_SERVO_ANGLE_T angle_group3[4][3] = {{-30, 0, 0},
                                         {-30, 35, 15},
                                         {-30, -65, 30},
                                         {45, -65, 30}};
void car2()
{
    for (int j = 0; j < 4; j++)
    {
        for (size_t i = 0; i < AddedNumofLeg; i++)
        {
            LegConfig *TargetLeg;
            xQueuePeek(LegQueue[i], &TargetLeg, portMAX_DELAY);
            switch (i)
            {
            case 0:
                TargetLeg->LegSetAngle(angle_group1[j][0] + defaultLeg1HipAngle, angle_group1[j][1] + defaultLeg1KneeAngle, angle_group1[j][2] + defaultLeg1AnkleAngle, 1000);
                break;
            case 1:
                TargetLeg->LegSetAngle(angle_group2[j][0] + defaultLeg2HipAngle, angle_group2[j][1] + defaultLeg2KneeAngle, angle_group2[j][2] + defaultLeg2AnkleAngle, 1000);
                break;
            case 2:
                TargetLeg->LegSetAngle(angle_group3[j][0] + defaultLeg3HipAngle, angle_group3[j][1] + defaultLeg3KneeAngle, angle_group3[j][2] + defaultLeg3AnkleAngle, 1000);
                break;
            case 3:
                TargetLeg->LegSetAngle(angle_group1[j][0] + defaultLeg4HipAngle, angle_group1[j][1] + defaultLeg4KneeAngle, angle_group1[j][2] + defaultLeg4AnkleAngle, 1000);
                break;

            case 4:
                TargetLeg->LegSetAngle(angle_group2[j][0] + defaultLeg5HipAngle, angle_group2[j][1] + defaultLeg5KneeAngle, angle_group2[j][2] + defaultLeg5AnkleAngle, 1000);
                break;
            case 5:
                TargetLeg->LegSetAngle(angle_group3[j][0] + defaultLeg6HipAngle, angle_group3[j][1] + defaultLeg6KneeAngle, angle_group1[j][2] + defaultLeg6AnkleAngle, 1000);
                break;
            default:
                break;
            }
        }
        vTaskDelay(500);
    }
    ControlExit_Flag = true;
}
void car2_Task(void *pvParameters)
{
    car2();
    if (ControlExit_Flag)
    {
        Serial.println("[ControlExit]Control_Task is Over");
        vTaskResume(BLEServer_TaskHandle);
        ControlExit_Flag = false;
    }
    vTaskDelete(NULL);
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