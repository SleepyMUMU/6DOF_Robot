// #include <JY901S.h>

// void JY901S::JY901S_Init()
// {
//     Wire.begin();              // Start the I2C interface
//     Wire.setClock(IIC_Speend); // Set the I2C clock speed
//     WitInit(WIT_PROTOCOL_I2C,0x50);//初始化I2C
//     WitI2cFuncRegister(write_func, read_func);//注册I2C函数
//     WitRegisterCallBack(CopeSensorData);//注册获取传感器数据回调函数
//     WitDelayMsRegister(Delayms);//注册延时函数'
//     AutoScanSensor();//自动校准
// }