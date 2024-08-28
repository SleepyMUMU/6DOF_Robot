// #include <BLE_Init.h>
// // #define SERVICE_UUID "7cc6a5c1-e646-4acc-b349-0c511336f439"
// // #define CHARACTERISTIC_UUID "938e4227-073a-46d8-8508-ce58075c1d8a"

// BLE_Config ::BLE_Config()
// {
//     BLEDevice::init("SIX_RBOT");                                               // 初始化BLE设备
//     pBLEScan = BLEDevice::getScan();                                           // 获取BLE扫描
//     pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks()); // 设置回调函数
//     pBLEScan->setActiveScan(true);                                             // 使用更多电力，更快获得结果
//     pBLEScan->setInterval(100);                                                // 设置扫描间隔
//     pBLEScan->setWindow(99);                                                   // 设置扫描窗口,需要小于扫描间隔
//     BLEScanResults foundDevices = pBLEScan->start(5, false);                   // 扫描5秒
//     Serial.println(foundDevices.getCount());                                   // BLE客户端准备连接
//     Serial.println("Devices found:");
//     for (int i = 0; i < foundDevices.getCount(); i++)
//     {
//         BLEAdvertisedDevice device = foundDevices.getDevice(i);
//         Serial.println("Device found: " + device.toString());
//     }
//     pBLEScan->clearResults(); // 清除结果
// }
// BLE_Config::BLE_Config(std::string DeviceName, std::string SERVICE_UUID)
// {
//     this->DeviceName = DeviceName;
//     this->SERVICE_UUID = SERVICE_UUID;
//     BLEServer *pServer = BLEDevice::createServer();                      // 创建BLE服务实例
//     BLEService *pService = pServer->createService(SERVICE_UUID);         // 创建BLE服务
//     BLECharacteristic *pCharacteristic = pService->createCharacteristic( // 创建BLE特征值
//         CHARACTERISTIC_UUID,                                             // 特征值UUID
//         BLECharacteristic::PROPERTY_READ |                               // 特征值可读
//             BLECharacteristic::PROPERTY_WRITE |                          // 特征值可写
//             BLECharacteristic::PROPERTY_NOTIFY |                         // 特征值可通知
//             BLECharacteristic::PROPERTY_INDICATE);                       // 特征值可指示
//     pService->start();                                                   // 启动BLE服务
//     pServer->getAdvertising()->start();                                  // 获取BLE广播
//     Serial.println("The BLE Server is ready to be connected");
// }

// // BLE_Config::~BLE_Config()
// // {
// //     Serial.println("The BLE Server is disconnected");
// // }