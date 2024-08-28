// #ifndef _BLE_INIT_H_
// #define _BLE_INIT_H_
// #include <BLEDevice.h>
// #include <BLEServer.h>
// #include <BLEUtils.h>
// #include <BLE2902.h>
// #include <Arduino.h>

// class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
// {
//   void onResult(BLEAdvertisedDevice advertisedDevice)
//     {
//     Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
//   }
// };
// class BLE_Config : public BLEDevice, public BLEServer, public BLEService, public BLECharacteristic, public BLEScan
// {
//   BLEScan *pBLEScan;                                  // BLE扫描
//   BLEServer *pServer;                                 // BLE服务端
//   String SERVICE_UUID;                                // 服务UUID
//   String CHARACTERISTIC_UUID;                         // 特征UUID
//   String DeviceName;                                  // 设备密码
//   String DeviceValue;                                 // 设备值
//   BLE_Config();                                       // 默认使用BLE客户端
//   BLE_Config(String DeviceName, String SERVICE_UUID); // BLE服务端
//   ~BLE_Config();
// };

// #endif