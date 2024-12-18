#ifndef _BLE_INIT_H_
#define _BLE_INIT_H_
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEService.h>
#include <BLECharacteristic.h>
#include <Arduino.h>
#include <Servo.h>

#define SERVICE_UUID "b408e1a0-3d8a-11ed-b878-0242ac120002"        // 服务UUID
#define CHARACTERISTIC_UUID "dfd84064-3d8a-11ed-b878-0242ac120002" // 特征UUID
#define CONTROL_UUID "de045162-3d97-11ed-b878-0242ac120002"        // 控制特征UUID

extern TaskHandle_t BLEServer_TaskHandle;
extern TaskHandle_t BLEMainLogic_TaskHandle;
extern TaskHandle_t Control_TaskHandle;
extern BLECharacteristic controlCharacteristic;
// extern char state;
extern bool connected_state;
extern bool flag;
extern char state;
extern const char default_state;
// extern bool connected_state;

void BLE_Init();
void BLEServer_Task(void *pvParam);
void BLEMainLogic_Task(void *pvParam);
#endif