#include <BLE_Init.h>

bool flag = true;
char state = 48;
const char default_state = 48;
bool connected_state = false; // 创建设备连接标识符
TaskHandle_t Control_TaskHandle = NULL;
TaskHandle_t BLEServer_TaskHandle = NULL;
TaskHandle_t BLEMainLogic_TaskHandle = NULL;

// #define SERVICE_UUID "7cc6a5c1-e646-4acc-b349-0c511336f439"
// #define CHARACTERISTIC_UUID "938e4227-073a-46d8-8508-ce58075c1d8a"
// BLECharacteristic controlCharacteristic(CONTROL_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE); // 控制特征
BLECharacteristic controlCharacteristic(CONTROL_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY); // 控制特征
// 连接和断开连接的回调函数
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        connected_state = true;
        Serial.println("Client connected");
    };

    void onDisconnect(BLEServer *pServer)
    {
        connected_state = false;
        Serial.println("Client disconnected");
    }
};

void BLE_Init()
{
    BLEDevice::init("MUMU"); // 初始化BLE设备
    BLEServer *pServer = BLEDevice::createServer();
    // 创建BLE服务
    BLEService *pService = pServer->createService(SERVICE_UUID);                                                                                                             // 使用服务UUID创建服务
    BLECharacteristic *nameCharacteristic = pService->createCharacteristic(BLEUUID((uint16_t)0x2A00), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE); // 创建特征
    nameCharacteristic->setValue("Control");                                                                                                                                 // 设置特征值
    pService->addCharacteristic(&controlCharacteristic);                                                                                                                     // 添加特征
    controlCharacteristic.setValue(&state);
    // 设置特征， 使用上面的特征UUID，需要将特征的属性作为参数传递。此情况下是读或写
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    pCharacteristic->setValue("Hello HWD!!!"); // 创建完特征后，可以使用setValue()方法为其在此赋值
    pServer->setCallbacks(new MyServerCallbacks());
    pService->start();
    BLEAdvertising *pAdvertising = pServer->getAdvertising(); // 获取广播
    pAdvertising->addServiceUUID(SERVICE_UUID);               // 添加服务UUID
    pAdvertising->setScanResponse(true);                      // 设置扫描响应
    pAdvertising->setMinPreferred(0x06);                      // 设置最小首选值
    pAdvertising->setMinPreferred(0x12);                      // 设置最大首选值
    BLEDevice::startAdvertising();                            // 开始广播
    Serial.println("BLE Server Started");
    //  Serial.println("Waiting a client connection to notify...");
}
void BLEServer_Task(void *pvParam)
{
    while (true)
    {

        while (!connected_state)
        {
            Serial.println("Waiting for connection");
            BLEDevice::startAdvertising(); // 开始广播
            delay(1000);
        }
        state = default_state;
        std::string controlValue = controlCharacteristic.getValue();

        if (controlValue[0] == default_state)
        {
            if (flag)
            {
                Serial.println("[BLE Server is Running]");
                flag = false;
            }
        }
        if (controlValue[0] != default_state)
        {
            state = controlValue[0];
            controlCharacteristic.setValue(&default_state);
            xTaskCreate(BLEMainLogic_Task, "MainLogic", 4096, NULL, 1, &BLEMainLogic_TaskHandle);
            vTaskSuspend(NULL);
            // vTaskSuspend(&BLEServer_TaskHandle);
        }
        // }

        // if (controlValue[0] == default_state)
        // {

        //     Serial.println("[BLEServer]return Success");
        // }
        // else if (controlValue[0] != default_state)
        // {
        //     state = controlValue[0];
        //     Serial.println("up success");
        //     // vTaskResume(&BLEServer_TaskHandle);
        // }
        // else
        // {
        // };
        vTaskDelay(20);
    }
}
void BLEMainLogic_Task(void *pvParam)
{
    while (1)
    {

        if (!flag)
        {
            Serial.println("[BLE]BLEMainLogic is Running");
            flag = true;
        }
        if (state == 'w')
        {
            Serial.println("[Control]Straight Walk");
            xTaskCreate(straight_walk_task, "straight", 4096, NULL, 1, &Control_TaskHandle);
        }
        else if (state == 's')
        {
            Serial.println("[Control]Back Walk");
            xTaskCreate(back_walk_task, "back", 4096, NULL, 1, &Control_TaskHandle);
        }
        else if (state == 'a')
        {
            Serial.println("[Control]Left Walk");
            xTaskCreate(left_cross_walk_task, "left", 4096, NULL, 1, &Control_TaskHandle);
        }
        else if (state == 'd')
        {
            Serial.println("[Control]Right Walk");
            xTaskCreate(right_cross_walk_task, "right", 4096, NULL, 1, &Control_TaskHandle);
        }
        else if (state == 'l')
        {
            Serial.println("[Control]Left Cross Walk");
            xTaskCreate(left_walk_task, "left_cross", 4096, NULL, 1, &Control_TaskHandle);
        }
        else if (state == 'r')
        {
            Serial.println("[Control]Right Cross Walk");
            xTaskCreate(right_walk_task, "right_cross", 4096, NULL, 1, &Control_TaskHandle);
        }
        // else if (state == 'u')
        // {
        //     Serial.println("[Control]Up Slope Walk");
        //     xTaskCreate(up_slope_walk_task, "up_slope", 4096, NULL, 1, &Control_TaskHandle);
        // }
        // else if (state == 'n')
        // {
        //     Serial.println("[Control]Down Slope Walk");
        //     xTaskCreate(down_slope_walk_task, "down_slope", 4096, NULL, 1, &Control_TaskHandle);
        // }
        else if (state == 'c')
        {
            Serial.println("[Control]Car Walk");
            xTaskCreate(car2_Task, "Transform", 4096, NULL, 1, &Control_TaskHandle);
        }
        else if (state == 'v')
        {
            Serial.println("[Control]Pos Test");
            xTaskCreate(Stand_Task, "Stand", 4096, NULL, 1, &Control_TaskHandle);
        }

        else
        {
            Serial.println("default");
            vTaskResume(BLEServer_TaskHandle);
        }
        // if (eTaskGetState(BLEServer_TaskHandle) == eSuspended)
        // {
        //     vTaskResume(BLEServer_TaskHandle);
        //     Serial.println("BLEServer_Task has been resumed.");
        // }
        // else
        // {
        //     Serial.println("BLEServer_Task was not suspended.");
        // }
        //
        // controlCharacteristic.setValue(&state);
        // delay(20);
        // 删除自身
        vTaskDelete(NULL);
    }
}