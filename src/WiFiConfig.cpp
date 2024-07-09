#include "WiFiConfig.h"

WiFiConfig::WiFiConfig()
{
    this->ssid = defaultSSID;
    this->passward = defaultPassward;
}

bool WiFiConfig::WiFiInit()
{
    this->ssid = defaultSSID;
    this->passward = defaultPassward;
    // WiFi.config(defaultLocalIP, defaultGateway, defaultSubnetMask, defaultPrimaryDNS, defaultSecondaryDNS); // 设置静态IP
    WiFi.begin(ssid, passward);                                                                             // 连接WiFi
    while (WiFi.status() != WL_CONNECTED)                                                                   // 等待WiFi连接
    {
        DebugSerial.println("[WiFi]Try to connected");
        delay(defaultReconnectTime);
        if (WiFi.status() == WL_CONNECTED) // 判断是否连接成功
        {
            DebugSerial.println("[WiFi]Successful connected ");
            DebugSerial.print("[WiFi]IP Address: ");
            DebugSerial.println(WiFi.localIP());
            return true;
        }
        else
        {
            DebugSerial.println("[WiFi]Fail to connected");
            WiFi.setAutoReconnect(true); // 设置自动重连
            return false;
        }
    }
    return true;
}

bool WiFiConfig::WiFiInit(String ssid, String passward, IPAddress localIP, IPAddress gateway, IPAddress subnetMask, IPAddress primaryDNS, IPAddress secondaryDNS)
{
    this->ssid = ssid;
    this->passward = passward;
    this->localIP = localIP;
    this->gateway = gateway;
    this->subnetMask = subnetMask;
    this->primaryDNS = primaryDNS;
    this->secondaryDNS = secondaryDNS;
    WiFi.config(localIP, gateway, subnetMask, primaryDNS, secondaryDNS); // 设置静态IP
    WiFi.begin(ssid, passward);                                          // 连接WiFi
    while (WiFi.status() != WL_CONNECTED)                                // 等待WiFi连接
    {
        DebugSerial.println("[WiFi]Try to connected");
        delay(defaultReconnectTime);
        if (WiFi.status() == WL_CONNECTED) // 判断是否连接成功
        {
            DebugSerial.println("[WiFi]Successful connected ");
            DebugSerial.print("IP Address: ");
            DebugSerial.println(WiFi.localIP());
            return true;
        }
        else
        {
            DebugSerial.println("[WiFi]Fail to connected");
            WiFi.setAutoReconnect(true); // 设置自动重连
            return false;
        }
    }
    return true;
}

bool WiFiConfig::WiFiInit(String ssid, String passward)
{
    this->ssid = ssid;
    this->passward = passward;
    WiFi.config(defaultLocalIP, defaultGateway, defaultSubnetMask, defaultPrimaryDNS, defaultSecondaryDNS); // 设置静态IP
    WiFi.begin(ssid, passward);                                                                             // 连接WiFi
    while (WiFi.status() != WL_CONNECTED)                                                                   // 等待WiFi连接
    {
        DebugSerial.println("[WiFi]Try to connected");
        delay(defaultReconnectTime);
        if (WiFi.status() == WL_CONNECTED) // 判断是否连接成功
        {
            DebugSerial.println("[WiFi]Successful connected ");
            DebugSerial.print("IP Address: ");
            DebugSerial.println(WiFi.localIP());

            return true;
        }
        else
        {
            DebugSerial.println("[WiFi]Fail to connected");
            WiFi.setAutoReconnect(true); // 设置自动重连
            return false;
        }
    }
    return true;
}

void WiFiConfig::OTAconfig()
{
    this->OTAHostname = defaultOTAHostname;
    this->OTAPassword = defaultOTAPassword;
    ArduinoOTA.setHostname(defaultOTAHostname);
    ArduinoOTA.setPassword(defaultOTAPassword);
    ArduinoOTA.setPort(defaultOTAPort);
    ArduinoOTA
        .onStart([]()
                 {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type); })
        .onEnd([]()
               { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

    ArduinoOTA.begin();
    DebugSerial.println("[OTA]Ready");
    DebugSerial.print("[OTA]IP Address");
    DebugSerial.println(WiFi.localIP());
    DebugSerial.printf("[OTA]esp_get_free_heap_size : %d  \n", esp_get_free_heap_size());
    DebugSerial.printf("[OTA]Password : %s  \n", OTAPassword);
}

void WiFiConfig::OTAconfig(const char *OTAHostname, const char *OTAPassword)
{
    this->OTAHostname = OTAHostname;
    this->OTAPassword = OTAPassword;
    ArduinoOTA.setHostname(OTAHostname);
    ArduinoOTA.setPassword(OTAPassword);

    ArduinoOTA.setPort(defaultOTAPort);
    ArduinoOTA
        .onStart([]()
                 {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type); })
        .onEnd([]()
               { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

    ArduinoOTA.begin();

    DebugSerial.println("[OTA]Ready");
    DebugSerial.print("[OTA]IP Address");
    DebugSerial.println(WiFi.localIP());
    DebugSerial.printf("[OTA]esp_get_free_heap_size : %d  \n", esp_get_free_heap_size());
    DebugSerial.printf("[OTA]Password : %s  \n", OTAPassword);
}

void OTATask(void *pvParam)
{
    WiFiConfig *Target = (WiFiConfig *)pvParam;
    while (true)
    {
        if (Target->WiFi.status() == WL_CONNECTED)
        {
            ArduinoOTA.handle(); // Handle OTA update
        }
        else
        {
            DebugSerial.println("[OTA]WiFi disconnected, OTA stop, try to reconnect.");
            while (!Target->WiFiInit())
                ;
        }
        delay(1);
    }
}
void WiFiStatusTask(void *pvParam)
{
    TCPConfig *Target = (TCPConfig *)pvParam;
    if (WiFi.status() == WL_CONNECTED)
    {

        Target->TCP.println("[I][WiFi]IP: " + WiFi.localIP().toString());
    }
    else
    {
        Target->TCP.printf("[E][WiFi]WiFi disconnected\n");
    }
    vTaskDelete(NULL);
}
// void WiFiReconnect_Task(void *pvParam)
// {
//     WiFiConfig *Target = (WiFiConfig *)pvParam;
//     while (true)
//     {
//         if (Target->WiFi.status() != WL_CONNECTED)
//         {
//             DebugSerial.println("[WiFi]WiFi disconnected, try to reconnect");
//             while (!Target->WiFiInit())
//                 ;
//         }
//         delay(1000);
//     }
// }