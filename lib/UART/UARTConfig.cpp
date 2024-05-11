#include "UARTConfig.h"

void UARTInit()
{

    Serial.begin(115200);
    while (!Serial)
    {
        ; // 等待串口连接。对于 Leonardo/Micro，等待是必要的
    }
    DebugPrintTest(&Serial);
    Serial.println("[Robot]Master V1.0");

    DebugSerial.println("[UART]Serial Port Init Success");
    // DebugSerial.println("[UART]Servo Serial1 Initing...");
    ServoSerial1.begin(ServoSerial1Baud, SERIAL_8N1, ServoSerial1Rx, ServoSerial1Tx);
    // while (!ServoSerial1)
    // {
    //     ; // 等待串口连接。对于 Leonardo/Micro，等待是必要的
    // }
    DebugSerial.println("[UART]Servo Serial1 Init Success");

    // DebugSerial.println("[UART]Servo Serial2 Initing...");
    ServoSerial2.begin(ServoSerial2Baud, SERIAL_8N1, ServoSerial2Rx, ServoSerial2Tx);
    // while (!ServoSerial2)
    // {
    //     ; // 等待串口连接。对于 Leonardo/Micro，等待是必要的
    // }
    DebugSerial.println("[UART]Servo Serial2 Init Success");
}

void DebugPrintTest(Stream *stream)
{
    stream->println("                                                     ");
    stream->println("                                                     ");
    stream->println("________  ________  ________  ________  _________   ");
    stream->println("|\\   __  \\|\\   __  \\|\\   __  \\|\\   __  \\|\\___   ___\\ ");
    stream->println("\\ \\  \\|\\  \\ \\  \\|\\  \\ \\  \\|\\ /\\ \\  \\|\\  \\|___ \\  \\_| ");
    stream->println(" \\ \\   _  _\\ \\  \\\\\\  \\ \\   __  \\ \\  \\\\\\  \\   \\ \\  \\  ");
    stream->println("  \\ \\  \\\\  \\\\ \\  \\\\\\  \\ \\  \\|\\  \\ \\  \\\\\\  \\   \\ \\  \\ ");
    stream->println("   \\ \\__\\\\ _\\\\ \\_______\\ \\_______\\ \\_______\\   \\ \\__\\");
    stream->println("    \\|__|\\|__|\\|_______|\\|_______|\\|_______|    \\|__|");
    stream->println("                                                     ");
    stream->println("                                                     ");
}

String readStringFromStream(Stream *stream)
{
    String ReceivedData;
    while (stream->available())
    {
        ReceivedData = stream->readString();
    }
    return ReceivedData;
}
