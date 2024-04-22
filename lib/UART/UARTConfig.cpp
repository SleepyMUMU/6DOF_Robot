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
