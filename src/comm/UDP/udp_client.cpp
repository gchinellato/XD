/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: UDP Server
 * Owner: gchinellato
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "udp_client.h"
#include "../../main.h"

WiFiUDP UdpClient;

// local port to listen on
const char * localAddr = "192.168.0.101";
const char * udpAddress = "192.168.0.100";
unsigned int udpPort = 5000;
char receivedBuffer[255];

void udpClient(void *pvParameter){
    vTaskDelay(1000);
    UdpClient.begin(udpPort);
    for(;;){
        if(uxQueueMessagesWaiting(gQueueReply)){
            xQueueReceive(gQueueReply, &receivedBuffer, portMAX_DELAY);

            /*Serial.print("Reading UDP:");
            for(int i=0; i < 255; i++){
                Serial.print(receivedBuffer[i]);
                if(receivedBuffer[i] == '#')
                    break;
            }
            Serial.print("\n");*/

            UdpClient.beginPacket(udpAddress, udpPort);
            UdpClient.printf(receivedBuffer);
            UdpClient.endPacket(); 
        }
        vTaskDelay(200);   
    }
}

