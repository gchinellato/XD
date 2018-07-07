/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: Entry point
 * Owner: gchinellato
 */

extern "C" {
   void app_main();
}

#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "pinmux/pinmux.h"
#include "motion/control.h"
#include "motion/pid/PID.h"
#include "motion/encoder/encoder.h"
#include "comm/UDP/udp_server.h"

//inter-task communication queue
char buffer[255];
QueueHandle_t gQueue = xQueueCreate(1, sizeof(buffer));

/**
  * @brief Main App Entry point
  */
void setup()
{
  //Serial Init
  Serial.begin(115200);
  Serial.setTimeout(10);
  while(!Serial) {}

  //motor reverse
  pinMode (GPIO_NUM_27, OUTPUT);
  pinMode (GPIO_NUM_33, OUTPUT);

  if(gQueue == NULL){
    Serial.println("Error creating the queue");
  }

  delay(1000);

  xTaskCreate(&control, "control", configMINIMAL_STACK_SIZE+8192, NULL, 255, NULL);
  xTaskCreate(&udpServer, "UDP_Server", configMINIMAL_STACK_SIZE+8192, NULL, 10, NULL);
}

void loop()
{
  //interface
  vTaskDelay(1000);
}