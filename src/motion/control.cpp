/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: System Control to keep balance
 * Owner: gchinellato
 */

#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include <driver/dac.h>
#include "control.h"
#include "pinmux/pinmux.h"
#include "imu/imu.h"
#include "imu/MPU9250.h"
#include "pid/PID.h"
#include "motion/motor/motor.h"
#include "motion/encoder/encoder.h"
#include "../main.h"

//PID objects
PID speedPID;
PID anglePID;

//Motor objects
Motor motor1(DAC_CHANNEL_1, MOTOR_A_REV);
Motor motor2(DAC_CHANNEL_2, MOTOR_B_REV);

void setConfiguration(Configuration *configuration)
{
    configuration->speedPIDKp = SPEED_KP;
    configuration->speedPIDKi = SPEED_KI;
    configuration->speedPIDKd = SPEED_KD;
    configuration->speedPIDOutputLowerLimit = -10.00;
    configuration->speedPIDOutputHigherLimit = 10.00;
    configuration->anglePIDAggKp = ANGLE_KP_AGGR;
    configuration->anglePIDAggKi = ANGLE_KI_AGGR;
    configuration->anglePIDAggKd = ANGLE_KD_AGGR;
    configuration->anglePIDConKp = ANGLE_KP_CONS;
    configuration->anglePIDConKi = ANGLE_KI_CONS;
    configuration->anglePIDConKd = ANGLE_KD_CONS;
    configuration->anglePIDLowerLimit = ANGLE_LIMIT;
    configuration->calibratedZeroAngle = CALIBRATED_ZERO_ANGLE;
}

void control(void *pvParameter)
{ 
  //I2C Init
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  
  vTaskDelay(50);
  float dt=0;
  int command = 0;
  int size; 
  char *ret;
  unsigned long timestamp=0;
  unsigned long timestamp_old=0;
  float *ori;
  float speedPIDInput, anglePIDInput;
  float speedPIDOutput, anglePIDOutput;
  boolean started = true;
  char receivedBuffer[255];
  char sendBuffer[255];
  MPU9250 myIMU;
  myIMU.initMPU9250();

  UserControl userControl = {0, 0};
  Configuration configuration;  
 
  setConfiguration(&configuration);

  vTaskDelay(50);

  for(;;)
  {
    timestamp = millis();
    if ((timestamp - timestamp_old) >= DATA_INTERVAL)
    {
      //convert from ms to sec
      dt = (float)(timestamp - timestamp_old)/1000.0f; 
      timestamp_old = timestamp;

      //getEvent
      if(uxQueueMessagesWaiting(gQueueEvent)){
        xQueueReceive(gQueueEvent, &receivedBuffer, portMAX_DELAY);
        
        //split string into tokens
        ret = strtok(receivedBuffer, ",");

        //get command
        command = atoi(ret);

        //get number of parameters
        //size = int(strtok(NULL, ","));

        switch (command) {
            case STARTED:
                started = atoi(strtok(NULL, ","));
                Serial.println("STARTED command: " + String(started));
                break;
            case DIRECTION:
                userControl.direction = atof(strtok(NULL, ","));
                userControl.direction /= 10;
                Serial.println("DIRECTION command: " + String(userControl.direction));
                break;
            case STEERING:
                userControl.steering = atof(strtok(NULL, ","));
                userControl.steering /= 10;
                Serial.println("STEERING command: " + String(userControl.steering));
                break;
            case ANGLE_PID_CONS:
                configuration.anglePIDConKp = atof(strtok(NULL, ","));
                configuration.anglePIDConKi = atof(strtok(NULL, ","));
                configuration.anglePIDConKd = atof(strtok(NULL, ","));
                configuration.calibratedZeroAngle = atof(strtok(NULL, ","));
                configuration.anglePIDLowerLimit = atof(strtok(NULL, ","));
                Serial.println("ANGLE_PID_CONS command: " + String(configuration.anglePIDConKp) +","+ String(configuration.anglePIDConKi) +","+ String(configuration.anglePIDConKd));
                Serial.println("ZERO_ANGLE command: " + String(configuration.calibratedZeroAngle));
                Serial.println("ANGLE_LIMITE command: " + String(configuration.anglePIDLowerLimit));
                break;
            case ZERO_ANGLE:
                configuration.calibratedZeroAngle = atof(strtok(NULL, ","));
                Serial.println("ZERO_ANGLE command: " + String(configuration.calibratedZeroAngle));
                break;
            case ANGLE_LIMITE:
                configuration.anglePIDLowerLimit = atof(strtok(NULL, ","));
                Serial.println("ANGLE_LIMITE command: " + String(configuration.anglePIDLowerLimit));
                break;
            default:
                Serial.println("Unknown command");
                break;
        }
      }

      ori = myIMU.getOrientation(1, dt);
      //Serial.println("dt: " + String(dt) + ", Roll: " + String(ori[0]) + ", Pitch: " + String(ori[1]) + ", Yaw: " + String(ori[2]));
      //anglePIDInput = -ori[0];
      anglePIDInput = ori[1];

      //analog input (0V=0 / 3V3=4095)
      int analog_input = analogRead(ANALOG_INPUT);
      //Serial.println("Analog Input: " + String(analog_input));
      userControl.direction = (float) (analog_input/40.95) - (675/40.95); 

      //Set angle setpoint and compensate to reach equilibrium point
      anglePID.setSetpoint(configuration.calibratedZeroAngle);
      anglePID.setTunings(configuration.anglePIDConKp, configuration.anglePIDConKi, configuration.anglePIDConKd);
      //Compute Angle PID (input is current angle)
      anglePIDOutput = anglePID.compute(anglePIDInput);
      //Serial.println("anglePIDoutput: " + String(anglePIDOutput));

      //Auto or Manual Mode
      if (started &&
          (abs(anglePIDInput) > (abs(CALIBRATED_ZERO_ANGLE)-configuration.anglePIDLowerLimit) && 
          abs(anglePIDInput) < (abs(CALIBRATED_ZERO_ANGLE)+configuration.anglePIDLowerLimit))) {
          //compensate dead band
          if(anglePIDOutput >= 0) {
              anglePIDOutput += DEAD_BAND;
          }else {
              anglePIDOutput -= DEAD_BAND;
          }

          //max speed - self balance mode
          int max_speed = 20;
          if(anglePIDOutput > max_speed) {
              anglePIDOutput = max_speed;
          }else if (anglePIDOutput < -max_speed) {
              anglePIDOutput = -max_speed;
          }

          //Serial.println("Auto Mode: " + String(anglePIDOutput));
          motor1.setSpeedPercentage(anglePIDOutput);
          motor2.setSpeedPercentage(anglePIDOutput);
      }
      else {
          if(digitalRead(GPIO_NUM_27)){
              userControl.direction = -userControl.direction;
          }

          Serial.println("Manual Mode: " + String(userControl.direction));
          motor1.setSpeedPercentage(userControl.direction);
          motor2.setSpeedPercentage(userControl.direction);
      }

      //notify
      sprintf(sendBuffer, "%0.2f,", anglePIDInput);
      sprintf(sendBuffer, "%s%0.2f#", sendBuffer, anglePIDOutput);
      xQueueSend(gQueueReply, &sendBuffer, 10);
    }
    vTaskDelay(DATA_INTERVAL / portTICK_RATE_MS); 
  }
  vTaskDelete(NULL);
}