/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: Motor
 * Owner: gchinellato
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <driver/dac.h>

#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

//(Vin/Vmax)*255
//(1.2V/3.3V=0.364*255=93)
#define VOLTAGE_MIN 93 //1.2V
#define VOLTAGE_MAX 255 //2.0V

#define DEAD_BAND 0

class Motor
{
public:
	Motor(dac_channel_t channel, gpio_num_t revPin);
	void setSpeedPercentage(float speed);
    void motorOff();
    void currentSense();
	float motorSpeed;
private:
	void motorGo(int direct, float pwm);
    dac_channel_t dac_num;
	gpio_num_t reversePin;
};

#endif