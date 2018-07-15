/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: Motor
 * Owner: gchinellato
 */

#include <Arduino.h>
#include "motor.h"
#include "pinmux/pinmux.h"

Motor::Motor(dac_channel_t channel, gpio_num_t revPin) 
{
    dac_num = channel; // DAC Channel(0-1)
    reversePin = revPin;
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 pwm: should be a value between ? and 255, higher the number, the faster it'll go
 */
void Motor::motorGo(int direct, float voltage)
{
    // Set inA
    if (direct <= CW)
    {
        //93
        voltage = abs(VOLTAGE_MIN+voltage);
        digitalWrite(reversePin, HIGH);
    }
    else
    {
        //101
        voltage = abs(VOLTAGE_MIN+8+voltage);
        digitalWrite(reversePin, LOW);
    }

    if(voltage > 255)
    {
        voltage = VOLTAGE_MAX;
    }

    dac_out_voltage(dac_num, abs(voltage));
}

/* set speed in percentage from -100 to 100 */
void Motor::setSpeedPercentage(float speed)
{
    // anything above 100 or below -100 is invalid
    if (speed > 100)
        speed = 100;
    else if (speed < -100)
        speed = -100;

    // negative speed
    if (speed > 0) {
        motorGo(CW, (255/100 * speed));
    }
    else if (speed < 0){
        motorGo(CCW, (-255/100 * speed));
    }
    else {
        motorOff();
    }
}

void Motor::motorOff()
{
    dac_out_voltage(dac_num, abs(0));
}

void Motor::currentSense()
{

}