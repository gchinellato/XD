/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: GPIO mapping
 * Owner: gchinellato
 */

#ifndef PINMUX_H
#define PINMUX_H

#define I2C_SDA     GPIO_NUM_18
#define I2C_SCL     GPIO_NUM_19
#define I2C_FREQ    100000

#define MOTOR_A_REV 	GPIO_NUM_33
#define MOTOR_B_REV 	GPIO_NUM_27

#define ANALOG_INPUT	GPIO_NUM_34

#endif