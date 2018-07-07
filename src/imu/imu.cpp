/* 
 * Project: Toy Robot
 * Platfrom: ESP32
 * Description: PID
 * Owner: gchinellato
 */

#include <Arduino.h>
#include <EEPROM.h>
#include "imu.h"

GY80::GY80()
{
    delay(50); // time for sensors to start
}

void GY80::magCalibration()
{

}

float* GY80::getOrientation(int algorithm, float G_dt)
{
    complementaryFilter(G_dt, orientation); // roll, pitch, yaw
	return orientation;
}

void GY80::complementaryFilter(float G_dt, float (&orientationDeg)[3])
{
    accelerometer.getAccVector(accVector);
    gyro.getGyroVector(gyroVector);

    magnetometer.getMagVector(magVector);
    magnetometer.tiltCompensation(accelerometer.roll, accelerometer.pitch, compMagVector);
    magnetometer.setDeclination(-21.0, 7.0);
    magnetometer.getHeading(compMagVector[0], compMagVector[1], true);

    // CF = tau / (tau+LP)
    // tau = CF*LP/(1-CF)
	//i.e: 0.98*0.01sec/(1-0.98) = 0.49tau
    //(if the loop period is shorter than this value, gyro take precedence, otherwise, acceleromter is given more weighting than gyro)
    // orientation in degrees (pitch, roll, yaw from rotation matrix)
    orientationDeg[0] = CF*(orientationDeg[0] + gyro.rateVector[0]*G_dt) + (1-CF)*accelerometer.roll*RAD_TO_DEG;
    orientationDeg[1] = CF*(orientationDeg[1] + gyro.rateVector[1]*G_dt) + (1-CF)*accelerometer.pitch*RAD_TO_DEG;
    orientationDeg[2] = CF*(orientationDeg[2] + gyro.rateVector[2]*G_dt) + (1-CF)*magnetometer.heading*RAD_TO_DEG;
}