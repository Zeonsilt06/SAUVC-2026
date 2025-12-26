#ifndef SENSOR_H
#define SENSOR_H

#include <Wire.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

extern Adafruit_LSM9DS0 lsm;

// Inisialisasi sensor
bool initializeSensor();
void setupSensor();

// Pembacaan sensor
void readSensorData(sensors_event_t &accel, sensors_event_t &mag, 
                    sensors_event_t &gyro, sensors_event_t &temp);

#endif