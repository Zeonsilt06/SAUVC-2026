#include <Arduino.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <I2Cdev.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <MadgwickAHRS.h>


HMC5883L mag;
//MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
MPU6050 accelgyro(0x68, &Wire1); // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object
Madgwick IMU;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Ax, Ay, Az;
float Gx, Gy, Gz;
float Mx, My, Mz;
float mw_pitch, mw_roll, mw_yaw;

void setup() {

    Wire1.begin();

    delay(500);

    // initialize serial communication
    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    mag.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

    delay(500);
    IMU.begin(35);
}

void loop() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // read raw heading measurements from device
  mag.getHeading(&mx, &my, &mz);

  Ax = (float) ax / 16384.0;
  Ay = (float) ay / 16384.0;
  Az = (float) az / 16384.0;
  Gx = (float) gx / 131.0;
  Gy = (float) gy / 131.0;
  Gz = (float) gz / 131.0;
  Mx = (float) mx / 1090.0;
  My = (float) my / 1090.0;
  Mz = (float) mz / 1090.0;

  IMU.update(Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz);
  mw_pitch = IMU.getPitch();
  mw_roll = IMU.getRoll();
  mw_yaw = IMU.getYaw();


  // display tab-separated accel/gyro x/y/z values
//   Serial.print("a/g/m:\t");
//   Serial.print(Ax); Serial.print("\t");
//   Serial.print(Ay); Serial.print("\t");
//   Serial.print(Az); Serial.print("\t");
//   Serial.print(Gx); Serial.print("\t");
//   Serial.print(Gy); Serial.print("\t");
//   Serial.print(Gz); Serial.print("\t");
//   Serial.print(Mx); Serial.print("\t");
//   Serial.print(My); Serial.print("\t");
//   Serial.println(Mz);
    Serial.print(mw_pitch); Serial.print("\t");
    Serial.print(mw_roll); Serial.print("\t");
    Serial.println(mw_yaw);

  delay(10);
}











