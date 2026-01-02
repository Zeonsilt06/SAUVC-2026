#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <HMC5883L.h>


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

HMC5883L mag;
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
//MPU6050 accelgyro(0x68, &Wire1); // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Ax, Ay, Az;
float Gx, Gy, Gz;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

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

  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g/m:\t");
  Serial.print(Ax); Serial.print("\t");
  Serial.print(Ay); Serial.print("\t");
  Serial.print(Az); Serial.print("\t");
  Serial.print(Gx); Serial.print("\t");
  Serial.print(Gy); Serial.print("\t");
  Serial.print(Gz); Serial.print("\t");
  Serial.print(mx); Serial.print("\t");
  Serial.print(my); Serial.print("\t");
  Serial.println(mz);

  delay(10);
}











