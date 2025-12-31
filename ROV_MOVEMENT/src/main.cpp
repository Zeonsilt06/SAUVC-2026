#include <Arduino.h>
#include "ThrusterControl.h"
#include "imu.h"
#include "depth.h"
#include "PID.h"

#define BAUD_RATE 9600

IMU imu;

void setup()
{   
    Serial.begin(BAUD_RATE);

    initMotor();
    stopAll();

    if (imu.begin())
    {
        Serial.println("IMU OK");
    }
    else
    {
        Serial.println("IMU FAIL");
    }
}

void loop()
{

    // while(simulation)
    // {
    //     //put ur code here
    // }
    
    imu.update();

    float yaw = imu.getYaw();
    float pitch = imu.getPitch();
    float roll = imu.getRoll();

    Serial.print("Yaw: ");
    Serial.print(yaw);
    Serial.print(" | Pitch: ");
    Serial.print(pitch);
    Serial.print(" | Roll: ");
    Serial.println(roll);

    // Contoh gerakan
    forward(0);
    delay(3000);

    rotateRight(0);
    delay(3000);

    up(0);
    delay(3000);

    rotateLeftUp(0, 0);
    delay(3000);

    stopAll();
    delay(4000);
}