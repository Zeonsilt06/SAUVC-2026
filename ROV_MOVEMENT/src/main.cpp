#include <Arduino.h>
#include "ThrusterControl.h"
#include "imu.h"
#include "depth.h"
#include "PID.h"

#define BAUD_RATE 9600

IMU imu;
DepthController depth;

// PID DEPTH
PID depthPID(
    2.0,    // Kp
    0.05,   // Ki
    0.8,    // Kd
    -150,   // min output
    150,    // max output
    false   // bukan angle
);

void setup()
{
    Serial.begin(BAUD_RATE);

    initMotor();
    stopAll();
    delay(3000); // ARM ESC (WAJIB)

    imu.begin();
    depth.begin(Wire);

    depthPID.setSetpoint(85.0); // target depth cm

    Serial.println("SYSTEM READY");
}

void loop()
{
    imu.update();
    depth.update();

    float currentDepth = depth.getDepth();

    // FAILSAFE SENSOR
    if (isnan(currentDepth))
    {
        stopAll();
        return;
    }

    int depthOffset = (int)depthPID.compute(currentDepth);

    up(depthOffset);

    Serial.print("Depth: ");
    Serial.print(currentDepth);
    Serial.print(" cm | PID: ");
    Serial.println(depthOffset);

    delay(50);
}