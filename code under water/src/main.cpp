#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "pid_control.h"
#include "sensor.h"

PID yawPID(YAW_KP, YAW_KI, YAW_KD, 200);

float yawSetpoint = 90; // heading tengah
unsigned long lastPrint = 0;

void setup() {
    Serial.begin(9600);
    while (!Serial && millis() < 3000);

    sensor_init();
    motor_init();
}

void loop() {

    // ================= SERIAL PLOT =================
    if (millis() - lastPrint >= 50) {
        lastPrint = millis();
        print_imu_serial();
    }

    // ================= PID YAW =================
    float yaw, pitch, roll;
    read_imu(yaw, pitch, roll);

    float yawOut = yawPID.compute(yawSetpoint, yaw);

    if (yawOut > 10) {
        yaw_right(abs(yawOut));
    }
    else if (yawOut < -10) {
        yaw_left(abs(yawOut));
    }
    else {
        forward(120);
    }
}
