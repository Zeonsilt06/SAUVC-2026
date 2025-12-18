#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "pid_control.h"
#include "sensor.h"
#include "serial_cmd.h"

PID headPID(HEAD_KP, HEAD_KI, HEAD_KD, 200);

float set_head = 0;

void setup() {
    Serial.begin(9600);
    while (!Serial && millis() < 3000); // tunggu serial (penting di Teensy)

    sensor_init();
    motor_init();

    float yaw, pitch, roll;
    read_imu(yaw, pitch, roll);
    set_head = yaw;

    Serial.println("ROV TEENSY READY");
    Serial.println("Yaw,Pitch,Roll");
}

void loop() {
    float yaw, pitch, roll;
    read_imu(yaw, pitch, roll);

    // =====================
    // SERIAL OUTPUT (LSM9DS0)
    // =====================
    Serial.print(yaw, 2);
    Serial.print(",");
    Serial.print(pitch, 2);
    Serial.print(",");
    Serial.println(roll, 2);

    // =====================
    // PID 
    // =====================
    float out = headPID.compute(set_head, yaw);

    if (abs(out) > 30) {
        yaw_right(abs(out));
    } else {
        forward(120);
    }

    serial_command();
    delay(20);
}
