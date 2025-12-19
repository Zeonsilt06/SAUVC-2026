#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "pid_control.h"
#include "sensor.h"
#include "serial_cmd.h"

PID headPID(HEAD_KP, HEAD_KI, HEAD_KD, 200);

float set_head = 0;
unsigned long lastPrint = 0;
const unsigned long SERIAL_INTERVAL = 1000; // 1 detik

void setup() {
    Serial.begin(9600);
    while (!Serial && millis() < 3000); // penting di Teensy

    sensor_init();
    motor_init();

    Serial.println("ROV TEENSY READY");
    Serial.println("LSM9DS0 DATA OUTPUT");
}

void loop() {

    // =========================
    // SERIAL OUTPUT LSM9DS0
    // =========================
    if (millis() - lastPrint >= SERIAL_INTERVAL) {
        lastPrint = millis();

        float ax, ay, az;
        float mx, my, mz;
        float gx, gy, gz;
        float temp;

        read_lsm9ds0(ax, ay, az, mx, my, mz, gx, gy, gz, temp);

        Serial.print("Acceleration (m/s^2): (");
        Serial.print(ax, 3); Serial.print(", ");
        Serial.print(ay, 3); Serial.print(", ");
        Serial.print(az, 3); Serial.println(")");

        Serial.print("Magnetometer (gauss): (");
        Serial.print(mx, 3); Serial.print(", ");
        Serial.print(my, 3); Serial.print(", ");
        Serial.print(mz, 3); Serial.println(")");

        Serial.print("Gyroscope (deg/s): (");
        Serial.print(gx, 3); Serial.print(", ");
        Serial.print(gy, 3); Serial.print(", ");
        Serial.print(gz, 3); Serial.println(")");

        Serial.print("Temperature: ");
        Serial.print(temp, 3);
        Serial.println(" C");
        Serial.println("--------------------------------");
    }

    // =========================
    // PID & MOTOR (TIDAK DIUBAH)
    // =========================
    float yaw = 0; // sementara dummy (tidak dipakai dosen)
    float out = headPID.compute(set_head, yaw);

    if (abs(out) > 30) {
        yaw_right(abs(out));
    } else {
        forward(120);
    }

    serial_command();
}
