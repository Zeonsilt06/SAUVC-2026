#include <Arduino.h>
#include "Thrustercontrol.h"
#include "imu.h"
#include "depth.h"
#include "PID.h"

// Gunakan baud rate tinggi untuk Teensy
#define BAUD_RATE 115200

IMU imu;
DepthController depth;

// PID DEPTH: (Kp, Ki, Kd, MinOut, MaxOut, isAngle)
PID depthPID(2.5, 0.08, 0.6, -200, 200, false);

void setup() {
    Serial.begin(BAUD_RATE);

    // 1. Inisialisasi Thruster
    initMotor();
    stopAll();
    
    // 2. Waktu tunggu ESC Arming (Bakal bunyi Beep-Beep)
    Serial.println("ESC Arming... JANGAN GERAKKAN ROBOT");
    delay(4000); 

    // 3. Inisialisasi Sensor
    if(!imu.begin()) Serial.println("IMU GAGAL!");
    if(!depth.begin(Wire)) Serial.println("DEPTH SENSOR GAGAL!");

    depthPID.setSetpoint(80.0); // Target kedalaman 80 cm
    Serial.println("SYSTEM READY");
}

void loop() {
    // ... pembacaan sensor dan PID ...

    updateMotors(); // Menggerakkan motor selangkah demi selangkah ke target
    delay(20);

    // 1. Baca Sensor
    imu.update();
    depth.update();

    float currentDepth = depth.getDepth();

    // 2. Failsafe: Jika sensor tidak terbaca (NaN)
    if (isnan(currentDepth)) {
        stopAll();
    } 
    else {
        // 3. Hitung PID
        // Jika hasil > 0 berati robot harus naik (UP)
        // Jika hasil < 0 berarti robot harus turun (DOWN)
        int output = (int)depthPID.compute(currentDepth);

        if (output > 0) {
            up(output);
        } else {
            down(abs(output));
        }
    }

    // 4. Update gerakan motor secara halus (Non-Blocking)
    updateMotors();

    // 5. Telemetri (Debug)
    static unsigned long timer = 0;
    if (millis() - timer > 100) {
        Serial.print("Depth: "); Serial.print(currentDepth);
        Serial.print(" | Yaw: "); Serial.print(imu.getYaw());
        Serial.print(" | PID Out: "); Serial.println(depthPID.compute(currentDepth));
        timer = millis();
    }

    delay(20); // Loop 50Hz (Standar industri untuk robotika)
}