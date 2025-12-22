#include <Arduino.h>
#include "sensor_handler.h"  // <--- INI HARUS ADA
#include "AngleKalman.h"

AngleKalman kRoll, kPitch, kYaw;
uint32_t lastMicros;

void setup() {
    Serial.begin(115200);
    if (!initSensor()) {
        while(1) { Serial.println("Sensor Missing!"); delay(1000); }
    }
    lastMicros = micros();
}

void loop() {
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    getRawData(ax, ay, az, gx, gy, gz, mx, my, mz);

    uint32_t now = micros();
    float dt = (float)(now - lastMicros) / 1000000.0f;
    lastMicros = now;

    // Hitung sudut dasar
    float rollAcc  = atan2(ay, az) * 57.29578f;
    float pitchAcc = atan2(-ax, sqrt(ay*ay + az*az)) * 57.29578f;
    float yawMag   = atan2(my, mx) * 57.29578f;

    // Filter dengan Kalman
    float rollF  = kRoll.update(gx, rollAcc, dt);
    float pitchF = kPitch.update(gy, pitchAcc, dt);
    float yawF   = kYaw.update(gz, yawMag, dt);

    // Kirim ke Serial Plotter dengan pemisahan gelombang
    Serial.print("Pitch:"); Serial.print(pitchF); Serial.print(",");
    Serial.print("Roll:");  Serial.print(rollF);  Serial.print(",");
    Serial.print("Yaw:");   Serial.println(yawF);

    delay(5);
}