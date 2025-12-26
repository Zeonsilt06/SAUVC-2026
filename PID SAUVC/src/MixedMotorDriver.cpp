// src/MixedMotorDriver.cpp

#include "MixedMotorDriver.h"
#include <Arduino.h>

MixedMotorDriver::MixedMotorDriver(int pinH1, int pinH2, int pinV1, int pinV2) {
    escPins[0] = pinH1;  // Horizontal motor 1 (kiri)
    escPins[1] = pinH2;  // Horizontal motor 2 (kanan)
    escPins[2] = pinV1;  // Vertical motor 1
    escPins[3] = pinV2;  // Vertical motor 2
}

bool MixedMotorDriver::begin() {
    // Attach semua ESC ke pin masing-masing
    for (int i = 0; i < 4; i++) {
        esc[i].attach(escPins[i]);
        if (!esc[i].attached()) {
            Serial.print("Gagal attach ESC pada pin ");
            Serial.println(escPins[i]);
            return false;
        }
    }

    // Arming sequence: set semua ke posisi neutral/stop
    // Horizontal: neutral = 1500 μs (stop untuk bidirectional ESC)
    // Vertical: stop = 1000 μs (unidirectional)
    esc[0].writeMicroseconds(1500);
    esc[1].writeMicroseconds(1500);
    esc[2].writeMicroseconds(1000);
    esc[3].writeMicroseconds(1000);

    delay(4000);  // Tunggu 4 detik untuk arming (sesuaikan dengan ESC kamu, biasanya 2-5 detik)

    Serial.println("Semua 4 ESC berhasil di-arming.");
    return true;
}

void MixedMotorDriver::writeMicroseconds(int idx, int us) {
    us = constrain(us, 1000, 2000);  // Batasi agar aman
    esc[idx].writeMicroseconds(us);
}

void MixedMotorDriver::setHorizontalSpeed(int motor, int speed) {
    if (motor < 0 || motor > 1) return;  // Hanya motor 0 atau 1

    speed = constrain(speed, -100, 100);

    int us;
    if (speed == 0) {
        us = 1500;  // Neutral / stop
    } else {
        us = map(speed, -100, 100, 1000, 2000);  // -100 = full mundur, 100 = full maju
    }

    writeMicroseconds(motor, us);  // motor 0 atau 1
}

void MixedMotorDriver::setVerticalThrottle(int motor, int throttle) {
    if (motor < 0 || motor > 1) return;  // Hanya motor 0 atau 1

    throttle = constrain(throttle, 0, 100);  // Unidirectional: 0 = stop, 100 = full up

    int us = map(throttle, 0, 100, 1000, 2000);  // 1000 = stop, 2000 = full thrust up

    writeMicroseconds(motor + 2, us);  // Vertical motor index 2 dan 3
}

void MixedMotorDriver::stopAll() {
    setHorizontalSpeed(0, 0);
    setHorizontalSpeed(1, 0);
    setVerticalThrottle(0, 0);
    setVerticalThrottle(1, 0);

    Serial.println("Semua motor di-stop (emergency safe).");
}