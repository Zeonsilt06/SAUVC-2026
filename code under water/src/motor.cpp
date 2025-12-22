#include "motor.h"
#include "config.h"
#include <Arduino.h>
#include <Servo.h>

// ================= THRUSTER OBJECT =================
Servo thrus_fl;
Servo thrus_fr;
Servo thrus_bl;
Servo thrus_br;

// ================= UTIL =================
static int limitPWM(int pwm) {
    if (pwm > ESC_MAX) return ESC_MAX;
    if (pwm < ESC_MIN) return ESC_MIN;
    return pwm;
}

// ================= INIT =================
void motor_init() {
    thrus_fl.attach(THRUS_FL);
    thrus_fr.attach(THRUS_FR);
    thrus_bl.attach(THRUS_BL);
    thrus_br.attach(THRUS_BR);

    stop_all();
}

// ================= BASIC =================
void stop_all() {
    thrus_fl.writeMicroseconds(ESC_NEUTRAL);
    thrus_fr.writeMicroseconds(ESC_NEUTRAL);
    thrus_bl.writeMicroseconds(ESC_NEUTRAL);
    thrus_br.writeMicroseconds(ESC_NEUTRAL);
}

// ================= MOVEMENT =================
void forward(int speed) {
    thrus_fl.writeMicroseconds(limitPWM(ESC_NEUTRAL + speed));
    thrus_fr.writeMicroseconds(limitPWM(ESC_NEUTRAL + speed));
    thrus_bl.writeMicroseconds(limitPWM(ESC_NEUTRAL + speed));
    thrus_br.writeMicroseconds(limitPWM(ESC_NEUTRAL + speed));
}

void yaw_right(int speed) {
    thrus_fl.writeMicroseconds(limitPWM(ESC_NEUTRAL + speed));
    thrus_bl.writeMicroseconds(limitPWM(ESC_NEUTRAL + speed));

    thrus_fr.writeMicroseconds(limitPWM(ESC_NEUTRAL - speed));
    thrus_br.writeMicroseconds(limitPWM(ESC_NEUTRAL - speed));
}

void yaw_left(int speed) {
    thrus_fl.writeMicroseconds(limitPWM(ESC_NEUTRAL - speed));
    thrus_bl.writeMicroseconds(limitPWM(ESC_NEUTRAL - speed));

    thrus_fr.writeMicroseconds(limitPWM(ESC_NEUTRAL + speed));
    thrus_br.writeMicroseconds(limitPWM(ESC_NEUTRAL + speed));
}

// ================= DEPTH (DUMMY) =================
void depth_up(int speed) {
    // nanti diisi kalau ada thruster vertikal
}

void depth_down(int speed) {
    // nanti diisi
}
