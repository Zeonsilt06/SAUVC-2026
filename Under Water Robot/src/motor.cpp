#include "motor.h"
#include "config.h"

Servo thrus_ka, thrus_ki, thrus_pi;
Servo thrus_1, thrus_2, thrus_3, thrus_4;

static int limitPWM(int pwm) {
    if (pwm > ESC_MAX) return ESC_MAX;
    if (pwm < ESC_MIN) return ESC_MIN;
    return pwm;
}

void motor_init() {
    thrus_ka.attach(THRUS_KA);
    thrus_ki.attach(THRUS_KI);
    thrus_pi.attach(THRUS_PI);
    thrus_1.attach(THRUS_1);
    thrus_2.attach(THRUS_2);
    thrus_3.attach(THRUS_3);
    thrus_4.attach(THRUS_4);
    stop_all();
}

void stop_all() {
    thrus_ka.writeMicroseconds(ESC_NEUTRAL);
    thrus_ki.writeMicroseconds(ESC_NEUTRAL);
    thrus_pi.writeMicroseconds(ESC_NEUTRAL);
    thrus_1.writeMicroseconds(ESC_NEUTRAL);
    thrus_2.writeMicroseconds(ESC_NEUTRAL);
    thrus_3.writeMicroseconds(ESC_NEUTRAL);
    thrus_4.writeMicroseconds(ESC_NEUTRAL);
}

void forward(int speed) {
    thrus_1.writeMicroseconds(limitPWM(ESC_NEUTRAL + speed));
    thrus_2.writeMicroseconds(limitPWM(ESC_NEUTRAL - speed));
    thrus_3.writeMicroseconds(limitPWM(ESC_NEUTRAL + speed));
    thrus_4.writeMicroseconds(limitPWM(ESC_NEUTRAL + speed));
}

void yaw_right(int speed) {
    thrus_1.writeMicroseconds(limitPWM(ESC_NEUTRAL + speed));
    thrus_2.writeMicroseconds(limitPWM(ESC_NEUTRAL + speed));
    thrus_3.writeMicroseconds(limitPWM(ESC_NEUTRAL + speed));
    thrus_4.writeMicroseconds(limitPWM(ESC_NEUTRAL - speed));
}

void set_vertical(int ka, int ki) {
    thrus_ka.writeMicroseconds(limitPWM(ESC_NEUTRAL + ka));
    thrus_ki.writeMicroseconds(limitPWM(ESC_NEUTRAL + ki));
}
