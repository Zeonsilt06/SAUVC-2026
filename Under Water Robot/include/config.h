#ifndef CONFIG_H
#define CONFIG_H

// ===== ESC PWM =====
#define ESC_NEUTRAL 1500
#define ESC_MIN     1100
#define ESC_MAX     1900

// ===== THRUSTER PIN (Teensy 4.1) =====
#define THRUS_KA 7
#define THRUS_KI 8
#define THRUS_PI 9
#define THRUS_1  10
#define THRUS_2  11
#define THRUS_3  16
#define THRUS_4  12

// ===== PID DEFAULT =====
#define DEPTH_KP 20.0
#define DEPTH_KI 0.0
#define DEPTH_KD 0.0

#define HEAD_KP  10.0
#define HEAD_KI  0.0
#define HEAD_KD  2.0

#endif
