#ifndef CONFIG_H
#define CONFIG_H

// ================= PWM THRUSTER =================
#define ESC_MIN      1100
#define ESC_NEUTRAL  1500
#define ESC_MAX      1900

// ================= PIN THRUSTER =================
// Depan
#define THRUS_FL 2   // Front Left
#define THRUS_FR 3   // Front Right

// Belakang
#define THRUS_BL 4   // Back Left
#define THRUS_BR 5   // Back Right

// ================= PID YAW =================
#define YAW_KP  2.0
#define YAW_KI  0.0
#define YAW_KD  0.15

// ================= PID DEPTH =================
#define DEPTH_KP  3.0
#define DEPTH_KI  0.0
#define DEPTH_KD  0.2

#endif
