#ifndef CONFIG_H
#define CONFIG_H

// ============================================
// HARDWARE PIN CONFIGURATION - TEENSY 4.1
// ============================================
#define THRUSTER_M1_PIN 2
#define THRUSTER_M2_PIN 3
#define THRUSTER_M3_PIN 4
#define THRUSTER_M4_PIN 5

#define KILL_SWITCH_PIN 14
#define KILL_SWITCH_LED 13

#define LSM9DS0_XM_ADDR  0x1D
#define LSM9DS0_G_ADDR   0x6B

// ============================================
// TIMING CONFIGURATION
// ============================================
#define MISSION_DURATION 20000
#define MAIN_LOOP_RATE 20
#define IMU_UPDATE_RATE 10
#define PID_UPDATE_RATE 20
#define CALIBRATION_TIME 3000

// ============================================
// THRUSTER PWM CONFIGURATION
// ============================================
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION 16

#define PWM_STOP 1500
#define PWM_MIN 1100
#define PWM_MAX 1900
#define PWM_DEADZONE 10

#define BASE_FORWARD_SPEED 100
#define BASE_VERTICAL_SPEED 0

// ============================================
// PID CONFIGURATION
// ============================================
#define PID_HEADING_KP 4.0
#define PID_HEADING_KI 0.05
#define PID_HEADING_KD 1.5
#define PID_HEADING_SETPOINT 0.0
#define PID_HEADING_MAX_OUTPUT 150.0
#define PID_HEADING_MAX_INTEGRAL 50.0

#define PID_ROLL_KP 3.0
#define PID_ROLL_KI 0.02
#define PID_ROLL_KD 1.0
#define PID_ROLL_SETPOINT 0.0
#define PID_ROLL_MAX_OUTPUT 100.0
#define PID_ROLL_MAX_INTEGRAL 30.0

#define PID_PITCH_KP 3.0
#define PID_PITCH_KI 0.02
#define PID_PITCH_KD 1.0
#define PID_PITCH_SETPOINT 0.0
#define PID_PITCH_MAX_OUTPUT 100.0
#define PID_PITCH_MAX_INTEGRAL 30.0

#define PID_DEPTH_KP 50.0
#define PID_DEPTH_KI 0.1
#define PID_DEPTH_KD 15.0
#define PID_DEPTH_SETPOINT 0.5
#define PID_DEPTH_MAX_OUTPUT 200.0
#define PID_DEPTH_MAX_INTEGRAL 100.0

// ============================================
// SAFETY
// ============================================
#define MAX_TILT_ANGLE 45.0
#define IMU_TIMEOUT 1000

#endif