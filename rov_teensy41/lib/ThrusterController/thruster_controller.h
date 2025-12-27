#ifndef THRUSTER_CONTROLLER_H
#define THRUSTER_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>
#include "../../include/config.h"

// PID Controller class
class PIDController {
public:
    PIDController(float kp, float ki, float kd, float setpoint, 
                  float max_out, float max_int);
    
    float compute(float input, float dt);
    void setSetpoint(float sp);
    void setTunings(float kp, float ki, float kd);
    void reset();
    float getOutput();
    
private:
    float kp, ki, kd;
    float setpoint;
    float integral;
    float lastError;
    float output;
    float maxOutput;
    float maxIntegral;
    
    float normalizeAngle(float angle);
};

// Thruster Controller class
class ThrusterController {
public:
    ThrusterController();
    
    // Initialization
    bool begin();
    void arm();
    
    // Control modes
    void setManualMode(int8_t forward, int8_t strafe, int8_t vertical, int8_t yaw);
    void setStabilizedMode(float rollTarget, float pitchTarget, float yawTarget, float depthTarget);
    
    // PID Updates
    void updatePID(float roll, float pitch, float yaw, float depth, float dt);
    
    // Direct thruster control
    void setThruster(uint8_t id, int16_t pwm);
    void setAllThrusters(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
    void stopAll();
    
    // Base movement commands
    void moveForward(uint8_t speed);
    void moveBackward(uint8_t speed);
    void strafeRight(uint8_t speed);
    void strafeLeft(uint8_t speed);
    void ascend(uint8_t speed);
    void descend(uint8_t speed);
    void rotateClockwise(uint8_t speed);
    void rotateCounterClockwise(uint8_t speed);
    
    // PID enable/disable
    void enablePID(bool heading, bool roll, bool pitch, bool depth);
    
    // Status
    void printStatus();
    bool isArmed();
    
private:
    Servo thruster_M1, thruster_M2, thruster_M3, thruster_M4;
    
    // PID Controllers
    PIDController pidHeading;
    PIDController pidRoll;
    PIDController pidPitch;
    PIDController pidDepth;
    
    // PID enable flags
    bool pidHeadingEnabled;
    bool pidRollEnabled;
    bool pidPitchEnabled;
    bool pidDepthEnabled;
    
    // Base speeds
    int16_t baseForward;
    int16_t baseStrafe;
    int16_t baseVertical;
    int16_t baseYaw;
    
    // Status
    bool armed;
    
    // Helper methods
    int16_t speedToPWM(int8_t speed);
    int16_t constrainPWM(int16_t pwm);
    void applyPIDCorrections(float headingCorr, float rollCorr, 
                            float pitchCorr, float depthCorr);
};

#endif // THRUSTER_CONTROLLER_H