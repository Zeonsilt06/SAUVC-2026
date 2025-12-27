#include "thruster_controller.h"

// ============================================
// PID CONTROLLER IMPLEMENTATION
// ============================================

PIDController::PIDController(float kp, float ki, float kd, float setpoint,
                             float max_out, float max_int) :
    kp(kp), ki(ki), kd(kd), setpoint(setpoint),
    maxOutput(max_out), maxIntegral(max_int),
    integral(0), lastError(0), output(0)
{
}

float PIDController::compute(float input, float dt) {
    // Calculate error
    float error = setpoint - input;
    
    // For angular values, normalize error
    if (abs(setpoint) <= 180) {  // Assume angular if in [-180, 180]
        error = normalizeAngle(error);
    }
    
    // Proportional
    float P = kp * error;
    
    // Integral with anti-windup
    integral += error * dt;
    integral = constrain(integral, -maxIntegral, maxIntegral);
    float I = ki * integral;
    
    // Derivative
    float derivative = (error - lastError) / dt;
    float D = kd * derivative;
    
    // Calculate output
    output = P + I + D;
    output = constrain(output, -maxOutput, maxOutput);
    
    // Update last error
    lastError = error;
    
    return output;
}

void PIDController::setSetpoint(float sp) {
    setpoint = sp;
}

void PIDController::setTunings(float _kp, float _ki, float _kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

void PIDController::reset() {
    integral = 0;
    lastError = 0;
    output = 0;
}

float PIDController::getOutput() {
    return output;
}

float PIDController::normalizeAngle(float angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

// ============================================
// THRUSTER CONTROLLER IMPLEMENTATION
// ============================================

ThrusterController::ThrusterController() :
    pidHeading(PID_HEADING_KP, PID_HEADING_KI, PID_HEADING_KD, 
               PID_HEADING_SETPOINT, PID_HEADING_MAX_OUTPUT, PID_HEADING_MAX_INTEGRAL),
    pidRoll(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD,
            PID_ROLL_SETPOINT, PID_ROLL_MAX_OUTPUT, PID_ROLL_MAX_INTEGRAL),
    pidPitch(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD,
             PID_PITCH_SETPOINT, PID_PITCH_MAX_OUTPUT, PID_PITCH_MAX_INTEGRAL),
    pidDepth(PID_DEPTH_KP, PID_DEPTH_KI, PID_DEPTH_KD,
             PID_DEPTH_SETPOINT, PID_DEPTH_MAX_OUTPUT, PID_DEPTH_MAX_INTEGRAL),
    pidHeadingEnabled(true), pidRollEnabled(false), 
    pidPitchEnabled(false), pidDepthEnabled(false),
    baseForward(0), baseStrafe(0), baseVertical(0), baseYaw(0),
    armed(false)
{
}

bool ThrusterController::begin() {
    Serial.println("[THRUSTER] Initializing...");
    
    // Attach PWM servos
    thruster_M1.attach(THRUSTER_M1_PIN, PWM_MIN, PWM_MAX);
    thruster_M2.attach(THRUSTER_M2_PIN, PWM_MIN, PWM_MAX);
    thruster_M3.attach(THRUSTER_M3_PIN, PWM_MIN, PWM_MAX);
    thruster_M4.attach(THRUSTER_M4_PIN, PWM_MIN, PWM_MAX);
    
    // Initialize to stop
    stopAll();
    
    Serial.println("[THRUSTER] Attached to pins");
    Serial.println("[THRUSTER] Waiting for ESC arming...");
    
    delay(3000);  // Wait for ESC to arm
    
    Serial.println("[THRUSTER] Ready!");
    return true;
}

void ThrusterController::arm() {
    armed = true;
    Serial.println("[THRUSTER] Armed!");
}

void ThrusterController::setThruster(uint8_t id, int16_t pwm) {
    if (!armed) return;
    
    pwm = constrainPWM(pwm);
    
    switch(id) {
        case 1: thruster_M1.writeMicroseconds(pwm); break;
        case 2: thruster_M2.writeMicroseconds(pwm); break;
        case 3: thruster_M3.writeMicroseconds(pwm); break;
        case 4: thruster_M4.writeMicroseconds(pwm); break;
    }
}

void ThrusterController::setAllThrusters(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
    setThruster(1, m1);
    setThruster(2, m2);
    setThruster(3, m3);
    setThruster(4, m4);
}

void ThrusterController::stopAll() {
    thruster_M1.writeMicroseconds(PWM_STOP);
    thruster_M2.writeMicroseconds(PWM_STOP);
    thruster_M3.writeMicroseconds(PWM_STOP);
    thruster_M4.writeMicroseconds(PWM_STOP);
}

void ThrusterController::updatePID(float roll, float pitch, float yaw, float depth, float dt) {
    if (!armed) return;
    
    // Compute PID outputs
    float headingCorrection = pidHeadingEnabled ? pidHeading.compute(yaw, dt) : 0;
    float rollCorrection = pidRollEnabled ? pidRoll.compute(roll, dt) : 0;
    float pitchCorrection = pidPitchEnabled ? pidPitch.compute(pitch, dt) : 0;
    float depthCorrection = pidDepthEnabled ? pidDepth.compute(depth, dt) : 0;
    
    // Apply corrections to base speeds
    applyPIDCorrections(headingCorrection, rollCorrection, pitchCorrection, depthCorrection);
}

void ThrusterController::applyPIDCorrections(float headingCorr, float rollCorr, 
                                            float pitchCorr, float depthCorr) {
    // Base PWM from manual input
    int16_t baseM1 = PWM_STOP + baseForward + baseStrafe;
    int16_t baseM2 = PWM_STOP + baseForward - baseStrafe;
    int16_t baseM3 = PWM_STOP + baseVertical;
    int16_t baseM4 = PWM_STOP + baseVertical;
    
    // Apply heading correction (differential on M1, M2)
    int16_t m1PWM = baseM1 + headingCorr;
    int16_t m2PWM = baseM2 - headingCorr;
    
    // Apply roll correction (differential on M3, M4)
    int16_t m3PWM = baseM3 + rollCorr;
    int16_t m4PWM = baseM4 - rollCorr;
    
    // Apply pitch correction (affects M1, M2 and M3, M4)
    m1PWM += pitchCorr;
    m2PWM += pitchCorr;
    m3PWM -= pitchCorr * 0.5;  // Reduced effect on vertical thrusters
    m4PWM -= pitchCorr * 0.5;
    
    // Apply depth correction (affects M3, M4)
    m3PWM += depthCorr;
    m4PWM += depthCorr;
    
    // Set thrusters
    setAllThrusters(m1PWM, m2PWM, m3PWM, m4PWM);
}

void ThrusterController::setManualMode(int8_t forward, int8_t strafe, int8_t vertical, int8_t yaw) {
    baseForward = speedToPWM(forward);
    baseStrafe = speedToPWM(strafe);
    baseVertical = speedToPWM(vertical);
    baseYaw = speedToPWM(yaw);
}

void ThrusterController::moveForward(uint8_t speed) {
    setManualMode(speed, 0, 0, 0);
}

void ThrusterController::moveBackward(uint8_t speed) {
    setManualMode(-speed, 0, 0, 0);
}

void ThrusterController::strafeRight(uint8_t speed) {
    setManualMode(0, speed, 0, 0);
}

void ThrusterController::strafeLeft(uint8_t speed) {
    setManualMode(0, -speed, 0, 0);
}

void ThrusterController::ascend(uint8_t speed) {
    setManualMode(0, 0, speed, 0);
}

void ThrusterController::descend(uint8_t speed) {
    setManualMode(0, 0, -speed, 0);
}

void ThrusterController::enablePID(bool heading, bool roll, bool pitch, bool depth) {
    pidHeadingEnabled = heading;
    pidRollEnabled = roll;
    pidPitchEnabled = pitch;
    pidDepthEnabled = depth;
    
    Serial.print("[THRUSTER] PID Enabled - H:");
    Serial.print(heading); Serial.print(" R:");
    Serial.print(roll); Serial.print(" P:");
    Serial.print(pitch); Serial.print(" D:");
    Serial.println(depth);
}

int16_t ThrusterController::speedToPWM(int8_t speed) {
    // Convert -100 to +100 range to PWM offset
    return map(speed, -100, 100, -(PWM_STOP - PWM_MIN), (PWM_MAX - PWM_STOP));
}

int16_t ThrusterController::constrainPWM(int16_t pwm) {
    // Apply deadzone
    if (abs(pwm - PWM_STOP) < PWM_DEADZONE) {
        return PWM_STOP;
    }
    
    return constrain(pwm, PWM_MIN, PWM_MAX);
}

bool ThrusterController::isArmed() {
    return armed;
}

void ThrusterController::printStatus() {
    Serial.println("[THRUSTER] Status:");
    Serial.print("  Armed: "); Serial.println(armed);
    Serial.print("  PID Heading: "); Serial.println(pidHeading.getOutput());
    Serial.print("  PID Roll: "); Serial.println(pidRoll.getOutput());
    Serial.print("  PID Pitch: "); Serial.println(pidPitch.getOutput());
    Serial.print("  PID Depth: "); Serial.println(pidDepth.getOutput());
}