#pragma once
#include <Arduino.h>
// PIDConfig.h not required; remove dependency

class PIDController {
private:
    String name;
    
    // PID parameters
    float kp, ki, kd;
    float max_output;
    float max_integral;
    
    // State variables
    float error_prev = 0;
    float integral = 0;
    float derivative = 0;
    float output_prev = 0;
    
    // Anti-windup and filtering
    bool anti_windup_enabled = true;
    float error_deadband = 0.001;
    float derivative_cutoff_freq = 20.0; // Hz
    
    // Low-pass filter for derivative
    float alpha_d = 0.0; // Filter coefficient
    float derivative_filtered = 0;
    
public:
    PIDController(const String& name = "PID");
    
    // Configuration
    void setParameters(float kp, float ki, float kd, float max_output);
    void setAntiWindup(float max_integral);
    void setDeadband(float deadband);
    void setDerivativeFilter(float cutoff_freq, float sample_freq);
    
    // Update PID
    float update(float setpoint, float measurement, float dt);
    float update(float error, float dt);
    
    // Reset controller
    void reset();
    
    // Getters
    float getIntegral() const { return integral; }
    float getDerivative() const { return derivative_filtered; }
    float getLastOutput() const { return output_prev; }
    String getInfo() const;
    
    // Tuning interface
    void tuneZieglerNichols(float ku, float tu); // Ultimate gain method
    void tuneITAE(float tau, float theta);       // ITAE method
    
private:
    float applyLimits(float value, float min_val, float max_val);
    void updateDerivativeFilter(float raw_derivative, float dt);
    void applyAntiWindup();
};