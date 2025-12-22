#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(const String& name) : name(name) {
    reset();
}

void PIDController::setParameters(float kp, float ki, float kd, float max_output) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->max_output = max_output;
    this->max_integral = max_output * 0.5; // Default anti-windup limit
}

void PIDController::setAntiWindup(float max_integral) {
    this->max_integral = max_integral;
    this->anti_windup_enabled = true;
}

void PIDController::setDeadband(float deadband) {
    this->error_deadband = deadband;
}

void PIDController::setDerivativeFilter(float cutoff_freq, float sample_freq) {
    if (cutoff_freq > 0 && sample_freq > 0) {
        // First order low-pass filter coefficient
        float RC = 1.0 / (2.0 * M_PI * cutoff_freq);
        float dt = 1.0 / sample_freq;
        this->alpha_d = dt / (RC + dt);
    }
}

float PIDController::update(float setpoint, float measurement, float dt) {
    if (dt <= 0.0) {
        return output_prev;
    }
    
    // Calculate error
    float error = setpoint - measurement;
    
    // Apply deadband
    if (fabs(error) < error_deadband) {
        error = 0.0;
    }
    
    // Update PID terms
    return update(error, dt);
}

float PIDController::update(float error, float dt) {
    if (dt <= 0.0) {
        return output_prev;
    }
    
    // Proportional term
    float proportional = kp * error;
    
    // Integral term (with anti-windup)
    integral += ki * error * dt;
    if (anti_windup_enabled) {
        if (integral > max_integral) integral = max_integral;
        if (integral < -max_integral) integral = -max_integral;
    }
    
    // Derivative term (with filtering)
    if (dt > 0.0) {
        float raw_derivative = (error - error_prev) / dt;
        
        // Apply low-pass filter to derivative
        if (alpha_d > 0.0) {
            derivative_filtered = derivative_filtered * (1.0 - alpha_d) + raw_derivative * alpha_d;
        } else {
            derivative_filtered = raw_derivative;
        }
    }
    
    // Calculate output
    float output = proportional + integral + kd * derivative_filtered;
    
    // Apply output limits
    if (output > max_output) output = max_output;
    if (output < -max_output) output = -max_output;
    
    // Store previous values
    error_prev = error;
    output_prev = output;
    
    return output;
}

void PIDController::reset() {
    error_prev = 0.0;
    integral = 0.0;
    derivative = 0.0;
    derivative_filtered = 0.0;
    output_prev = 0.0;
}

void PIDController::tuneZieglerNichols(float ku, float tu) {
    // Ziegler-Nichols method for PID tuning
    // ku: ultimate gain (when system oscillates)
    // tu: oscillation period
    
    if (tu > 0.0) {
        kp = 0.6 * ku;
        ki = 2.0 * kp / tu;
        kd = kp * tu / 8.0;
        max_output = ku * 2.0; // Conservative limit
    }
}

void PIDController::tuneITAE(float tau, float theta) {
    // ITAE (Integral Time Absolute Error) tuning method
    // tau: process time constant
    // theta: process dead time
    
    if (tau > 0.0) {
        float ratio = theta / tau;
        
        // ITAE tuning rules for step response
        if (ratio < 0.1) {
            kp = 0.042 / tau;
            ki = 0.94 / tau;
            kd = 0.43 * tau;
        } else if (ratio < 1.0) {
            kp = (0.9 - 0.08 * ratio) / tau;
            ki = (0.33 - 0.03 * ratio) / tau;
            kd = (0.27 - 0.02 * ratio) * tau;
        } else {
            kp = 0.8 / tau;
            ki = 0.25 / tau;
            kd = 0.25 * tau;
        }
    }
}

String PIDController::getInfo() const {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), 
             "%s: Kp=%.3f, Ki=%.3f, Kd=%.3f, I=%.3f, D=%.3f",
             name.c_str(), kp, ki, kd, integral, derivative_filtered);
    return String(buffer);
}