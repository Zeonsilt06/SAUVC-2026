#pragma once
#include <Arduino.h>
#include "../utils/Vector3.h"

enum MixerConfig {
    QUAD_X,      // X configuration
    QUAD_PLUS,   // + configuration
    QUAD_Y4,     // Y4 configuration
    HEX_X,       // Hexacopter X
    OCTO_X8      // Octocopter X8
};

class ThrusterMixer {
private:
    MixerConfig config = QUAD_X;
    
    // Thruster layout matrix (depends on configuration)
    float mix_matrix[4][4]; // For 4 outputs × 4 inputs (thrust, roll, pitch, yaw)
    
    // Physical properties
    float arm_length = 0.15;      // Distance from center to thruster (m)
    float torque_coefficient = 0.05; // Propeller torque coefficient
    
    // Output limits
    float min_thrust = 0.0;
    float max_thrust = 1.0;
    
public:
    ThrusterMixer();
    
    // Configuration
    void setConfiguration(MixerConfig config);
    void setArmLength(float length);
    void setTorqueCoefficient(float coeff);
    
    // Mixing function - converts desired forces/torques to thruster commands
    void mix(float* thruster_outputs,            // Output array (size 4)
             float thrust,                       // Total thrust (0-1)
             const Vector3& torques);            // Roll, Pitch, Yaw torques
    
    // Alternative mixing with full 6DOF control
    void mix6DOF(float* thruster_outputs,        // Output array (size 4)
                 const Vector3& forces,          // X, Y, Z forces
                 const Vector3& torques);        // Roll, Pitch, Yaw torques
    
    // Safety features
    void applyLimits(float* outputs, int count);
    void applyMotorFailure(float* outputs, int failed_motor);
    void applySlewRateLimit(float* outputs, float* prev_outputs, 
                           float max_change, int count);
    
    // Matrix operations
    void updateMixMatrix();
    void printMixMatrix();
    
private:
    void setupQuadXMatrix();
    void setupQuadPlusMatrix();
    void setupQuadY4Matrix();
    
    // Helper functions
    float saturate(float value, float min_val, float max_val);
    void normalizeOutputs(float* outputs, int count);
};