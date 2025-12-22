#include "ThrusterMixer.h"
#include <math.h>

ThrusterMixer::ThrusterMixer() {
    setConfiguration(QUAD_X);
}

void ThrusterMixer::setConfiguration(MixerConfig config) {
    this->config = config;
    updateMixMatrix();
}

void ThrusterMixer::setArmLength(float length) {
    this->arm_length = length;
    updateMixMatrix();
}

void ThrusterMixer::setTorqueCoefficient(float coeff) {
    this->torque_coefficient = coeff;
    updateMixMatrix();
}

void ThrusterMixer::mix(float* thruster_outputs, float thrust, const Vector3& torques) {
    // Input vector: [thrust, τ_roll, τ_pitch, τ_yaw]
    float inputs[4] = {
        thrust,
        torques.x,  // Roll torque
        torques.y,  // Pitch torque
        torques.z   // Yaw torque
    };
    
    // Matrix multiplication: outputs = mix_matrix × inputs
    for (int i = 0; i < 4; i++) {
        thruster_outputs[i] = 0;
        for (int j = 0; j < 4; j++) {
            thruster_outputs[i] += mix_matrix[i][j] * inputs[j];
        }
    }
    
    // Apply limits and ensure positive thrust
    applyLimits(thruster_outputs, 4);
}

void ThrusterMixer::mix6DOF(float* thruster_outputs, const Vector3& forces, const Vector3& torques) {
    // For 4 thrusters, we can't fully control 6DOF directly
    // This uses pseudo-inverse or prioritization
    
    if (config == QUAD_X || config == QUAD_PLUS) {
        // For aerial vehicles, we control thrust (Z) and torques (roll, pitch, yaw)
        // X and Y forces are achieved by tilting (not directly)
        mix(thruster_outputs, forces.z, torques);
    } else {
        // For other configurations, use the standard mix
        mix(thruster_outputs, forces.z, torques);
    }
}

void ThrusterMixer::applyLimits(float* outputs, int count) {
    for (int i = 0; i < count; i++) {
        // Ensure outputs are between 0 and 1
        if (outputs[i] < min_thrust) outputs[i] = min_thrust;
        if (outputs[i] > max_thrust) outputs[i] = max_thrust;
        
        // Ensure minimum thrust for motor spin-up
        if (outputs[i] > 0 && outputs[i] < 0.1) outputs[i] = 0.1;
    }
}

void ThrusterMixer::applyMotorFailure(float* outputs, int failed_motor) {
    if (failed_motor >= 0 && failed_motor < 4) {
        outputs[failed_motor] = 0;
        
        // Redistribute thrust to maintain stability
        float remaining_thrust = 0;
        for (int i = 0; i < 4; i++) {
            if (i != failed_motor) {
                remaining_thrust += outputs[i];
            }
        }
        
        // Scale up remaining motors
        if (remaining_thrust > 0) {
            float scale = (outputs[0] + outputs[1] + outputs[2] + outputs[3]) / remaining_thrust;
            for (int i = 0; i < 4; i++) {
                if (i != failed_motor) {
                    outputs[i] *= scale;
                }
            }
        }
    }
}

void ThrusterMixer::applySlewRateLimit(float* outputs, float* prev_outputs, float max_change, int count) {
    for (int i = 0; i < count; i++) {
        float change = outputs[i] - prev_outputs[i];
        if (fabs(change) > max_change) {
            if (change > 0) {
                outputs[i] = prev_outputs[i] + max_change;
            } else {
                outputs[i] = prev_outputs[i] - max_change;
            }
        }
    }
}

void ThrusterMixer::updateMixMatrix() {
    switch (config) {
        case QUAD_X:
            setupQuadXMatrix();
            break;
        case QUAD_PLUS:
            setupQuadPlusMatrix();
            break;
        case QUAD_Y4:
            setupQuadY4Matrix();
            break;
        default:
            setupQuadXMatrix();
            break;
    }
}

void ThrusterMixer::setupQuadXMatrix() {
    // X configuration (45° between arms)
    // Motors: M1: front-left, M2: front-right, M3: back-right, M4: back-left
    // M1 and M3: clockwise, M2 and M4: counter-clockwise
    
    float L = arm_length * 0.7071; // arm_length * sin(45°)
    
    // Mixing matrix (normalized)
    // Each row: [thrust, roll, pitch, yaw] contributions
    
    // Motor 1 (front-left, CW)
    mix_matrix[0][0] =  0.25;  // thrust
    mix_matrix[0][1] = -L;     // roll (negative because left side)
    mix_matrix[0][2] =  L;     // pitch (positive because front)
    mix_matrix[0][3] =  torque_coefficient; // yaw (positive for CW)
    
    // Motor 2 (front-right, CCW)
    mix_matrix[1][0] =  0.25;
    mix_matrix[1][1] =  L;     // roll (positive because right side)
    mix_matrix[1][2] =  L;     // pitch (positive because front)
    mix_matrix[1][3] = -torque_coefficient; // yaw (negative for CCW)
    
    // Motor 3 (back-right, CW)
    mix_matrix[2][0] =  0.25;
    mix_matrix[2][1] =  L;     // roll (positive because right side)
    mix_matrix[2][2] = -L;     // pitch (negative because back)
    mix_matrix[2][3] =  torque_coefficient; // yaw (positive for CW)
    
    // Motor 4 (back-left, CCW)
    mix_matrix[3][0] =  0.25;
    mix_matrix[3][1] = -L;     // roll (negative because left side)
    mix_matrix[3][2] = -L;     // pitch (negative because back)
    mix_matrix[3][3] = -torque_coefficient; // yaw (negative for CCW)
}

void ThrusterMixer::setupQuadPlusMatrix() {
    // + configuration (arms at 0°, 90°, 180°, 270°)
    // Motors: M1: front, M2: right, M3: back, M4: left
    // M1 and M3: clockwise, M2 and M4: counter-clockwise
    
    // Mixing matrix for + configuration
    mix_matrix[0][0] =  0.25;  // M1: front
    mix_matrix[0][1] =  0.0;   // no roll contribution
    mix_matrix[0][2] =  arm_length; // pitch
    mix_matrix[0][3] =  torque_coefficient; // yaw
    
    mix_matrix[1][0] =  0.25;  // M2: right
    mix_matrix[1][1] =  arm_length; // roll
    mix_matrix[1][2] =  0.0;   // no pitch contribution
    mix_matrix[1][3] = -torque_coefficient; // yaw
    
    mix_matrix[2][0] =  0.25;  // M3: back
    mix_matrix[2][1] =  0.0;   // no roll contribution
    mix_matrix[2][2] = -arm_length; // pitch
    mix_matrix[2][3] =  torque_coefficient; // yaw
    
    mix_matrix[3][0] =  0.25;  // M4: left
    mix_matrix[3][1] = -arm_length; // roll
    mix_matrix[3][2] =  0.0;   // no pitch contribution
    mix_matrix[3][3] = -torque_coefficient; // yaw
}

void ThrusterMixer::setupQuadY4Matrix() {
    // Y4 configuration (tricopter-like with two front motors)
    // Not commonly used for quadcopters
    setupQuadXMatrix(); // Default to X for now
}

float ThrusterMixer::saturate(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

void ThrusterMixer::normalizeOutputs(float* outputs, int count) {
    float max_val = 0;
    for (int i = 0; i < count; i++) {
        if (fabs(outputs[i]) > max_val) {
            max_val = fabs(outputs[i]);
        }
    }
    
    if (max_val > 1.0) {
        for (int i = 0; i < count; i++) {
            outputs[i] /= max_val;
        }
    }
}

void ThrusterMixer::printMixMatrix() {
    Serial.println("Thruster Mixing Matrix:");
    for (int i = 0; i < 4; i++) {
        Serial.print("Motor ");
        Serial.print(i);
        Serial.print(": ");
        for (int j = 0; j < 4; j++) {
            Serial.print(mix_matrix[i][j], 4);
            Serial.print(" ");
        }
        Serial.println();
    }
}