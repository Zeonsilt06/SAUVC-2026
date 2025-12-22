#pragma once
#include "../utils/Vector3.h"
#include "../utils/Quaternion.h"
#include "../utils/Matrix3x3.h"

struct VehicleState {
    Quaternion orientation;      // Current orientation
    Vector3 position;            // World position
    Vector3 velocity;            // World velocity
    Vector3 angular_velocity;    // Body angular velocity
    Vector3 acceleration;        // Body acceleration
    
    void reset() {
        orientation = Quaternion(1, 0, 0, 0);
        position = Vector3(0, 0, 0);
        velocity = Vector3(0, 0, 0);
        angular_velocity = Vector3(0, 0, 0);
        acceleration = Vector3(0, 0, 0);
    }
};

struct VehicleParameters {
    float mass = 1.0;           // kg
    float arm_length = 0.15;    // m
    Matrix3x3 inertia;          // Moment of inertia tensor
    
    // Aerodynamic coefficients
    float drag_coefficient = 0.1;
    float lift_coefficient = 0.5;
    
    // Motor parameters
    float motor_time_constant = 0.02; // s
    float motor_max_thrust = 4.0;     // N per motor
    float motor_min_thrust = 0.1;     // N per motor
};

class QuadcopterDynamics {
private:
    VehicleState state;
    VehicleParameters params;
    
    // Control inputs
    Vector3 torques = Vector3(0, 0, 0);
    float total_thrust = 0;
    
    // Environmental
    const Vector3 gravity = Vector3(0, 0, -9.81);
    float air_density = 1.225; // kg/m³
    
public:
    QuadcopterDynamics();
    
    // Configuration
    void setMass(float mass);
    void setArmLength(float length);
    void setInertia(float Ixx, float Iyy, float Izz);
    
    // State update
    void update(float dt);
    void updateRK4(float dt); // Runge-Kutta 4th order
    
    // Apply controls
    void setControls(const Vector3& torques, float thrust);
    
    // Physics calculations
    Vector3 calculateForces() const;
    Vector3 calculateTorques() const;
    Matrix3x3 calculateRotationMatrix() const;
    
    // Limits and constraints
    Vector3 applyLimits(const Vector3& torques) const;
    float applyThrustLimit(float thrust) const;
    
    // Get state
    const VehicleState& getState() const { return state; }
    const VehicleParameters& getParams() const { return params; }
    String getStateString() const;
    
    // Reset state
    void reset();
    
private:
    // Differential equations for state derivatives
    void stateDerivatives(const VehicleState& state,
                         Vector3& pos_dot,
                         Vector3& vel_dot,
                         Vector3& ang_vel_dot,
                         Quaternion& quat_dot) const;
    
    // Helper functions
    Vector3 worldToBody(const Vector3& world_vec) const;
    Vector3 bodyToWorld(const Vector3& body_vec) const;
    Quaternion quaternionDerivative(const Quaternion& q, 
                                   const Vector3& omega) const;
};