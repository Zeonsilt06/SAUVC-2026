#include "QuadcopterDynamics.h"
#include <math.h>

QuadcopterDynamics::QuadcopterDynamics() {
    reset();
}

void QuadcopterDynamics::setMass(float mass) {
    if (mass > 0) {
        params.mass = mass;
    }
}

void QuadcopterDynamics::setArmLength(float length) {
    if (length > 0) {
        params.arm_length = length;
    }
}

void QuadcopterDynamics::setInertia(float Ixx, float Iyy, float Izz) {
    params.inertia = Matrix3x3(
        Ixx, 0, 0,
        0, Iyy, 0,
        0, 0, Izz
    );
}

void QuadcopterDynamics::update(float dt) {
    if (dt <= 0) return;
    
    // Simple Euler integration for now
    Vector3 pos_dot, vel_dot, ang_vel_dot;
    Quaternion quat_dot;
    
    stateDerivatives(state, pos_dot, vel_dot, ang_vel_dot, quat_dot);
    
    // Update state
    state.position = state.position + pos_dot * dt;
    state.velocity = state.velocity + vel_dot * dt;
    state.angular_velocity = state.angular_velocity + ang_vel_dot * dt;
    state.orientation = state.orientation + quat_dot * dt;
    state.orientation.normalize();
    
    // Update acceleration
    state.acceleration = vel_dot;
}

void QuadcopterDynamics::updateRK4(float dt) {
    // Runge-Kutta 4th order integration (more accurate)
    VehicleState k1_state = state;
    Vector3 k1_pos_dot, k1_vel_dot, k1_ang_vel_dot;
    Quaternion k1_quat_dot;
    
    stateDerivatives(k1_state, k1_pos_dot, k1_vel_dot, k1_ang_vel_dot, k1_quat_dot);
    
    VehicleState k2_state = state;
    k2_state.position = state.position + k1_pos_dot * (dt * 0.5);
    k2_state.velocity = state.velocity + k1_vel_dot * (dt * 0.5);
    k2_state.angular_velocity = state.angular_velocity + k1_ang_vel_dot * (dt * 0.5);
    k2_state.orientation = state.orientation + k1_quat_dot * (dt * 0.5);
    k2_state.orientation.normalize();
    
    Vector3 k2_pos_dot, k2_vel_dot, k2_ang_vel_dot;
    Quaternion k2_quat_dot;
    stateDerivatives(k2_state, k2_pos_dot, k2_vel_dot, k2_ang_vel_dot, k2_quat_dot);
    
    VehicleState k3_state = state;
    k3_state.position = state.position + k2_pos_dot * (dt * 0.5);
    k3_state.velocity = state.velocity + k2_vel_dot * (dt * 0.5);
    k3_state.angular_velocity = state.angular_velocity + k2_ang_vel_dot * (dt * 0.5);
    k3_state.orientation = state.orientation + k2_quat_dot * (dt * 0.5);
    k3_state.orientation.normalize();
    
    Vector3 k3_pos_dot, k3_vel_dot, k3_ang_vel_dot;
    Quaternion k3_quat_dot;
    stateDerivatives(k3_state, k3_pos_dot, k3_vel_dot, k3_ang_vel_dot, k3_quat_dot);
    
    VehicleState k4_state = state;
    k4_state.position = state.position + k3_pos_dot * dt;
    k4_state.velocity = state.velocity + k3_vel_dot * dt;
    k4_state.angular_velocity = state.angular_velocity + k3_ang_vel_dot * dt;
    k4_state.orientation = state.orientation + k3_quat_dot * dt;
    k4_state.orientation.normalize();
    
    Vector3 k4_pos_dot, k4_vel_dot, k4_ang_vel_dot;
    Quaternion k4_quat_dot;
    stateDerivatives(k4_state, k4_pos_dot, k4_vel_dot, k4_ang_vel_dot, k4_quat_dot);
    
    // Combine derivatives
    state.position = state.position + (k1_pos_dot + 2.0f*k2_pos_dot + 2.0f*k3_pos_dot + k4_pos_dot) * (dt / 6.0);
    state.velocity = state.velocity + (k1_vel_dot + 2.0f*k2_vel_dot + 2.0f*k3_vel_dot + k4_vel_dot) * (dt / 6.0);
    state.angular_velocity = state.angular_velocity + (k1_ang_vel_dot + 2.0f*k2_ang_vel_dot + 2.0f*k3_ang_vel_dot + k4_ang_vel_dot) * (dt / 6.0);
    state.orientation = state.orientation + (k1_quat_dot + 2.0f*k2_quat_dot + 2.0f*k3_quat_dot + k4_quat_dot) * (dt / 6.0);
    state.orientation.normalize();
    
    // Update acceleration
    state.acceleration = (k1_vel_dot + 2.0f*k2_vel_dot + 2.0f*k3_vel_dot + k4_vel_dot) * (1.0f / 6.0f);
}

void QuadcopterDynamics::setControls(const Vector3& torques, float thrust) {
    this->torques = applyLimits(torques);
    this->total_thrust = applyThrustLimit(thrust);
}

Vector3 QuadcopterDynamics::calculateForces() const {
    // Forces in body frame: thrust + drag
    Vector3 F_body(0, 0, total_thrust);
    
    // Simple aerodynamic drag (proportional to velocity)
    Vector3 drag = -state.velocity * params.drag_coefficient;
    F_body = F_body + drag;
    
    return F_body;
}

Vector3 QuadcopterDynamics::calculateTorques() const {
    return torques;
}

Matrix3x3 QuadcopterDynamics::calculateRotationMatrix() const {
    return state.orientation.toRotationMatrix();
}

Vector3 QuadcopterDynamics::applyLimits(const Vector3& torques) const {
    Vector3 limited = torques;
    
    // Apply torque limits based on motor capabilities
    float max_torque = params.motor_max_thrust * params.arm_length * sqrt(2.0);
    
    if (limited.norm() > max_torque) {
        limited = limited.normalized() * max_torque;
    }
    
    return limited;
}

float QuadcopterDynamics::applyThrustLimit(float thrust) const {
    return fmax(params.motor_min_thrust * 4.0, fmin(thrust, params.motor_max_thrust * 4.0));
}

void QuadcopterDynamics::reset() {
    state.reset();
    torques = Vector3(0, 0, 0);
    total_thrust = 0;
}

void QuadcopterDynamics::stateDerivatives(const VehicleState& state,
                                         Vector3& pos_dot,
                                         Vector3& vel_dot,
                                         Vector3& ang_vel_dot,
                                         Quaternion& quat_dot) const {
    // Position derivative = velocity
    pos_dot = state.velocity;
    
    // Calculate rotation matrix
    Matrix3x3 R = calculateRotationMatrix();
    
    // Forces in body frame
    Vector3 F_body = calculateForces();
    
    // Convert forces to world frame
    Vector3 F_world = R * F_body;
    
    // Add gravity
    F_world.z += params.mass * -9.81;
    
    // Acceleration in world frame (F = ma)
    vel_dot = F_world / params.mass;
    
    // Angular acceleration (Euler's rotation equations)
    Vector3 tau = calculateTorques();
    
    // Simple damping
    tau = tau - state.angular_velocity * 0.1;
    
    // τ = Iα + ω × (Iω)
    Vector3 Iomega = params.inertia * state.angular_velocity;
    Vector3 omega_cross_Iomega = state.angular_velocity.cross(Iomega);
    ang_vel_dot = params.inertia.inverse() * (tau - omega_cross_Iomega);
    
    // Quaternion derivative
    quat_dot = state.orientation.derivative(state.angular_velocity);
}

Vector3 QuadcopterDynamics::worldToBody(const Vector3& world_vec) const {
    Quaternion q_inv = state.orientation.inverse();
    return q_inv.rotate(world_vec);
}

Vector3 QuadcopterDynamics::bodyToWorld(const Vector3& body_vec) const {
    return state.orientation.rotate(body_vec);
}

String QuadcopterDynamics::getStateString() const {
    char buffer[256];
    Vector3 euler = state.orientation.toEuler();
    snprintf(buffer, sizeof(buffer),
             "Pos: %s | Vel: %s | RPY: %.2f,%.2f,%.2f | Thrust: %.2f",
             state.position.toString().c_str(),
             state.velocity.toString().c_str(),
             euler.x * 57.2958, euler.y * 57.2958, euler.z * 57.2958,
             total_thrust);
    return String(buffer);
}