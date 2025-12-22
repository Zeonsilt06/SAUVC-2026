#include "MadgwickAHRS.h"
#include <math.h>

MadgwickAHRS::MadgwickAHRS() : beta(0.1), zeta(0.0) {
    q = Quaternion(1, 0, 0, 0);
    w_bias = Vector3(0, 0, 0);
}

void MadgwickAHRS::begin(float sample_freq, float beta_def, float zeta_def) {
    beta = beta_def;
    zeta = zeta_def;
}

void MadgwickAHRS::updateIMU(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float dt) {
    // Normalize accelerometer measurement
    float norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm < 0.001) return; // Avoid division by zero
    
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    // Compute objective function and Jacobian for gradient descent
    // (Simplified version without magnetometer)
    float _2q0 = 2.0 * q.w;
    float _2q1 = 2.0 * q.x;
    float _2q2 = 2.0 * q.y;
    float _2q3 = 2.0 * q.z;
    float _4q0 = 4.0 * q.w;
    float _4q1 = 4.0 * q.x;
    float _4q2 = 4.0 * q.y;
    float _8q1 = 8.0 * q.x;
    float _8q2 = 8.0 * q.y;
    float q0q0 = q.w * q.w;
    float q1q1 = q.x * q.x;
    float q2q2 = q.y * q.y;
    float q3q3 = q.z * q.z;
    
    // Reference direction of gravity
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q.x - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0 * q0q0 * q.y + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0 * q1q1 * q.z - _2q1 * ax + 4.0 * q2q2 * q.z - _2q2 * ay;
    
    // Normalize step magnitude
    norm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norm > 0.0) {
        s0 /= norm;
        s1 /= norm;
        s2 /= norm;
        s3 /= norm;
    }
    
    // Compute error
    Vector3 error(s1, s2, s3);
    
    // Apply feedback to gyro measurements
    gx -= w_bias.x + error.x * beta;
    gy -= w_bias.y + error.y * beta;
    gz -= w_bias.z + error.z * beta;
    
    // Compute quaternion derivative
    Quaternion q_dot = q.derivative(Vector3(gx, gy, gz));
    
    // Integrate to get new quaternion
    q = q + q_dot * dt;
    q.normalize();
    
    // Update gyro bias estimate
    w_bias.x += error.x * zeta * dt;
    w_bias.y += error.y * zeta * dt;
    w_bias.z += error.z * zeta * dt;
}

void MadgwickAHRS::update(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float mx, float my, float mz,
                         float dt) {
    // Use 9DOF version with magnetometer
    update9DOF(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
}

void MadgwickAHRS::update9DOF(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float mx, float my, float mz,
                             float dt) {
    // Normalize accelerometer measurement
    float norm_accel = sqrt(ax * ax + ay * ay + az * az);
    if (norm_accel < 0.001) return;
    ax /= norm_accel;
    ay /= norm_accel;
    az /= norm_accel;
    
    // Normalize magnetometer measurement
    float norm_mag = sqrt(mx * mx + my * my + mz * mz);
    if (norm_mag < 0.001) return;
    mx /= norm_mag;
    my /= norm_mag;
    mz /= norm_mag;
    
    // Auxiliary variables to avoid repeated arithmetic
    float _2q0mx = 2.0 * q.w * mx;
    float _2q0my = 2.0 * q.w * my;
    float _2q0mz = 2.0 * q.w * mz;
    float _2q1mx = 2.0 * q.x * mx;
    float _2q0 = 2.0 * q.w;
    float _2q1 = 2.0 * q.x;
    float _2q2 = 2.0 * q.y;
    float _2q3 = 2.0 * q.z;
    float _2q0q2 = 2.0 * q.w * q.y;
    float _2q2q3 = 2.0 * q.y * q.z;
    float q0q0 = q.w * q.w;
    float q0q1 = q.w * q.x;
    float q0q2 = q.w * q.y;
    float q0q3 = q.w * q.z;
    float q1q1 = q.x * q.x;
    float q1q2 = q.x * q.y;
    float q1q3 = q.x * q.z;
    float q2q2 = q.y * q.y;
    float q2q3 = q.y * q.z;
    float q3q3 = q.z * q.z;
    
    // Reference direction of Earth's magnetic field
    float hx = mx * q0q0 - _2q0my * q.z + _2q0mz * q.y + mx * q1q1 + _2q1 * my * q.y + _2q1 * mz * q.z - mx * q2q2 - mx * q3q3;
    float hy = _2q0mx * q.z + my * q0q0 - _2q0mz * q.x + _2q1mx * q.y - my * q1q1 + my * q2q2 + _2q2 * mz * q.z - my * q3q3;
    float _2bx = sqrt(hx * hx + hy * hy);
    float _2bz = -_2q0mx * q.y + _2q0my * q.x + mz * q0q0 + _2q1mx * q.z - mz * q1q1 + _2q2 * my * q.z - mz * q2q2 + mz * q3q3;
    float _4bx = 2.0 * _2bx;
    float _4bz = 2.0 * _2bz;
    
    // Gradient descent algorithm corrective step
    float s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q.y * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q.z + _2bz * q.x) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q.y * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
    float s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q.x * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q.z * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q.y + _2bz * q.w) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q.z - _4bz * q.x) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
    float s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q.y * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q.y - _2bz * q.w) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q.x + _2bz * q.z) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q.w - _4bz * q.y) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
    float s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q.z + _2bz * q.x) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q.w + _2bz * q.y) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q.x * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
    
    // Normalize step magnitude
    norm_mag = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norm_mag > 0.0) {
        s0 /= norm_mag;
        s1 /= norm_mag;
        s2 /= norm_mag;
        s3 /= norm_mag;
    }
    
    // Apply feedback step
    Vector3 error(s1, s2, s3);
    
    // Compute estimated gyro bias
    w_bias.x += error.x * zeta * dt;
    w_bias.y += error.y * zeta * dt;
    w_bias.z += error.z * zeta * dt;
    
    // Apply gyro bias correction
    gx -= w_bias.x;
    gy -= w_bias.y;
    gz -= w_bias.z;
    
    // Apply feedback to gyro measurements
    gx += error.x * beta;
    gy += error.y * beta;
    gz += error.z * beta;
    
    // Integrate quaternion derivative
    Quaternion q_dot = q.derivative(Vector3(gx, gy, gz));
    q = q + q_dot * dt;
    q.normalize();
}

Quaternion MadgwickAHRS::getQuaternion() const {
    return q;
}

Vector3 MadgwickAHRS::getEulerAngles() const {
    return q.toEuler();
}

Vector3 MadgwickAHRS::getGravityVector() const {
    // Gravity vector in body frame: [0, 0, -g] rotated by inverse quaternion
    Vector3 gravity_world(0, 0, -9.81);
    Quaternion q_inv = q.inverse();
    return q_inv.rotate(gravity_world);
}

void MadgwickAHRS::setBeta(float beta) {
    this->beta = beta;
}

void MadgwickAHRS::setZeta(float zeta) {
    this->zeta = zeta;
}