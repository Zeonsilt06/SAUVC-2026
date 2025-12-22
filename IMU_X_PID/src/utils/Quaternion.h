#pragma once
#include "Vector3.h"
#include "Matrix3x3.h"
#include <math.h>

class Quaternion {
public:
    float w, x, y, z;

    Quaternion(float w = 1, float x = 0, float y = 0, float z = 0);

    // Construction helpers
    static Quaternion fromEuler(float roll, float pitch, float yaw);
    static Quaternion fromAxisAngle(const Vector3& axis, float angle);

    // Basic operations
    Quaternion operator+(const Quaternion& q) const;
    Quaternion operator-(const Quaternion& q) const;
    Quaternion operator*(const Quaternion& q) const;
    Quaternion operator*(float s) const;
    Quaternion operator/(float s) const;
    Quaternion& operator+=(const Quaternion& q);
    Quaternion& operator*=(float s);

    // Conjugate and inverse
    Quaternion conjugate() const;
    Quaternion inverse() const;

    // Norm operations
    float norm() const;
    float normSquared() const;
    void normalize();
    Quaternion normalized() const;

    // Derivative: q_dot = 0.5 * q * ω (where ω is angular velocity as pure quaternion)
    Quaternion derivative(const Vector3& omega) const;

    // Convert to Euler angles (roll, pitch, yaw)
    Vector3 toEuler() const;
    Matrix3x3 toRotationMatrix() const;

    // Rotate vector by quaternion: v' = q * v * q⁻¹
    Vector3 rotate(const Vector3& v) const;

    // Slerp interpolation
    static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float t);

    String toString() const;
};

// Non-member operator
Quaternion operator*(float s, const Quaternion& q);