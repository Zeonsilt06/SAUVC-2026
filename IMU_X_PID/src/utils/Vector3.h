#pragma once
#include <Arduino.h>
#include <math.h>

class Vector3 {
public:
    float x, y, z;

    Vector3(float x = 0, float y = 0, float z = 0);

    // Basic operations
    Vector3 operator+(const Vector3& v) const;
    Vector3 operator-(const Vector3& v) const;
    Vector3 operator-() const;  // Unary minus
    Vector3 operator*(float s) const;
    Vector3 operator/(float s) const;

    Vector3& operator+=(const Vector3& v);
    Vector3& operator-=(const Vector3& v);
    Vector3& operator*=(float s);
    Vector3& operator/=(float s);

    bool operator==(const Vector3& v) const;
    bool operator!=(const Vector3& v) const;

    // Dot and cross products
    float dot(const Vector3& v) const;
    Vector3 cross(const Vector3& v) const;

    // Norm operations
    float norm() const;
    float normSquared() const;
    Vector3 normalized() const;
    void normalize();

    // Angle between vectors
    float angle(const Vector3& v) const;

    // Rotation (Rodrigues formula)
    Vector3 rotate(const Vector3& axis, float angle) const;

    String toString() const;
};

// Non-member operator
Vector3 operator*(float s, const Vector3& v);