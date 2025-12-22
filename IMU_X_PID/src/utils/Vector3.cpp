#include "Vector3.h"
#include <Arduino.h>

Vector3::Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

Vector3 Vector3::operator+(const Vector3& v) const {
    return Vector3(x + v.x, y + v.y, z + v.z);
}

Vector3 Vector3::operator-(const Vector3& v) const {
    return Vector3(x - v.x, y - v.y, z - v.z);
}

Vector3 Vector3::operator-() const {
    return Vector3(-x, -y, -z);
}

Vector3 Vector3::operator*(float s) const {
    return Vector3(x * s, y * s, z * s);
}

Vector3 Vector3::operator/(float s) const {
    if (s != 0.0) {
        return Vector3(x / s, y / s, z / s);
    }
    return *this;
}

Vector3& Vector3::operator+=(const Vector3& v) {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
}

Vector3& Vector3::operator*=(float s) {
    x *= s;
    y *= s;
    z *= s;
    return *this;
}

Vector3& Vector3::operator/=(float s) {
    if (s != 0.0) {
        x /= s;
        y /= s;
        z /= s;
    }
    return *this;
}

bool Vector3::operator==(const Vector3& v) const {
    return (fabs(x - v.x) < 1e-6) && (fabs(y - v.y) < 1e-6) && (fabs(z - v.z) < 1e-6);
}

bool Vector3::operator!=(const Vector3& v) const {
    return !(*this == v);
}

float Vector3::dot(const Vector3& v) const {
    return x * v.x + y * v.y + z * v.z;
}

Vector3 Vector3::cross(const Vector3& v) const {
    return Vector3(
        y * v.z - z * v.y,
        z * v.x - x * v.z,
        x * v.y - y * v.x
    );
}

float Vector3::norm() const {
    return sqrt(x * x + y * y + z * z);
}

float Vector3::normSquared() const {
    return x * x + y * y + z * z;
}

Vector3 Vector3::normalized() const {
    float n = norm();
    if (n > 0.0) {
        return Vector3(x / n, y / n, z / n);
    }
    return Vector3(0, 0, 0);
}

void Vector3::normalize() {
    float n = norm();
    if (n > 0.0) {
        x /= n;
        y /= n;
        z /= n;
    }
}

float Vector3::angle(const Vector3& v) const {
    float n1 = norm();
    float n2 = v.norm();
    if (n1 > 0.0 && n2 > 0.0) {
        float cos_angle = dot(v) / (n1 * n2);
        // Clamp to [-1, 1] to avoid numerical errors
        if (cos_angle > 1.0) cos_angle = 1.0;
        if (cos_angle < -1.0) cos_angle = -1.0;
        return acos(cos_angle);
    }
    return 0.0;
}

Vector3 Vector3::rotate(const Vector3& axis, float angle) const {
    // Rodrigues rotation formula: v_rot = v*cosθ + (k×v)*sinθ + k*(k·v)*(1-cosθ)
    Vector3 k = axis.normalized();
    Vector3 v = *this;
    
    float cos_theta = cos(angle);
    float sin_theta = sin(angle);
    
    return v * cos_theta + k.cross(v) * sin_theta + k * k.dot(v) * (1 - cos_theta);
}

String Vector3::toString() const {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.4f, %.4f, %.4f", x, y, z);
    return String(buffer);
}

Vector3 operator*(float s, const Vector3& v) {
    return v * s;
}