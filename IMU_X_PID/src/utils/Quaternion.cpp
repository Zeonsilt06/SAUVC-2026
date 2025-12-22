#include "Quaternion.h"
#include <math.h>

Quaternion::Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

Quaternion Quaternion::fromAxisAngle(const Vector3& axis, float angle) {
    Vector3 norm_axis = axis.normalized();
    float half_angle = angle * 0.5;
    float sin_half = sin(half_angle);
    float cos_half = cos(half_angle);
    
    return Quaternion(
        cos_half,
        norm_axis.x * sin_half,
        norm_axis.y * sin_half,
        norm_axis.z * sin_half
    );
}

Quaternion Quaternion::fromEuler(float roll, float pitch, float yaw) {
    // Roll (X), Pitch (Y), Yaw (Z) rotations
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    
    return Quaternion(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
    );
}

Quaternion Quaternion::operator+(const Quaternion& q) const {
    return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
}

Quaternion Quaternion::operator-(const Quaternion& q) const {
    return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
}

Quaternion Quaternion::operator*(const Quaternion& q) const {
    // Hamilton product
    return Quaternion(
        w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w
    );
}

Quaternion Quaternion::operator*(float s) const {
    return Quaternion(w * s, x * s, y * s, z * s);
}

Quaternion Quaternion::operator/(float s) const {
    if (s != 0.0) {
        return Quaternion(w / s, x / s, y / s, z / s);
    }
    return *this;
}

Quaternion& Quaternion::operator+=(const Quaternion& q) {
    w += q.w;
    x += q.x;
    y += q.y;
    z += q.z;
    return *this;
}

Quaternion& Quaternion::operator*=(float s) {
    w *= s;
    x *= s;
    y *= s;
    z *= s;
    return *this;
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

float Quaternion::norm() const {
    return sqrt(w * w + x * x + y * y + z * z);
}

float Quaternion::normSquared() const {
    return w * w + x * x + y * y + z * z;
}

void Quaternion::normalize() {
    float n = norm();
    if (n > 0.0) {
        w /= n;
        x /= n;
        y /= n;
        z /= n;
    }
}

Quaternion Quaternion::normalized() const {
    float n = norm();
    if (n > 0.0) {
        return Quaternion(w / n, x / n, y / n, z / n);
    }
    return Quaternion(1, 0, 0, 0);
}

Quaternion Quaternion::inverse() const {
    float n2 = normSquared();
    if (n2 > 0.0) {
        Quaternion conj = conjugate();
        return conj / n2;
    }
    return Quaternion(1, 0, 0, 0);
}

Quaternion Quaternion::derivative(const Vector3& omega) const {
    // q_dot = 0.5 * q * omega (where omega is pure quaternion)
    Quaternion omega_q(0, omega.x, omega.y, omega.z);
    return (*this * omega_q) * 0.5;
}

Vector3 Quaternion::toEuler() const {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0 * (w * x + y * z);
    float cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    float roll = atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    float sinp = 2.0 * (w * y - z * x);
    float pitch;
    if (fabs(sinp) >= 1.0) {
        pitch = copysign(M_PI / 2.0, sinp); // Use 90 degrees if out of range
    } else {
        pitch = asin(sinp);
    }
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2.0 * (w * z + x * y);
    float cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    float yaw = atan2(siny_cosp, cosy_cosp);
    
    return Vector3(roll, pitch, yaw);
}

Vector3 Quaternion::rotate(const Vector3& v) const {
    // v' = q * v * q⁻¹
    Quaternion v_q(0, v.x, v.y, v.z);
    Quaternion q_inv = inverse();
    Quaternion result = (*this * v_q) * q_inv;
    
    return Vector3(result.x, result.y, result.z);
}

Quaternion Quaternion::slerp(const Quaternion& q1, const Quaternion& q2, float t) {
    // Ensure t is in [0, 1]
    t = fmax(0.0, fmin(1.0, t));
    
    // Calculate cosine of angle between quaternions
    float cos_half_theta = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
    
    // If q1 = q2 or q1 = -q2, return q1
    if (fabs(cos_half_theta) >= 1.0) {
        return q1;
    }
    
    // Calculate temporary values
    float half_theta = acos(cos_half_theta);
    float sin_half_theta = sqrt(1.0 - cos_half_theta * cos_half_theta);
    
    // If theta = 0, return q1
    if (fabs(sin_half_theta) < 0.001) {
        return q1;
    }
    
    float ratio_a = sin((1.0 - t) * half_theta) / sin_half_theta;
    float ratio_b = sin(t * half_theta) / sin_half_theta;
    
    return Quaternion(
        q1.w * ratio_a + q2.w * ratio_b,
        q1.x * ratio_a + q2.x * ratio_b,
        q1.y * ratio_a + q2.y * ratio_b,
        q1.z * ratio_a + q2.z * ratio_b
    );
}

String Quaternion::toString() const {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "w:%.4f, x:%.4f, y:%.4f, z:%.4f", w, x, y, z);
    return String(buffer);
}

Matrix3x3 Quaternion::toRotationMatrix() const {
    // Convert quaternion to 3x3 rotation matrix
    float ww = w*w, xx = x*x, yy = y*y, zz = z*z;
    float wx = w*x, wy = w*y, wz = w*z;
    float xy = x*y, xz = x*z, yz = y*z;

    return Matrix3x3(
        ww + xx - yy - zz, 2.0f*(xy - wz),     2.0f*(xz + wy),
        2.0f*(xy + wz),     ww - xx + yy - zz, 2.0f*(yz - wx),
        2.0f*(xz - wy),     2.0f*(yz + wx),     ww - xx - yy + zz
    );
}

// Non-member operator
Quaternion operator*(float s, const Quaternion& q) {
    return q * s;
}