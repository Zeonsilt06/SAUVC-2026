// Minimal 3x3 matrix utility for dynamics
#pragma once
#include "Vector3.h"

class Matrix3x3 {
public:
    float m[3][3];

    Matrix3x3();
    Matrix3x3(float m00, float m01, float m02,
              float m10, float m11, float m12,
              float m20, float m21, float m22);

    Vector3 operator*(const Vector3& v) const;
    Matrix3x3 inverse() const;
    static Matrix3x3 identity();
};
