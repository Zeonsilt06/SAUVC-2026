#include "Matrix3x3.h"
#include <math.h>

Matrix3x3::Matrix3x3() {
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) m[i][j] = 0.0f;
}

Matrix3x3::Matrix3x3(float m00, float m01, float m02,
                     float m10, float m11, float m12,
                     float m20, float m21, float m22) {
    m[0][0]=m00; m[0][1]=m01; m[0][2]=m02;
    m[1][0]=m10; m[1][1]=m11; m[1][2]=m12;
    m[2][0]=m20; m[2][1]=m21; m[2][2]=m22;
}

Vector3 Matrix3x3::operator*(const Vector3& v) const {
    return Vector3(
        m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
        m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
        m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
    );
}

Matrix3x3 Matrix3x3::identity() {
    return Matrix3x3(1,0,0, 0,1,0, 0,0,1);
}

Matrix3x3 Matrix3x3::inverse() const {
    // Compute matrix inverse using analytic formula for 3x3
    float a = m[0][0], b = m[0][1], c = m[0][2];
    float d = m[1][0], e = m[1][1], f = m[1][2];
    float g = m[2][0], h = m[2][1], i = m[2][2];

    float A =   e*i - f*h;
    float B = -(d*i - f*g);
    float C =   d*h - e*g;

    float D = -(b*i - c*h);
    float E =   a*i - c*g;
    float F = -(a*h - b*g);

    float G =   b*f - c*e;
    float H = -(a*f - c*d);
    float I =   a*e - b*d;

    float det = a*A + b*B + c*C;
    if (fabs(det) < 1e-9) {
        // Return identity as fallback to avoid division by zero
        return Matrix3x3::identity();
    }

    float invdet = 1.0f / det;
    return Matrix3x3(
        A*invdet, D*invdet, G*invdet,
        B*invdet, E*invdet, H*invdet,
        C*invdet, F*invdet, I*invdet
    );
}
