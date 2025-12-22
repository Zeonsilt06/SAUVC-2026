#ifndef ANGLEKALMAN_H
#define ANGLEKALMAN_H

class AngleKalman {
public:
    AngleKalman() {
        Q_angle = 0.001f; Q_bias = 0.003f; R_measure = 0.03f;
        angle = 0.0f; bias = 0.0f;
        P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0.0f;
    }
    float update(float newRate, float newAngle, float dt) {
        float rate = newRate - bias;
        angle += dt * rate;
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;
        float y = newAngle - angle;
        if (y > 180.0f) y -= 360.0f;
        if (y <= -180.0f) y += 360.0f;
        float S = P[0][0] + R_measure;
        float K0 = P[0][0] / S;
        float K1 = P[1][0] / S;
        angle += K0 * y;
        bias += K1 * y;
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];
        P[0][0] = (1 - K0) * P00_temp;
        P[0][1] = (1 - K0) * P01_temp;
        P[1][0] -= K1 * P00_temp;
        P[1][1] -= K1 * P01_temp;
        return angle;
    }
    void setAngle(float a) { angle = a; }
private:
    float Q_angle, Q_bias, R_measure, angle, bias, P[2][2];
};
#endif