#include "AngleKalman.h"

AngleKalman::AngleKalman() {
    angle = 0;
    bias = 0;
    P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0;
}

void AngleKalman::setAngle(float newAngle) {
    angle = newAngle;
}

float AngleKalman::getAngle(float newAngle, float newRate, float dt) {
    rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + 1);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += 0.003 * dt;

    float S = P[0][0] + 0.03;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    float y = newAngle - angle;
    angle += K0 * y;
    bias  += K1 * y;

    float P00 = P[0][0];
    float P01 = P[0][1];

    P[0][0] -= K0 * P00;
    P[0][1] -= K0 * P01;
    P[1][0] -= K1 * P00;
    P[1][1] -= K1 * P01;

    return angle;
}
