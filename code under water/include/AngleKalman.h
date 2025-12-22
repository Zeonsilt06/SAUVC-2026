#ifndef ANGLE_KALMAN_H
#define ANGLE_KALMAN_H

class AngleKalman {
public:
    AngleKalman();

    void setAngle(float angle);
    float getAngle(float newAngle, float newRate, float dt);

private:
    float angle;
    float bias;
    float rate;
    float P[2][2];
};

#endif
