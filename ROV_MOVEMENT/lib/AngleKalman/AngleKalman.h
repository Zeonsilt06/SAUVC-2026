#ifndef ANGLEKALMAN_H
#define ANGLEKALMAN_H

class AngleKalman {

    public:
        AngleKalman();
        float update(float newRate, float newAngle, float dt);
        void setQ(float qa, float qb, float r);
        void setAngle(float a);
        float getAngle();

    private:
        float Q_angle, Q_bias, R_measure;
        float angle, bias;
        float P[2][2];
    };


#endif