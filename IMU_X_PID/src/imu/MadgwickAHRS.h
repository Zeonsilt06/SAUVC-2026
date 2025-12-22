#pragma once
#include "../utils/Quaternion.h"
#include "../utils/Vector3.h"

class MadgwickAHRS {
private:
    Quaternion q = Quaternion(1, 0, 0, 0); // Initial orientation
    float beta;                           // Algorithm gain
    float zeta;                           // Gyro bias gain
    
    // Gyro bias estimation
    Vector3 w_bias = Vector3(0, 0, 0);
    
public:
    MadgwickAHRS();
    
    void begin(float sample_freq, float beta_def = 0.1, float zeta_def = 0.0);
    
    // IMU update (6DOF)
    void updateIMU(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt);
    
    // Full update (9DOF)
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt);

    // Lower-level 9DOF update implementation
    void update9DOF(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float mx, float my, float mz,
                   float dt);
    
    // Get orientation
    Quaternion getQuaternion() const;
    Vector3 getEulerAngles() const;
    Vector3 getGravityVector() const;
    
    // Algorithm parameters
    void setBeta(float beta);
    void setZeta(float zeta);
    
private:
    void gradientDescent(float ax, float ay, float az,
                         float mx, float my, float mz);
    void computeAngularError(const Quaternion& q, 
                             float ax, float ay, float az,
                             float mx, float my, float mz,
                             Vector3& error);
};