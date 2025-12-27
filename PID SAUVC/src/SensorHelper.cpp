#include "SensorHelper.h"

float roll, pitch, yaw;

bool initSensors(MS5803 &press) {
    return press.begin();
}

void updateSensors(Adafruit_LSM9DS0 &lsm, MS5803 &press, float surfaceP, float &dpt) {
    sensors_event_t a, m, g, t;
    lsm.getEvent(&a, &m, &g, &t);
    // Sederhana: Roll/Pitch dari Accel
    roll = atan2(a.acceleration.y, a.acceleration.z) * 180/PI;
    pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180/PI;
    
    float currentP = press.getPressure(ADC_4096);
    dpt = (currentP - surfaceP) / 101.3; 
    if(dpt < 0) dpt = 0;
}