#include "depth.h"

#define PWM_MIN   1000
#define PWM_STOP  1500
#define PWM_MAX   2000

#define MBAR_PER_CM 0.980665
#define DEPTH_DIRECTION 1   // ganti -1 jika arah terbalik

DepthController::DepthController(uint8_t pinLeft, uint8_t pinRight)
    : pinM3(pinLeft),
      pinM4(pinRight),
      pressureSensor(ADDRESS_HIGH),
      surfacePressure(0),
      targetDepthCm(85.0),
      deadbandCm(3.0),
      Kp(2.0),
      Ki(0.05),
      Kd(0.8),
      pidIntegral(0),
      lastError(0),
      lastPIDTime(0)
{
}

bool DepthController::begin(TwoWire &wire)
{
    M3.attach(pinM3);
    M4.attach(pinM4);
    M3.writeMicroseconds(PWM_STOP);
    M4.writeMicroseconds(PWM_STOP);
    delay(3000);

    wire.begin();
    wire.setClock(50000);

    pressureSensor.reset();
    delay(20);

    if (pressureSensor.begin(wire) != 0)
        return false;

    pressureSensor.getTemperature(CELSIUS, ADC_512);
    delay(10);
    surfacePressure = pressureSensor.getPressure(ADC_2048);

    return true;
}

void DepthController::setTargetDepth(float depthCm)
{
    targetDepthCm = depthCm;
}

void DepthController::update()
{
    pressureSensor.getTemperature(CELSIUS, ADC_512);
    delay(5);
    float pressure = pressureSensor.getPressure(ADC_2048);
    depthPID(pressure);
}

void DepthController::depthPID(float pressure)
{
    if (pressure < 900 || pressure > 1300) {
        M3.writeMicroseconds(PWM_STOP);
        M4.writeMicroseconds(PWM_STOP);
        return;
    }

    static float filteredPressure = 0;
    filteredPressure = 0.9 * filteredPressure + 0.1 * pressure;

    float depthCm = (filteredPressure - surfacePressure) / MBAR_PER_CM;
    float error = targetDepthCm - depthCm;

    if (abs(error) <= deadbandCm) {
        M3.writeMicroseconds(PWM_STOP);
        M4.writeMicroseconds(PWM_STOP);
        pidIntegral = 0;
        lastError = error;
        return;
    }

    unsigned long now = millis();
    float dt = (now - lastPIDTime) / 1000.0;
    if (dt <= 0 || dt > 1.0) dt = 0.05;
    lastPIDTime = now;

    pidIntegral += error * dt;
    pidIntegral = constrain(pidIntegral, -50, 50);

    float derivative = (error - lastError) / dt;
    lastError = error;

    float pidOutput = DEPTH_DIRECTION *
        (Kp * error + Ki * pidIntegral + Kd * derivative);

    int pwm = PWM_STOP + pidOutput;
    pwm = constrain(pwm, PWM_MIN + 50, PWM_MAX - 50);

    M3.writeMicroseconds(pwm);
    M4.writeMicroseconds(pwm);
}