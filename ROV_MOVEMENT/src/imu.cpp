#include "IMU.h"

IMU::IMU() : accelgyro(0x68, &Wire1) {}

bool IMU::begin() {
    Wire1.begin();
    delay(100);

    accelgyro.initialize();
    mag.initialize();

    if (!accelgyro.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        return false;
    }
    if (!mag.testConnection()) {
        Serial.println("HMC5883L connection failed!");
        return false;
    }

    filter.begin(100);  // 100 Hz
    Serial.println("IMU initialized successfully.");
    return true;
}

void IMU::update() {
    readSensors();
    filter.update(Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz);

    float raw_pitch = filter.getPitch();
    float raw_roll  = filter.getRoll();
    float raw_yaw   = filter.getYaw();

    // Apply zero offset
    raw_pitch -= pitchOffset;
    raw_roll  -= rollOffset;
    raw_yaw   -= yawOffset;

    // Normalisasi yaw & roll ke 0-360
    while (raw_yaw < 0) raw_yaw += 360.0;
    while (raw_yaw >= 360) raw_yaw -= 360.0;
    while (raw_roll < 0) raw_roll += 360.0;
    while (raw_roll >= 360) raw_roll -= 360.0;

    // Rate limiting + low-pass filter
    float diff_roll = raw_roll - smooth_roll;
    if (abs(diff_roll) > maxChange_roll) {
        smooth_roll += (diff_roll > 0 ? maxChange_roll : -maxChange_roll);
    } else {
        smooth_roll = raw_roll;
    }
    smooth_roll = alpha_roll * raw_roll + (1.0 - alpha_roll) * smooth_roll;

    smooth_pitch = alpha_general * raw_pitch + (1.0 - alpha_general) * smooth_pitch;

    float diff_yaw = raw_yaw - smooth_yaw;
    if (diff_yaw > 180) diff_yaw -= 360;
    if (diff_yaw < -180) diff_yaw += 360;
    if (abs(diff_yaw) > maxChange_general) {
        smooth_yaw += (diff_yaw > 0 ? maxChange_general : -maxChange_general);
    } else {
        smooth_yaw += diff_yaw;
    }
    smooth_yaw = alpha_general * smooth_yaw + (1.0 - alpha_general) * smooth_yaw;
}

float IMU::getPitch() { return smooth_pitch; }
float IMU::getRoll()  { return smooth_roll; }
float IMU::getYaw()   { return smooth_yaw; }

void IMU::calibrateZero() {
    Serial.println("Calibrating zero... Keep sensor still for 4 seconds!");

    for (int i = 0; i < 400; i++) {
        update();
        delay(10);
    }

    pitchOffset += filter.getPitch();
    rollOffset  += filter.getRoll();
    yawOffset   += filter.getYaw();

    smooth_pitch = 0.0;
    smooth_roll  = 0.0;
    smooth_yaw   = 0.0;

    Serial.println("Zero calibration complete! All axes set to 0.");
}

void IMU::readSensors() {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mag.getHeading(&mx, &my, &mz);

    Ax = ax / 16384.0;
    Ay = ay / 16384.0;
    Az = az / 16384.0;

    Gx = gx / 131.0;
    Gy = gy / 131.0;
    Gz = gz / 131.0;

    Mx = mx;
    My = my;
    Mz = mz;
}