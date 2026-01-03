#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <MadgwickAHRS.h>

HMC5883L mag;
MPU6050 accelgyro(0x68, &Wire1);
Madgwick IMU;

int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
float Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz;

// Offset magnetometer (isi 0 kalau belum kalibrasi)
float mag_offset_x = 0.0;
float mag_offset_y = 0.0;
float mag_offset_z = 0.0;

// Auto-zero
float yawOffset = 0.0;
float pitchOffset = 0.0;
float rollOffset = 0.0;

// Nilai akhir yang sudah dismoothing
float smooth_pitch = 0.0;
float smooth_roll  = 0.0;
float smooth_yaw   = 0.0;

// Parameter khusus untuk ROLL (dibuat lebih halus)
const float alpha_roll  = 0.08;   // Low-pass lebih kuat untuk roll (semakin kecil = semakin halus)
const float maxChange_roll = 2.0; // Maksimal perubahan roll per 10ms (mencegah loncat)

// Parameter normal untuk pitch & yaw
const float alpha_general = 0.15;
const float maxChange_general = 4.0;

void readSensors();

void setup() {
    Wire1.begin();
    delay(500);
    Serial.begin(115200);

    accelgyro.initialize();
    mag.initialize();

    if (accelgyro.testConnection()) Serial.println("MPU6050 OK");
    if (mag.testConnection()) Serial.println("HMC5883L OK");

    IMU.begin(100);

    Serial.println("Stabilisasi & auto-zero... JANGAN GERAKKAN SENSOR 4 detik!");

    for (int i = 0; i < 400; i++) {
        readSensors();
        IMU.update(Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz);
        delay(10);
    }

    yawOffset   = IMU.getYaw();
    pitchOffset = IMU.getPitch();
    rollOffset  = IMU.getRoll();

    smooth_pitch = 0.0;
    smooth_roll  = 0.0;
    smooth_yaw   = 0.0;

    Serial.println("Siap! Roll sekarang sangat stabil & tidak loncat-loncat lagi.");
    Serial.println("P\tR\tY");
}

void loop() {
    readSensors();
    IMU.update(Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz);

    float raw_pitch = IMU.getPitch() - pitchOffset;
    float raw_roll  = IMU.getRoll()  - rollOffset;
    float raw_yaw   = IMU.getYaw()   - yawOffset;

    // Normalisasi yaw & roll ke 0-360
    while (raw_yaw < 0)     raw_yaw += 360.0;
    while (raw_yaw >= 360)  raw_yaw -= 360.0;

    while (raw_roll < 0)    raw_roll += 360.0;
    while (raw_roll >= 360) raw_roll -= 360.0;

    // === RATE LIMITING TERPISAH ===
    // Roll: lebih ketat
    float diff_roll = raw_roll - smooth_roll;
    if (abs(diff_roll) > maxChange_roll) {
        smooth_roll += (diff_roll > 0 ? maxChange_roll : -maxChange_roll);
    } else {
        smooth_roll = raw_roll;
    }

    // Pitch & Yaw: normal
    float diff_pitch = raw_pitch - smooth_pitch;
    if (abs(diff_pitch) > maxChange_general) {
        smooth_pitch += (diff_pitch > 0 ? maxChange_general : -maxChange_general);
    } else {
        smooth_pitch = raw_pitch;
    }

    float diff_yaw = raw_yaw - smooth_yaw;
    if (diff_yaw > 180) diff_yaw -= 360;
    if (diff_yaw < -180) diff_yaw += 360;
    if (abs(diff_yaw) > maxChange_general) {
        smooth_yaw += (diff_yaw > 0 ? maxChange_general : -maxChange_general);
    } else {
        smooth_yaw += diff_yaw;
    }

    // Normalisasi smooth_yaw & smooth_roll
    while (smooth_yaw < 0)    smooth_yaw += 360.0;
    while (smooth_yaw >= 360) smooth_yaw -= 360.0;
    while (smooth_roll < 0)   smooth_roll += 360.0;
    while (smooth_roll >= 360) smooth_roll -= 360.0;

    // === LOW-PASS FILTER TERPISAH ===
    // Roll pakai alpha lebih kecil → lebih halus
    smooth_roll  = alpha_roll * raw_roll + (1.0 - alpha_roll) * smooth_roll;

    // Pitch & Yaw pakai alpha normal
    smooth_pitch = alpha_general * raw_pitch + (1.0 - alpha_general) * smooth_pitch;
    smooth_yaw   = alpha_general * smooth_yaw + (1.0 - alpha_general) * smooth_yaw;

    // === TAMPILKAN ===
    Serial.print(smooth_pitch, 1); Serial.print("\t");
    Serial.print(smooth_roll, 1);  Serial.print("\t");
    Serial.println(smooth_yaw, 1);

    delay(10);
}

void readSensors() {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mag.getHeading(&mx, &my, &mz);

    Ax = ax / 16384.0;
    Ay = ay / 16384.0;
    Az = az / 16384.0;

    Gx = gx / 131.0;
    Gy = gy / 131.0;
    Gz = gz / 131.0;

    Mx = (float)mx - mag_offset_x;
    My = (float)my - mag_offset_y;
    Mz = (float)mz - mag_offset_z;
}