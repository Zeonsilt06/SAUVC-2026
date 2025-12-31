#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <SFE_LSM9DS0.h>
#include <math.h>

// ================== PIN ESC ==================
#define M1_PIN 2
#define M2_PIN 3
#define M3_PIN 4
#define M4_PIN 5

// ================== PWM ==================
#define PWM_STOP 1500
#define PWM_MIN  1400
#define PWM_MAX  1600

// ================== CONTROL ==================
#define KP 5.0
#define DEADZONE 2.0
#define FILTER_ALPHA 0.1

// ================== SAMPLING TIME ==================
#define TS_MS 20          // 20 ms
#define TS    0.02        // 0.02 s

// ================== IMU SETUP ==================
#define LSM9DS0_XM  0x1D
#define LSM9DS0_G   0x6B
LSM9DS0 imu(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

// ================== OBJECT ==================
Servo escM1, escM2, escM3, escM4;

// ================== ORIENTATION ==================
float roll = 0, pitch = 0;
float roll_ref = 0, pitch_ref = 0;

// ================== FILTER ==================
float roll_err_f = 0;
float pitch_err_f = 0;

// ================== TIME ==================
unsigned long lastSample = 0;

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(1000);

  escM1.attach(M1_PIN);
  escM2.attach(M2_PIN);
  escM3.attach(M3_PIN);
  escM4.attach(M4_PIN);

  escM1.writeMicroseconds(PWM_STOP);
  escM2.writeMicroseconds(PWM_STOP);
  escM3.writeMicroseconds(PWM_STOP);
  escM4.writeMicroseconds(PWM_STOP);

  Serial.println("ESC ARMING...");
  delay(3000);

  uint16_t status = imu.begin();
  if (status != 0x49D4) {
    Serial.println("LSM9DS0 tidak terdeteksi!");
    while (1);
  }

  Serial.println("Mengunci posisi referensi...");
  for (int i = 0; i < 50; i++) {
    imu.readAccel();
    float ax = imu.calcAccel(imu.ax);
    float ay = imu.calcAccel(imu.ay);
    float az = imu.calcAccel(imu.az);

    roll_ref  += atan2(ay, sqrt(ax * ax + az * az));
    pitch_ref += atan2(-ax, sqrt(ay * ay + az * az));
    delay(20);
  }

  roll_ref  /= 50.0;
  pitch_ref /= 50.0;

  lastSample = millis();
  Serial.println("=== SYSTEM START ===");
}

// ================== LOOP ==================
void loop() {
  unsigned long now = millis();

  if (now - lastSample >= TS_MS) {
    lastSample = now;

    imu.readAccel();
    imu.readGyro();

    float ax = imu.calcAccel(imu.ax);
    float ay = imu.calcAccel(imu.ay);
    float az = imu.calcAccel(imu.az);

    float gx = imu.calcGyro(imu.gx) * DEG_TO_RAD;
    float gy = imu.calcGyro(imu.gy) * DEG_TO_RAD;

    float accelRoll  = atan2(ay, sqrt(ax * ax + az * az));
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az));

    roll  = 0.98 * (roll  + gx * TS) + 0.02 * accelRoll;
    pitch = 0.98 * (pitch + gy * TS) + 0.02 * accelPitch;

    float roll_err  = degrees(roll  - roll_ref);
    float pitch_err = degrees(pitch - pitch_ref);

    roll_err_f  = roll_err_f  * (1 - FILTER_ALPHA) + roll_err  * FILTER_ALPHA;
    pitch_err_f = pitch_err_f * (1 - FILTER_ALPHA) + pitch_err * FILTER_ALPHA;

    if (abs(roll_err_f) < DEADZONE)   roll_err_f = 0;
    if (abs(pitch_err_f) < DEADZONE) pitch_err_f = 0;

    int roll_pwm  = KP * roll_err_f;
    int pitch_pwm = KP * pitch_err_f;

    int pwmM1 = constrain(PWM_STOP - roll_pwm,  PWM_MIN, PWM_MAX);
    int pwmM2 = constrain(PWM_STOP + roll_pwm,  PWM_MIN, PWM_MAX);
    int pwmM3 = constrain(PWM_STOP - pitch_pwm, PWM_MIN, PWM_MAX);
    int pwmM4 = constrain(PWM_STOP + pitch_pwm, PWM_MIN, PWM_MAX);

    escM1.writeMicroseconds(pwmM1);
    escM2.writeMicroseconds(pwmM2);
    escM3.writeMicroseconds(pwmM3);
    escM4.writeMicroseconds(pwmM4);

    Serial.print("Roll:");
    Serial.print(roll_err_f, 2);
    Serial.print(", Pitch:");
    Serial.print(pitch_err_f, 2);
    Serial.print(", M1:");
    Serial.print(pwmM1);
    Serial.print(", M2:");
    Serial.print(pwmM2);
    Serial.print(", M3:");
    Serial.print(pwmM3);
    Serial.print(", M4:");
    Serial.println(pwmM4);
  }
}
