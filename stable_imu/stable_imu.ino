#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

// Inisialisasi LSM9DS0
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

// Variabel orientasi
float roll = 0;
float pitch = 0;
float yaw = 0;

// Kalibrasi offset
float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;
float accelOffsetX = 0;
float accelOffsetY = 0;

// Filter settings
float alpha = 0.98;  // Complementary filter
unsigned long lastTime = 0;

// Dead zone yang lebih ketat
const float GYRO_DEAD_ZONE = 0.015;      // rad/s
const float ACCEL_DEAD_ZONE = 0.05;      // m/s²
const float ANGLE_DEAD_ZONE = 0.5;       // degrees

// Low-pass filter untuk accelerometer
float accelFilterX = 0;
float accelFilterY = 0;
float accelFilterZ = 0;
const float ACCEL_FILTER_ALPHA = 0.3;

// Deteksi gerakan
float lastRoll = 0;
float lastPitch = 0;
float lastYaw = 0;
bool isMoving = false;

// Setup sensor
void setupSensor()
{
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

// Kalibrasi lengkap
void calibrateSensors() {
  Serial.println("=================================");
  Serial.println("KALIBRASI SENSOR");
  Serial.println("Letakkan sensor di permukaan DATAR");
  Serial.println("JANGAN GERAKKAN selama 5 detik!");
  Serial.println("=================================");
  
  delay(3000);
  
  float sumGX = 0, sumGY = 0, sumGZ = 0;
  float sumAX = 0, sumAY = 0;
  int samples = 200;
  
  Serial.println("Mengambil sample...");
  
  for(int i = 0; i < samples; i++) {
    lsm.read();
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);
    
    sumGX += gyro.gyro.x;
    sumGY += gyro.gyro.y;
    sumGZ += gyro.gyro.z;
    sumAX += accel.acceleration.x;
    sumAY += accel.acceleration.y;
    
    if (i % 20 == 0) Serial.print(".");
    delay(10);
  }
  
  Serial.println();
  
  gyroOffsetX = sumGX / samples;
  gyroOffsetY = sumGY / samples;
  gyroOffsetZ = sumGZ / samples;
  accelOffsetX = sumAX / samples;
  accelOffsetY = sumAY / samples;
  
  Serial.println("Kalibrasi Selesai!");
  Serial.println("-------------------");
  Serial.print("Gyro Offset X: "); Serial.println(gyroOffsetX, 5);
  Serial.print("Gyro Offset Y: "); Serial.println(gyroOffsetY, 5);
  Serial.print("Gyro Offset Z: "); Serial.println(gyroOffsetZ, 5);
  Serial.print("Accel Offset X: "); Serial.println(accelOffsetX, 3);
  Serial.print("Accel Offset Y: "); Serial.println(accelOffsetY, 3);
  Serial.println("=================================");
  Serial.println();
  
  delay(2000);
}

void setup() 
{
  Serial.begin(115200);
  
  while (!Serial && millis() < 3000) {
    delay(1);
  }
  
  Serial.println("LSM9DS0 9DOF - STABLE VERSION");
  Serial.println("");
  
  if(!lsm.begin())
  {
    Serial.println("ERROR: Sensor tidak ditemukan!");
    Serial.println("Periksa koneksi I2C:");
    Serial.println("  VIN  -> 3.3V");
    Serial.println("  GND  -> GND");
    Serial.println("  SCL  -> Pin 19");
    Serial.println("  SDA  -> Pin 18");
    while(1) {
      delay(1000);
    }
  }
  
  Serial.println("✓ Sensor LSM9DS0 ditemukan!");
  setupSensor();
  
  delay(1000);
  calibrateSensors();
  
  lastTime = millis();
  
  Serial.println("Mulai streaming data...");
  Serial.println();
}

void loop() 
{
  lsm.read();
  
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  
  // Koreksi offset dan low-pass filter untuk accelerometer
  float ax = accel.acceleration.x - accelOffsetX;
  float ay = accel.acceleration.y - accelOffsetY;
  float az = accel.acceleration.z;
  
  // Low-pass filter untuk accelerometer (mengurangi noise)
  accelFilterX = accelFilterX * (1.0 - ACCEL_FILTER_ALPHA) + ax * ACCEL_FILTER_ALPHA;
  accelFilterY = accelFilterY * (1.0 - ACCEL_FILTER_ALPHA) + ay * ACCEL_FILTER_ALPHA;
  accelFilterZ = accelFilterZ * (1.0 - ACCEL_FILTER_ALPHA) + az * ACCEL_FILTER_ALPHA;
  
  // Dead zone untuk accelerometer
  if (abs(accelFilterX) < ACCEL_DEAD_ZONE) accelFilterX = 0;
  if (abs(accelFilterY) < ACCEL_DEAD_ZONE) accelFilterY = 0;
  
  // Koreksi offset gyroscope
  float gx = gyro.gyro.x - gyroOffsetX;
  float gy = gyro.gyro.y - gyroOffsetY;
  float gz = gyro.gyro.z - gyroOffsetZ;
  
  // Dead zone untuk gyroscope
  if (abs(gx) < GYRO_DEAD_ZONE) gx = 0;
  if (abs(gy) < GYRO_DEAD_ZONE) gy = 0;
  if (abs(gz) < GYRO_DEAD_ZONE) gz = 0;
  
  // Deteksi gerakan
  isMoving = (abs(gx) > 0 || abs(gy) > 0 || abs(gz) > 0);
  
  // Hitung orientasi
  calculateOrientation(accelFilterX, accelFilterY, accelFilterZ, gx, gy, gz);
  
  // Kirim data untuk SerialPlot
  Serial.print(degrees(roll));
  Serial.print(",");
  Serial.print(degrees(yaw));
  Serial.print(",");
  Serial.println(degrees(pitch));
  
  delay(50);
}

void calculateOrientation(float ax, float ay, float az, float gx, float gy, float gz) {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // Validasi dt
  if (dt <= 0 || dt > 1.0) {
    dt = 0.05;
    return; // Skip frame ini
  }
  
  // Hitung sudut dari accelerometer
  float accelRoll = atan2(ay, az);
  float accelPitch = atan2(-ax, sqrt(ay*ay + az*az));
  
  // Integrasi gyroscope HANYA saat bergerak
  if (isMoving) {
    roll += gx * dt;
    pitch += gy * dt;
    yaw += gz * dt;
  } else {
    // Saat diam, decay perlahan ke 0 untuk yaw
    yaw *= 0.99;
  }
  
  // Complementary filter
  roll = alpha * roll + (1.0 - alpha) * accelRoll;
  pitch = alpha * pitch + (1.0 - alpha) * accelPitch;
  
  // Apply dead zone untuk sudut kecil
  float rollDeg = degrees(roll);
  float pitchDeg = degrees(pitch);
  float yawDeg = degrees(yaw);
  
  if (abs(rollDeg) < ANGLE_DEAD_ZONE) roll = 0;
  if (abs(pitchDeg) < ANGLE_DEAD_ZONE) pitch = 0;
  if (abs(yawDeg) < ANGLE_DEAD_ZONE) yaw = 0;
  
  // Normalize angles
  roll = normalizeAngle(roll);
  pitch = normalizeAngle(pitch);
  yaw = normalizeAngle(yaw);
  
  // Anti-drift: reset jika perubahan sangat kecil dalam waktu lama
  if (!isMoving) {
    float deltaRoll = abs(roll - lastRoll);
    float deltaPitch = abs(pitch - lastPitch);
    
    if (deltaRoll < 0.001 && abs(roll) < 0.01) roll = 0;
    if (deltaPitch < 0.001 && abs(pitch) < 0.01) pitch = 0;
  }
  
  lastRoll = roll;
  lastPitch = pitch;
  lastYaw = yaw;
}

float normalizeAngle(float angle) {
  while (angle > PI) angle -= TWO_PI;
  while (angle < -PI) angle += TWO_PI;
  return angle;
}