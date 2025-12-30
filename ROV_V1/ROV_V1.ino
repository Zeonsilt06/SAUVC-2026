#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "SparkFun_MS5803_I2C.h"

// ==================== MS5803 SENSOR ====================
MS5803 pressureSensor(ADDRESS_HIGH);

// ==================== PWM VALUES ====================
#define PWM_CW     1600  // Clockwise (maju/naik)
#define PWM_STOP   1500  // Stop
#define PWM_CCW    1400  // Counter-clockwise (mundur/turun)

// ==================== DEPTH LIMITS ====================
#define DEPTH_MIN  40.0
#define DEPTH_MAX  130.0

// ==================== THRUSTER PINS ====================
// Sesuai konfigurasi standar:
// T1 = M1 (Horizontal Kiri)
// T2 = M2 (Horizontal Kanan)
// T3 = M3 (Vertikal Depan)
// T4 = M4 (Vertikal Belakang)
#define M1_PIN 2   // T1 - Thruster horizontal kiri
#define M2_PIN 3   // T2 - Thruster horizontal kanan
#define M3_PIN 4   // T3 - Thruster vertikal depan
#define M4_PIN 5   // T4 - Thruster vertikal belakang

// ==================== REBOOT PIN ====================
#define REBOOT_PIN 34

// ==================== PID PARAMETERS ====================
float Kp = 2.0;
float Ki = 0.05;
float Kd = 0.8;

float pidIntegral = 0;
float lastError = 0;

// ==================== TIMING ====================
unsigned long lastPIDTime = 0;
const unsigned long PID_INTERVAL = 50; // 20 Hz

// ==================== STATE MACHINE ====================
enum State {
  DIAM,           // 1. Diam
  MAJU,           // 2. Maju
  MUNDUR,         // 3. Mundur
  KANAN,          // 4. Rotasi Kanan (Yaw+)
  KIRI,           // 5. Rotasi Kiri (Yaw-)
  NAIK,           // 6. Naik
  TURUN,          // 7. Turun
  MAJU_NAIK,      // 8. Maju + Naik
  MAJU_TURUN,     // 9. Maju + Turun
  MUNDUR_NAIK,    // 10. Mundur + Naik
  MUNDUR_TURUN,   // 11. Mundur + Turun
  KANAN_NAIK,     // 12. Rotasi Kanan + Naik
  KANAN_TURUN,    // 13. Rotasi Kanan + Turun
  KIRI_NAIK,      // 14. Rotasi Kiri + Naik
  KIRI_TURUN,     // 15. Rotasi Kiri + Turun
  SELESAI         // Finish state
};

State currentState = DIAM;
unsigned long stateStartTime = 0;
const unsigned long STATE_DURATION = 5000; // 5 detik per state

// ==================== SERVO OBJECTS ====================
Servo M1, M2, M3, M4;

// ==================== REBOOT FUNCTION ====================
void doReboot() {
  SCB_AIRCR = 0x05FA0004;
}

// ==================== SETUP ====================
void setup() {
  delay(1000);
  Serial.begin(115200);
  
  // Setup reboot pin
  pinMode(REBOOT_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Initialize I2C dan pressure sensor
  Wire.begin();
  pressureSensor.reset();
  delay(20);
  pressureSensor.begin();
  
  // Attach servos
  M1.attach(M1_PIN);
  M2.attach(M2_PIN);
  M3.attach(M3_PIN);
  M4.attach(M4_PIN);
  
  // Stop semua thruster
  stopAll();
  
  stateStartTime = millis();
  
  Serial.println("ROV Control System - Tabel 1 Logic");
  Serial.println("State sequence will execute automatically");
  Serial.println("Setup complete.");
  Serial.println();
  
  delay(1000);
}

// ==================== MAIN LOOP ====================
void loop() {
  // Check reboot pin
  int read_pin = digitalRead(REBOOT_PIN);
  if (read_pin == LOW) {
    Serial.println("Reboot triggered...");
    stopAll();
    delay(100);
    doReboot();
  }
  
  // LED blink
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  if (millis() - lastBlink >= 1000) {
    lastBlink = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
  }
  
  float depth = readDepth();
  
  // Tampilkan info state dan depth
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print("State: ");
    Serial.print(getStateName(currentState));
    Serial.print(" | Depth: ");
    Serial.print(depth);
    Serial.print(" mbar | T1:");
    Serial.print(M1.readMicroseconds());
    Serial.print(" T2:");
    Serial.print(M2.readMicroseconds());
    Serial.print(" T3:");
    Serial.print(M3.readMicroseconds());
    Serial.print(" T4:");
    Serial.println(M4.readMicroseconds());
  }
  
  // Check apakah waktunya pindah state
  if (millis() - stateStartTime >= STATE_DURATION) {
    nextState();
  }
  
  // Eksekusi gerakan sesuai state
  executeState();
}

// ==================== EXECUTE STATE ====================
// SEMUA GERAKAN SESUAI TABEL 1
void executeState() {
  switch (currentState) {
    // 1. DIAM
    case DIAM:
      M1.writeMicroseconds(PWM_STOP);  // T1: 0
      M2.writeMicroseconds(PWM_STOP);  // T2: 0
      M3.writeMicroseconds(PWM_STOP);  // T3: 0
      M4.writeMicroseconds(PWM_STOP);  // T4: 0
      break;
      
    // 2. MAJU
    case MAJU:
      M1.writeMicroseconds(PWM_CW);    // T1: 1(CW)
      M2.writeMicroseconds(PWM_CW);    // T2: 1(CW)
      M3.writeMicroseconds(PWM_STOP);  // T3: 0
      M4.writeMicroseconds(PWM_STOP);  // T4: 0
      break;
      
    // 3. MUNDUR
    case MUNDUR:
      M1.writeMicroseconds(PWM_CCW);   // T1: 1(CCW)
      M2.writeMicroseconds(PWM_CCW);   // T2: 1(CCW)
      M3.writeMicroseconds(PWM_STOP);  // T3: 0
      M4.writeMicroseconds(PWM_STOP);  // T4: 0
      break;
      
    // 4. ROTASI KANAN (Yaw+)
    case KANAN:
      M1.writeMicroseconds(PWM_CW);    // T1: 1(CW)
      M2.writeMicroseconds(PWM_CCW);   // T2: 1(CCW)
      M3.writeMicroseconds(PWM_STOP);  // T3: 0
      M4.writeMicroseconds(PWM_STOP);  // T4: 0
      break;
      
    // 5. ROTASI KIRI (Yaw-)
    case KIRI:
      M1.writeMicroseconds(PWM_CCW);   // T1: 1(CCW)
      M2.writeMicroseconds(PWM_CW);    // T2: 1(CW)
      M3.writeMicroseconds(PWM_STOP);  // T3: 0
      M4.writeMicroseconds(PWM_STOP);  // T4: 0
      break;
      
    // 6. NAIK
    case NAIK:
      M1.writeMicroseconds(PWM_STOP);  // T1: 0
      M2.writeMicroseconds(PWM_STOP);  // T2: 0
      M3.writeMicroseconds(PWM_CW);    // T3: 1(CW)
      M4.writeMicroseconds(PWM_CW);    // T4: 1(CW)
      break;
      
    // 7. TURUN
    case TURUN:
      M1.writeMicroseconds(PWM_STOP);  // T1: 0
      M2.writeMicroseconds(PWM_STOP);  // T2: 0
      M3.writeMicroseconds(PWM_CCW);   // T3: 1(CCW)
      M4.writeMicroseconds(PWM_CCW);   // T4: 1(CCW)
      break;
      
    // 8. MAJU + NAIK
    case MAJU_NAIK:
      M1.writeMicroseconds(PWM_CW);    // T1: 1(CW)
      M2.writeMicroseconds(PWM_CW);    // T2: 1(CW)
      M3.writeMicroseconds(PWM_CW);    // T3: 1(CW)
      M4.writeMicroseconds(PWM_CW);    // T4: 1(CW)
      break;
      
    // 9. MAJU + TURUN
    case MAJU_TURUN:
      M1.writeMicroseconds(PWM_CW);    // T1: 1(CW)
      M2.writeMicroseconds(PWM_CW);    // T2: 1(CW)
      M3.writeMicroseconds(PWM_CCW);   // T3: 1(CCW)
      M4.writeMicroseconds(PWM_CCW);   // T4: 1(CCW)
      break;
      
    // 10. MUNDUR + NAIK
    case MUNDUR_NAIK:
      M1.writeMicroseconds(PWM_CCW);   // T1: 1(CCW)
      M2.writeMicroseconds(PWM_CCW);   // T2: 1(CCW)
      M3.writeMicroseconds(PWM_CW);    // T3: 1(CW)
      M4.writeMicroseconds(PWM_CW);    // T4: 1(CW)
      break;
      
    // 11. MUNDUR + TURUN
    case MUNDUR_TURUN:
      M1.writeMicroseconds(PWM_CCW);   // T1: 1(CCW)
      M2.writeMicroseconds(PWM_CCW);   // T2: 1(CCW)
      M3.writeMicroseconds(PWM_CCW);   // T3: 1(CCW)
      M4.writeMicroseconds(PWM_CCW);   // T4: 1(CCW)
      break;
      
    // 12. ROTASI KANAN + NAIK
    case KANAN_NAIK:
      M1.writeMicroseconds(PWM_CW);    // T1: 1(CW)
      M2.writeMicroseconds(PWM_CCW);   // T2: 1(CCW)
      M3.writeMicroseconds(PWM_CW);    // T3: 1(CW)
      M4.writeMicroseconds(PWM_CW);    // T4: 1(CW)
      break;
      
    // 13. ROTASI KANAN + TURUN
    case KANAN_TURUN:
      M1.writeMicroseconds(PWM_CW);    // T1: 1(CW)
      M2.writeMicroseconds(PWM_CCW);   // T2: 1(CCW)
      M3.writeMicroseconds(PWM_CCW);   // T3: 1(CCW)
      M4.writeMicroseconds(PWM_CCW);   // T4: 1(CCW)
      break;
      
    // 14. ROTASI KIRI + NAIK
    case KIRI_NAIK:
      M1.writeMicroseconds(PWM_CCW);   // T1: 1(CCW)
      M2.writeMicroseconds(PWM_CW);    // T2: 1(CW)
      M3.writeMicroseconds(PWM_CW);    // T3: 1(CW)
      M4.writeMicroseconds(PWM_CW);    // T4: 1(CW)
      break;
      
    // 15. ROTASI KIRI + TURUN
    case KIRI_TURUN:
      M1.writeMicroseconds(PWM_CCW);   // T1: 1(CCW)
      M2.writeMicroseconds(PWM_CW);    // T2: 1(CW)
      M3.writeMicroseconds(PWM_CCW);   // T3: 1(CCW)
      M4.writeMicroseconds(PWM_CCW);   // T4: 1(CCW)
      break;
      
    // SELESAI
    case SELESAI:
      stopAll();
      break;
  }
}

// ==================== STATE TRANSITION ====================
void nextState() {
  // Transisi ke state berikutnya sesuai urutan TABEL 1
  switch (currentState) {
    case DIAM:         currentState = MAJU; break;
    case MAJU:         currentState = MUNDUR; break;
    case MUNDUR:       currentState = KANAN; break;
    case KANAN:        currentState = KIRI; break;
    case KIRI:         currentState = NAIK; break;
    case NAIK:         currentState = TURUN; break;
    case TURUN:        currentState = MAJU_NAIK; break;
    case MAJU_NAIK:    currentState = MAJU_TURUN; break;
    case MAJU_TURUN:   currentState = MUNDUR_NAIK; break;
    case MUNDUR_NAIK:  currentState = MUNDUR_TURUN; break;
    case MUNDUR_TURUN: currentState = KANAN_NAIK; break;
    case KANAN_NAIK:   currentState = KANAN_TURUN; break;
    case KANAN_TURUN:  currentState = KIRI_NAIK; break;
    case KIRI_NAIK:    currentState = KIRI_TURUN; break;
    case KIRI_TURUN:   currentState = SELESAI; break;
    case SELESAI:      stopAll(); return; // Tetap di SELESAI
  }
  
  stateStartTime = millis();
  Serial.println();
  Serial.print(">>> Switching to state: ");
  Serial.println(getStateName(currentState));
}

// ==================== GET STATE NAME ====================
const char* getStateName(State state) {
  switch (state) {
    case DIAM:         return "DIAM";
    case MAJU:         return "MAJU";
    case MUNDUR:       return "MUNDUR";
    case KANAN:        return "ROTASI KANAN (Yaw+)";
    case KIRI:         return "ROTASI KIRI (Yaw-)";
    case NAIK:         return "NAIK";
    case TURUN:        return "TURUN";
    case MAJU_NAIK:    return "MAJU + NAIK";
    case MAJU_TURUN:   return "MAJU + TURUN";
    case MUNDUR_NAIK:  return "MUNDUR + NAIK";
    case MUNDUR_TURUN: return "MUNDUR + TURUN";
    case KANAN_NAIK:   return "ROTASI KANAN + NAIK";
    case KANAN_TURUN:  return "ROTASI KANAN + TURUN";
    case KIRI_NAIK:    return "ROTASI KIRI + NAIK";
    case KIRI_TURUN:   return "ROTASI KIRI + TURUN";
    case SELESAI:      return "SELESAI";
    default:           return "UNKNOWN";
  }
}

// ==================== DEPTH READ ====================
float readDepth() {
  float pressure = pressureSensor.getPressure(ADC_2048);
  return pressure;
}

// ==================== STOP ALL ====================
void stopAll() {
  M1.writeMicroseconds(PWM_STOP);
  M2.writeMicroseconds(PWM_STOP);
  M3.writeMicroseconds(PWM_STOP);
  M4.writeMicroseconds(PWM_STOP);
}