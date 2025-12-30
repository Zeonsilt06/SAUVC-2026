#include <Arduino.h>
#include <Servo.h>

// ================= PWM (PELAAAAAN) =================
#define PWM_STOP   1500
#define PWM_CW     1480
#define PWM_CCW    1520

// ================= PIN =================
#define M1_PIN 2
#define M2_PIN 3
#define M3_PIN 4
#define M4_PIN 5

Servo M1, M2, M3, M4;

// ================= STATE =================
enum State {
  DIAM,
  MAJU,
  MUNDUR,
  ROTASI_KANAN,
  ROTASI_KIRI,
  NAIK,
  TURUN,
  MAJU_NAIK,
  MAJU_TURUN,
  MUNDUR_NAIK,
  MUNDUR_TURUN,
  ROTASI_KANAN_NAIK,
  ROTASI_KANAN_TURUN,
  ROTASI_KIRI_NAIK,
  ROTASI_KIRI_TURUN,
  JUMLAH_STATE   // <<< penanda jumlah state
};

State currentState = DIAM;
bool emergencyStop = false;

unsigned long stateStart = 0;
const unsigned long STATE_TIME = 800;   // <<< lebih responsif
unsigned long lastPrint = 0;

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  M1.attach(M1_PIN);
  M2.attach(M2_PIN);
  M3.attach(M3_PIN);
  M4.attach(M4_PIN);

  stopAll();
  stateStart = millis();

  Serial.println("=== ROV TEST LOOPING ===");
  Serial.println("Tekan 'S' untuk STOP DARURAT");
}

// ================= LOOP =================
void loop() {

  // ===== EMERGENCY STOP =====
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S') {
      emergencyStop = true;
      stopAll();
      Serial.println("!!! EMERGENCY STOP AKTIF !!!");
    }
  }
  if (emergencyStop) return;

  // ===== GANTI STATE =====
  if (millis() - stateStart >= STATE_TIME) {
    nextState();
    stateStart = millis();
  }

  jalankanState();
}

// ================= STATE LOGIC =================
void jalankanState() {

  int pwm1 = PWM_STOP, pwm2 = PWM_STOP;
  int pwm3 = PWM_STOP, pwm4 = PWM_STOP;

  switch (currentState) {

    case MAJU:
      pwm1 = PWM_CW; pwm2 = PWM_CW; break;

    case MUNDUR:
      pwm1 = PWM_CCW; pwm2 = PWM_CCW; break;

    case ROTASI_KANAN:
      pwm1 = PWM_CW; pwm2 = PWM_CCW; break;

    case ROTASI_KIRI:
      pwm1 = PWM_CCW; pwm2 = PWM_CW; break;

    case NAIK:
      pwm3 = PWM_CW; pwm4 = PWM_CW; break;

    case TURUN:
      pwm3 = PWM_CCW; pwm4 = PWM_CCW; break;

    case MAJU_NAIK:
      pwm1 = PWM_CW; pwm2 = PWM_CW;
      pwm3 = PWM_CW; pwm4 = PWM_CW; break;

    case MAJU_TURUN:
      pwm1 = PWM_CW; pwm2 = PWM_CW;
      pwm3 = PWM_CCW; pwm4 = PWM_CCW; break;

    case MUNDUR_NAIK:
      pwm1 = PWM_CCW; pwm2 = PWM_CCW;
      pwm3 = PWM_CW; pwm4 = PWM_CW; break;

    case MUNDUR_TURUN:
      pwm1 = PWM_CCW; pwm2 = PWM_CCW;
      pwm3 = PWM_CCW; pwm4 = PWM_CCW; break;

    case ROTASI_KANAN_NAIK:
      pwm1 = PWM_CW; pwm2 = PWM_CCW;
      pwm3 = PWM_CW; pwm4 = PWM_CW; break;

    case ROTASI_KANAN_TURUN:
      pwm1 = PWM_CW; pwm2 = PWM_CCW;
      pwm3 = PWM_CCW; pwm4 = PWM_CCW; break;

    case ROTASI_KIRI_NAIK:
      pwm1 = PWM_CCW; pwm2 = PWM_CW;
      pwm3 = PWM_CW; pwm4 = PWM_CW; break;

    case ROTASI_KIRI_TURUN:
      pwm1 = PWM_CCW; pwm2 = PWM_CW;
      pwm3 = PWM_CCW; pwm4 = PWM_CCW; break;

    default:
      break;
  }

  M1.writeMicroseconds(pwm1);
  M2.writeMicroseconds(pwm2);
  M3.writeMicroseconds(pwm3);
  M4.writeMicroseconds(pwm4);

  // ===== SERIAL TANPA DELAY =====
  if (millis() - lastPrint >= 150) {
    lastPrint = millis();
    Serial.print("KONDISI: ");
    Serial.print(getStateName(currentState));
    Serial.print(" | M1:"); Serial.print(pwm1);
    Serial.print(" M2:"); Serial.print(pwm2);
    Serial.print(" M3:"); Serial.print(pwm3);
    Serial.print(" M4:"); Serial.println(pwm4);
  }
}

// ================= NEXT STATE (LOOPING) =================
void nextState() {
  currentState = (State)((currentState + 1) % JUMLAH_STATE);
}

// ================= NAMA STATE =================
const char* getStateName(State s) {
  switch (s) {
    case DIAM: return "DIAM";
    case MAJU: return "MAJU";
    case MUNDUR: return "MUNDUR";
    case ROTASI_KANAN: return "ROTASI KANAN";
    case ROTASI_KIRI: return "ROTASI KIRI";
    case NAIK: return "NAIK";
    case TURUN: return "TURUN";
    case MAJU_NAIK: return "MAJU+NAIK";
    case MAJU_TURUN: return "MAJU+TURUN";
    case MUNDUR_NAIK: return "MUNDUR+NAIK";
    case MUNDUR_TURUN: return "MUNDUR+TURUN";
    case ROTASI_KANAN_NAIK: return "KANAN+NAIK";
    case ROTASI_KANAN_TURUN: return "KANAN+TURUN";
    case ROTASI_KIRI_NAIK: return "KIRI+NAIK";
    case ROTASI_KIRI_TURUN: return "KIRI+TURUN";
    default: return "UNKNOWN";
  }
}

// ================= UTIL =================
void stopAll() {
  M1.writeMicroseconds(PWM_STOP);
  M2.writeMicroseconds(PWM_STOP);
  M3.writeMicroseconds(PWM_STOP);
  M4.writeMicroseconds(PWM_STOP);
}