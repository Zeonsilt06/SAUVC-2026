#include <Arduino.h>
#include "MixedMotorDriver.h"
#include "Sensors.h"
#include "ControlConfig.h"
#include "Control.h"

// --- 1. VARIABEL MISI OTOMATIS ---
enum MissionState { IDLE, DESCENDING, FORWARD, ASCENDING };
MissionState currentState = IDLE;
unsigned long missionTimer = 0;
float currentDepthValue = 0; // Variabel penampung kedalaman saat ini

MixedMotorDriver motors(PIN_H1, PIN_H2, PIN_V1, PIN_V2);

unsigned long previousMillis = 0;
const long interval = 200; 

void setup() {
    Serial.begin(115200);
    delay(1000); // Memberi waktu Serial Monitor untuk siap
    Serial.println("=== MEMULAI INISIALISASI ===");

    // 1. Cek Motor
    if (!motors.begin()) {
        Serial.println("ERROR: Motor/ESC gagal dimulai! Periksa kabel atau power.");
        while (true) delay(100); // Berhenti di sini jika gagal
    }
    Serial.println("OK: Motor siap.");
    motors.stopAll(); 

    // 2. Cek Sensor
    if (!initSensors()) {
        Serial.println("ERROR: Sensor tidak ditemukan! Periksa kabel I2C (SDA/SCL) atau alamat sensor.");
        while (true) delay(100); // Berhenti di sini jika gagal
    }
    Serial.println("OK: Sensor siap.");

    // 3. Inisialisasi Target PID
    pidDepth.setSetpoint(targetDepth);
    pidRoll.setSetpoint(targetRoll);
    pidPitch.setSetpoint(targetPitch);

    Serial.println("=== ROV READY, MEMASUKI LOOP ===");
}

void loop() {
    unsigned long currentMillis = millis();

    // --- BAGIAN UTAMA (SENSOR & KONTROL) ---
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        float dt = interval / 1000.0f; 

        float roll = 0, pitch = 0, yaw = 0, depth = 0, temperature = 0;
        readSensors(roll, pitch, yaw, depth, temperature);
        currentDepthValue = depth; // Update nilai kedalaman global untuk misi

        // Jalankan PID (Membaca manualThrottle, manualTurn yang bisa diubah oleh misi)
        updateControl(roll, pitch, depth, dt, motors);

        // Kirim data ke Dashboard
        Serial.print(roll); Serial.print(","); 
        Serial.print(pitch); Serial.print(","); 
        Serial.print(yaw); Serial.print(","); 
        Serial.println(depth * 100);

        // --- CEK PERINTAH SERIAL DARI DASHBOARD ---
        if (Serial.available()) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            cmd.toLowerCase();

            // Perintah d untuk kedalaman
            if (cmd.startsWith("d")) {
                float nDepth = cmd.substring(1).toFloat();
                setNewTargetDepth(nDepth);
            }
            // Perintah "misi" untuk mulai otomatis
            else if (cmd == "misi") {
                Serial.println("ACK: MISI DIMULAI");
                currentState = DESCENDING; 
            }
            // Perintah "stop" untuk batalkan semua
            else if (cmd == "stop") {
                currentState = IDLE;
                manualThrottle = 0;
                setNewTargetDepth(0.0);
                Serial.println("ACK: EMERGENCY STOP");
            }
        }
    }

    // --- 2. LOGIKA MISI OTOMATIS (DI LUAR INTERVAL SENSOR AGAR RESPONSIF) ---
    switch (currentState) {
        case IDLE:
            // Tidak melakukan apa-apa, menunggu perintah "misi"
            break;

        case DESCENDING:
            setNewTargetDepth(0.8); // Target turun ke 800 centimeter
            // Jika sudah mendekati target (toleransi 10cm)
            if (abs(currentDepthValue - 0.8) < 0.1) { 
                missionTimer = millis();
                currentState = FORWARD;
                Serial.println("LOG: Kedalaman tercapai, Maju...");
            }
            break;

        case FORWARD:
            manualThrottle = 100; // Maju dengan power 100 (dari range 300)
            // Jika sudah maju selama 5 detik
            if (millis() - missionTimer > 5000) { 
                manualThrottle = 0;
                currentState = ASCENDING;
                Serial.println("LOG: Waktu maju habis, Naik...");
            }
            break;

        case ASCENDING:
            setNewTargetDepth(0.0); // Kembali ke permukaan
            if (currentDepthValue < 0.2) {
                currentState = IDLE;
                Serial.println("MISI SELESAI");
            }
            break;
    }
}