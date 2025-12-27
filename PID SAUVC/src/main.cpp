#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "SensorHelper.h"
#include "MixedMotorDriver.h"
#include "Control.h"

// 1. PERBAIKAN: Masukkan PIN ke dalam constructor
MixedMotorDriver rov(PIN_H1, PIN_H2, PIN_V1, PIN_V2);

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();
MS5803 press(ADDRESS_HIGH);

float surfP, dpt;
unsigned long tStart = 0;

enum Misi { IDLE, M1, B, M2, MD, N, STOP };
Misi state = IDLE;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    if (!lsm.begin()) {
        Serial.println("LSM9DS0 tidak deteksi!");
    }
    
    if (!initSensors(press)) {
        Serial.println("MS5803 tidak deteksi!");
    }

    surfP = press.getPressure(ADC_4096);
    rov.begin();
    Serial.println("ROV READY. Ketik 'misi' untuk mulai.");
}

void loop() {
    static unsigned long pM = 0;
    unsigned long cM = millis();
    float dt = (cM - pM) / 1000.0f;
    if (dt <= 0) dt = 0.02; // Mencegah pembagian nol
    pM = cM;

    updateSensors(lsm, press, surfP, dpt);

    unsigned long dur = cM - tStart;
    
    // 2. LOGIKA MISI (Target No. 3)
    switch(state) {
        case IDLE:
            manThr = 0; manTurn = 0; manVert = 0;
            break;
        case M1: // Maju 5 detik
            manThr = 20; manTurn = 0; 
            if(dur > 5000) { state = B; tStart = cM; } 
            break;
        case B:  // Belok 5 detik
            manThr = 0; manTurn = 15; 
            if(dur > 5000) { state = M2; tStart = cM; } 
            break;
        case M2: // Maju lagi 5 detik
            manThr = 20; manTurn = 0; 
            if(dur > 5000) { state = MD; tStart = cM; } 
            break;
        case MD: // Mundur 5 detik
            manThr = -20; manTurn = 0; 
            if(dur > 5000) { state = N; tStart = cM; } 
            break;
        case N:  // Naik 5 detik
            manThr = 0; manVert = 30; 
            if(dur > 5000) { state = STOP; } 
            break;
        case STOP:
            rov.stopAll(); 
            state = IDLE; 
            Serial.println("Selesai.");
            break;
    }

    // 3. Panggil fungsi kontrol PID
    updatePID1_Vertical(dpt, roll, dt, rov);
    updatePID2_Horizontal(0, dt, rov); // Yaw diisi 0 jika belum ada kompas aktif

    // Output untuk Telemetry Viewer (Laptop Celeron)
    if(cM % 100 == 0) { 
        Serial.print(dpt*100); Serial.print(",");
        Serial.print(roll); Serial.print(",");
        Serial.println(manThr);
    }

    if(Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if(input == "misi") {
            state = M1; 
            tStart = millis();
            Serial.println("Misi Dimulai...");
        }
    }
}