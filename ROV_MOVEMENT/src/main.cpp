#include <Arduino.h>
#include <PID_v1.h>
#include "ThrusterControl.h"
#include "IMU.h"
#include "Depth.h"

IMU imu;
DepthController depth;

double dIn, dOut, dSet; 
double rIn, rOut, rSet; 
double yIn, yOut, ySet; 

// PID Setup
PID pidDepth(&dIn, &dOut, &dSet, 4.0, 0.1, 1.0, DIRECT);
PID pidRoll(&rIn, &rOut, &rSet, 15.0, 0.5, 5.0, DIRECT);
PID pidYaw(&yIn, &yOut, &ySet, 10.0, 0.1, 1.0, DIRECT);

void setup() {
    Serial.begin(115200);
    initMotor();
    imu.begin();
    depth.begin(Wire1);

    delay(2000); // Tunggu sensor stabil
    imu.update();
    depth.update();

    // --- PENGATURAN TARGET ---
    dSet = 0.8;           // TARGET KEDALAMAN: 80 cm (0.8 meter)
    ySet = (double)imu.getYaw(); // Kunci arah hadap saat ini
    rSet = 0.0;           // Jaga robot tetap datar (roll 0)

    pidDepth.SetMode(AUTOMATIC);
    pidRoll.SetMode(AUTOMATIC);
    pidYaw.SetMode(AUTOMATIC);
    
    pidDepth.SetOutputLimits(-300, 300);
    pidRoll.SetOutputLimits(-150, 150);
    pidYaw.SetOutputLimits(-200, 200);
}

void loop() {
    static unsigned long lastTime = 0;
    if (millis() - lastTime < 20) return;
    lastTime = millis();

    imu.update();
    depth.update();

    // 1. Input Sensor
    dIn = (double)depth.getDepth();
    rIn = (double)imu.getRoll();
    
    // Heading Wrap Around Logic (Agar tidak pusing saat melewati 360/0 derajat)
    double currentYaw = (double)imu.getYaw();
    double yawError = ySet - currentYaw;
    if (yawError > 180) yawError -= 360;
    else if (yawError < -180) yawError += 360;
    yIn = -yawError;

    // 2. Hitung PID
    pidDepth.Compute();
    pidRoll.Compute();
    
    // Trick untuk PID Yaw agar bekerja dengan yawError
    double oldYSet = ySet; 
    ySet = 0;
    pidYaw.Compute();
    ySet = oldYSet;

    // 3. LOGIKA MISI OTOMATIS (Sesuai Urutan Waktu)
    unsigned long runTime = millis();

    if (runTime < 2000) {
        // Detik 0-2: Persiapan di permukaan/menyelam awal
        setForwardPower(0);
    } 
    else if (runTime < 7000) {         
        // Detik 2-7: MAJU sambil menjaga kedalaman 80cm
        setForwardPower(300); 
    } 
    else if (runTime < 10000) {      
        // Detik 7-10: DIAM & BELOK KANAN (Ubah arah ke 270 derajat)
        setForwardPower(0);
        ySet = 270.0; 
    }
    else if (runTime < 15000) {     
        // Detik 10-15: MUNDUR
        setForwardPower(-300);
    }
    else {
        // Detik > 15: SELESAI / BERHENTI
        setForwardPower(0);
    }

    // 4. Update Motor dengan hasil gabungan PID
    // dOut akan bekerja otomatis menjaga kedalaman 80cm selama misi berjalan
    updateMotors((int)yOut, (int)rOut, (int)dOut);

    // Telemetri ke Serial Monitor
    if (millis() % 500 < 20) {
        Serial.print("Status: "); 
        if(runTime < 7000) Serial.print("MAJU");
        else if(runTime < 10000) Serial.print("BELOK");
        else if(runTime < 15000) Serial.print("MUNDUR");
        else Serial.print("STOP");
        
        Serial.print(" | Depth In:"); Serial.print(dIn);
        Serial.print(" | Target:"); Serial.print(dSet);
        Serial.print(" | Yaw:"); Serial.println(currentYaw);
    }
}