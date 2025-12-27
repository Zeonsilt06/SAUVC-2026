#include <Arduino.h>
#include "config.h"
#include "../../lib/IMUController/imu_controller.h"
#include "../../lib/ThrusterController/thruster_controller.h"

// ============================================
// ✅ FORWARD DECLARATIONS (TAMBAHKAN INI!)
// ============================================
void killSystem();
void printMissionStatus(unsigned long elapsed);
void checkEmergencyConditions();

// ============================================
// GLOBAL OBJECTS
// ============================================
IMUController imu;
ThrusterController thrusters;

// ============================================
// TIMING VARIABLES
// ============================================
unsigned long missionStartTime = 0;
unsigned long lastMainLoopTime = 0;
unsigned long lastIMUUpdateTime = 0;
unsigned long lastPIDUpdateTime = 0;

bool missionActive = false;
bool systemKilled = false;

// ============================================
// KILL SWITCH
// ============================================
bool lastKillSwitchState = HIGH;

// ============================================
// SETUP
// ============================================
void setup() {
    // Initialize serial
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n");
    Serial.println("========================================");
    Serial.println("  TEENSY 4.1 ROV CONTROLLER");
    Serial.println("  4 THRUSTER + LSM9DS0 IMU + 4 PID");
    Serial.println("  Mission: 20 Second Forward Flight");
    Serial.println("========================================\n");
    
    // Print Teensy info
    Serial.print("[INFO] CPU Speed: ");
    Serial.print(F_CPU / 1000000);
    Serial.println(" MHz");
    
    // Setup kill switch
    pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);
    pinMode(KILL_SWITCH_LED, OUTPUT);
    digitalWrite(KILL_SWITCH_LED, LOW);
    Serial.println("[OK] Kill switch initialized");
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C
    Serial.println("[OK] I2C initialized at 400kHz");
    
    // Initialize IMU
    if (!imu.begin()) {
        Serial.println("[ERROR] IMU initialization failed!");
        while(1) {
            digitalWrite(KILL_SWITCH_LED, !digitalRead(KILL_SWITCH_LED));
            delay(200);
        }
    }
    
    // Calibrate IMU
    Serial.println("\n[INFO] IMU Calibration starting...");
    Serial.println("[INFO] Keep ROV STILL and LEVEL!");
    delay(1000);
    
    if (!imu.calibrate(CALIBRATION_TIME)) {
        Serial.println("[ERROR] IMU calibration failed!");
    }
    
    // Initialize thrusters
    if (!thrusters.begin()) {
        Serial.println("[ERROR] Thruster initialization failed!");
        while(1);
    }
    
    // Arm thrusters
    thrusters.arm();
    
    // Enable only heading PID for this mission
    thrusters.enablePID(true, false, false, false);  // Only heading stabilization
    
    Serial.println("\n[READY] System initialized!");
    Serial.println("[INFO] Starting mission in 3 seconds...\n");
    
    delay(3000);
    
    // Start mission
    missionActive = true;
    missionStartTime = millis();
    lastMainLoopTime = millis();
    lastIMUUpdateTime = millis();
    lastPIDUpdateTime = millis();
    
    Serial.println("[START] MISSION STARTED!\n");
    
    // Set forward movement
    thrusters.moveForward(BASE_FORWARD_SPEED);
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
    unsigned long currentTime = millis();
    
    // ==========================================
    // KILL SWITCH CHECK (highest priority)
    // ==========================================
    bool killSwitchPressed = digitalRead(KILL_SWITCH_PIN) == LOW;
    if (killSwitchPressed && !systemKilled) {
        Serial.println("\n[KILL SWITCH] Hardware override activated!");
        killSystem();  // ✅ SEKARANG SUDAH DIDEKLARASIKAN
        return;
    }
    
    // ==========================================
    // MISSION TIMER CHECK
    // ==========================================
    unsigned long elapsedTime = currentTime - missionStartTime;
    
    if (missionActive && elapsedTime >= MISSION_DURATION) {
        Serial.println("\n[MISSION COMPLETE] 20 seconds elapsed!");
        killSystem();  // ✅ SEKARANG SUDAH DIDEKLARASIKAN
        return;
    }
    
    if (systemKilled) {
        return;  // Do nothing if killed
    }
    
    // ==========================================
    // IMU UPDATE (100Hz)
    // ==========================================
    if (currentTime - lastIMUUpdateTime >= IMU_UPDATE_RATE) {
        lastIMUUpdateTime = currentTime;
        
        if (!imu.update()) {
            Serial.println("[WARNING] IMU update failed!");
        }
        
        // Check IMU health
        if (!imu.isHealthy()) {
            Serial.println("[ERROR] IMU unhealthy!");
            // Could trigger emergency stop here
        }
    }
    
    // ==========================================
    // PID UPDATE (50Hz)
    // ==========================================
    if (currentTime - lastPIDUpdateTime >= PID_UPDATE_RATE) {
        float dt = (currentTime - lastPIDUpdateTime) / 1000.0;
        lastPIDUpdateTime = currentTime;
        
        // Get IMU data
        IMUData imuData = imu.getData();
        
        // Update PID controllers
        // (depth = 0 since we don't have pressure sensor yet)
        thrusters.updatePID(imuData.roll, imuData.pitch, imuData.yaw, 0.0, dt);
    }
    
    // ==========================================
    // MAIN LOOP - STATUS DISPLAY (2Hz)
    // ==========================================
    if (currentTime - lastMainLoopTime >= 500) {
        lastMainLoopTime = currentTime;
        
        // Print status
        printMissionStatus(elapsedTime);  // ✅ SEKARANG SUDAH DIDEKLARASIKAN
    }
    
    // Small delay to prevent watchdog issues
    delay(1);
}

// ============================================
// HELPER FUNCTIONS IMPLEMENTATION
// ============================================

void killSystem() {
    if (systemKilled) return;
    
    systemKilled = true;
    missionActive = false;
    
    // Stop all thrusters
    thrusters.stopAll();
    
    // LED on
    digitalWrite(KILL_SWITCH_LED, HIGH);
    
    Serial.println("\n");
    Serial.println("========================================");
    Serial.println("       SYSTEM KILLED");
    Serial.println("       Mission Complete");
    Serial.println("========================================");
    Serial.println("[INFO] All thrusters stopped");
    Serial.println("[INFO] Reset Teensy to run again\n");
}

void printMissionStatus(unsigned long elapsed) {
    float remaining = (MISSION_DURATION - elapsed) / 1000.0;
    
    // Mission time
    Serial.print("[T+");
    Serial.print(elapsed / 1000.0, 1);
    Serial.print("s] ");
    
    // IMU data
    IMUData data = imu.getData();
    Serial.print("Yaw: ");
    Serial.print(data.yaw, 1);
    Serial.print("° | Roll: ");
    Serial.print(data.roll, 1);
    Serial.print("° | Pitch: ");
    Serial.print(data.pitch, 1);
    Serial.print("° ");
    
    // Remaining time
    Serial.print("| Remaining: ");
    Serial.print(remaining, 1);
    Serial.println("s");
}

void checkEmergencyConditions() {
    IMUData data = imu.getData();
    
    // Check excessive tilt
    if (abs(data.roll) > MAX_TILT_ANGLE || abs(data.pitch) > MAX_TILT_ANGLE) {
        Serial.println("[EMERGENCY] Excessive tilt detected!");
        killSystem();
    }
}

// ============================================
// END OF CODE
// ============================================