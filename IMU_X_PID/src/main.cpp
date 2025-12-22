#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "imu/LSM9DS0.h"
#include "imu/MadgwickAHRS.h"
#include "control/PIDController.h"
#include "control/ThrusterMixer.h"
#include "vehicle/QuadcopterDynamics.h"
#include "utils/Vector3.h"
#include "utils/Quaternion.h"

// Global objects
LSM9DS0 imu;
MadgwickAHRS filter;
QuadcopterDynamics vehicle;
ThrusterMixer mixer;

// PID Controllers
PIDController pid_roll("Roll");
PIDController pid_pitch("Pitch");
PIDController pid_yaw("Yaw");
PIDController pid_thrust("Thrust");

// Control variables
float dt = 0.01;  // 100Hz control loop
unsigned long last_time = 0;

// Setpoints (can come from remote control)
struct Setpoints {
    float roll = 0.0;     // radians
    float pitch = 0.0;    // radians
    float yaw = 0.0;      // radians
    float thrust = 0.0;   // 0-1 normalized
    float altitude = 0.0; // meters
} setpoints;

// IMU data structure
struct IMUData {
    Vector3 accel;      // m/s²
    Vector3 gyro;       // rad/s
    Vector3 mag;        // uT
    Quaternion quat;    // orientation
    Vector3 euler;      // roll, pitch, yaw (rad)
    float dt;           // sample time
} imu_data;

// Thruster outputs (normalized 0-1)
float thruster_cmds[4] = {0, 0, 0, 0};

// ESC pulse widths (in microseconds)
uint16_t esc_pulses[4] = {1500, 1500, 1500, 1500};

// Forward declarations - ADD THESE LINES
void updateESCs();
void logTelemetry();
void checkSerialCommands();

// Setup function
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== Quadcopter Control System ===");
    
    // Initialize I2C at 400kHz
    Wire.begin();
    Wire.setClock(400000);
    
    // Initialize IMU
    if (!imu.begin()) {
        Serial.println("Failed to initialize LSM9DS0!");
        while(1);
    }
    Serial.println("LSM9DS0 initialized successfully");
    
    // Configure IMU settings (use Adafruit library constants if desired)
    // Example: imu.setAccelRange(LSM9DS0_ACCEL_MG_LSB_4G);
    // If you want non-default ranges, replace the commented constants with
    // the appropriate ones from the installed Adafruit LSM9DS0 library.
    
    // Initialize Madgwick filter
    filter.begin(100.0); // 100Hz update rate
    
    // Initialize PID controllers with tuned parameters
    pid_roll.setParameters(3.5, 0.05, 0.8, 100.0); // Kp, Ki, Kd, max_output
    pid_pitch.setParameters(3.5, 0.05, 0.8, 100.0);
    pid_yaw.setParameters(2.0, 0.02, 0.5, 100.0);
    pid_thrust.setParameters(1.0, 0.1, 0.2, 100.0);
    
    // Initialize vehicle dynamics
    vehicle.setMass(1.2);      // 1.2kg
    vehicle.setArmLength(0.15); // 15cm arm length
    
    // Initialize thruster mixer with X configuration
    mixer.setConfiguration(MixerConfig::QUAD_X);
    
    // Calibrate IMU
    Serial.println("Calibrating IMU... Do not move the vehicle!");
    delay(2000);
    imu.calibrate();
    Serial.println("Calibration complete!");
    
    // Arm ESCs (send neutral pulse)
    for(int i = 0; i < 4; i++) {
        esc_pulses[i] = 1500; // Neutral position
    }
    
    last_time = micros();
}

// Main control loop
void loop() {
    unsigned long current_time = micros();
    float elapsed = (current_time - last_time) / 1000000.0;
    
    if(elapsed >= dt) {
        last_time = current_time;
        
        // 1. Read IMU data
        if(imu.read()) {
            imu_data.accel = imu.getAccel();
            imu_data.gyro = imu.getGyro();
            imu_data.mag = imu.getMag();
            imu_data.dt = elapsed;
            
            // 2. Update Madgwick filter
            filter.update(imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z,
                         imu_data.accel.x, imu_data.accel.y, imu_data.accel.z,
                         imu_data.mag.x, imu_data.mag.y, imu_data.mag.z,
                         elapsed);
            
            // 3. Get orientation
            imu_data.quat = filter.getQuaternion();
            imu_data.euler = imu_data.quat.toEuler();
            
            // 4. Run PID controllers
            float roll_out = pid_roll.update(setpoints.roll, imu_data.euler.x, elapsed);
            float pitch_out = pid_pitch.update(setpoints.pitch, imu_data.euler.y, elapsed);
            float yaw_out = pid_yaw.update(setpoints.yaw, imu_data.euler.z, elapsed);
            float thrust_out = pid_thrust.update(setpoints.altitude, 0, elapsed); // Simplified
            
            // 5. Apply vehicle dynamics and limits
            Vector3 torques(roll_out, pitch_out, yaw_out);
            torques = vehicle.applyLimits(torques);
            
            // 6. Mix to get thruster commands
            mixer.mix(thruster_cmds, thrust_out, torques);
            
            // 7. Convert to ESC pulses (1000-2000μs)
            for(int i = 0; i < 4; i++) {
                esc_pulses[i] = 1000 + thruster_cmds[i] * 1000;
                esc_pulses[i] = constrain(esc_pulses[i], 1000, 2000);
            }
            
            // 8. Update ESCs (would be hardware-specific)
            updateESCs();
            
            // 9. Log data (optional)
            if(current_time % 1000000 < 10000) { // Every ~1 second
                logTelemetry();
            }
        }
    }
    
    // Check for serial commands
    checkSerialCommands();
}

// Function definitions
void updateESCs() {
    // This is platform-specific
    // For Teensy 4.1, you'd use PWM library or Servo library
    // Example with Servo library:
    // for(int i = 0; i < 4; i++) {
    //     esc_servos[i].writeMicroseconds(esc_pulses[i]);
    // }
}

void logTelemetry() {
    Serial.print("RPY: ");
    Serial.print(imu_data.euler.x * 57.2958); // Rad to deg
    Serial.print(", ");
    Serial.print(imu_data.euler.y * 57.2958);
    Serial.print(", ");
    Serial.print(imu_data.euler.z * 57.2958);
    Serial.print(" | Thrusters: ");
    for(int i = 0; i < 4; i++) {
        Serial.print(thruster_cmds[i]);
        Serial.print(" ");
    }
    Serial.println();
}

void checkSerialCommands() {
    if(Serial.available()) {
        char cmd = Serial.read();
        switch(cmd) {
            case 'a': // Arm
                setpoints.thrust = 0.1;
                break;
            case 'd': // Disarm
                setpoints.thrust = 0.0;
                break;
            case 'w': // Increase thrust
                setpoints.thrust += 0.05;
                break;
            case 's': // Decrease thrust
                setpoints.thrust -= 0.05;
                break;
            case 'q': // Quaternion data
                Serial.print("Quat: ");
                Serial.print(imu_data.quat.w, 4);
                Serial.print(", ");
                Serial.print(imu_data.quat.x, 4);
                Serial.print(", ");
                Serial.print(imu_data.quat.y, 4);
                Serial.print(", ");
                Serial.println(imu_data.quat.z, 4);
                break;
        }
        setpoints.thrust = constrain(setpoints.thrust, 0.0, 1.0);
    }
}