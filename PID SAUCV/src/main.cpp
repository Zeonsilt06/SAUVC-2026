#include <Arduino.h>
#include <Encoder.h>
#include "PIDController.h"
#include "MotorController.h"
#include "RobotController.h"

// Pin definitions - DRIVE MOTORS (4 motor)
#define DRIVE_FL_PWM 2    // Front Left
#define DRIVE_FL_DIR 3
#define DRIVE_FL_ENC_A 5
#define DRIVE_FL_ENC_B 6

#define DRIVE_FR_PWM 7    // Front Right
#define DRIVE_FR_DIR 8
#define DRIVE_FR_ENC_A 9
#define DRIVE_FR_ENC_B 10

#define DRIVE_RL_PWM 11   // Rear Left
#define DRIVE_RL_DIR 12
#define DRIVE_RL_ENC_A 14
#define DRIVE_RL_ENC_B 15

#define DRIVE_RR_PWM 18   // Rear Right
#define DRIVE_RR_DIR 19
#define DRIVE_RR_ENC_A 20
#define DRIVE_RR_ENC_B 21

// Pin definitions - PUSH MOTORS (2 motor)
#define PUSH_UP_PWM 22    // Push Up
#define PUSH_UP_DIR 23
#define PUSH_UP_ENC_A 24
#define PUSH_UP_ENC_B 25

#define PUSH_DOWN_PWM 28  // Push Down
#define PUSH_DOWN_DIR 29
#define PUSH_DOWN_ENC_A 30
#define PUSH_DOWN_ENC_B 31

// PID parameters
const double DRIVE_KP = 2.0;
const double DRIVE_KI = 0.5;
const double DRIVE_KD = 0.1;

const double PUSH_KP = 3.0;
const double PUSH_KI = 0.8;
const double PUSH_KD = 0.2;

// Create PIDs for drive motors
PIDController pidDriveFL(DRIVE_KP, DRIVE_KI, DRIVE_KD);
PIDController pidDriveFR(DRIVE_KP, DRIVE_KI, DRIVE_KD);
PIDController pidDriveRL(DRIVE_KP, DRIVE_KI, DRIVE_KD);
PIDController pidDriveRR(DRIVE_KP, DRIVE_KI, DRIVE_KD);

// Create PIDs for push motors
PIDController pidPushUp(PUSH_KP, PUSH_KI, PUSH_KD);
PIDController pidPushDown(PUSH_KP, PUSH_KI, PUSH_KD);

// Create motor controllers - Drive
MotorController motorDriveFL(1, DRIVE_FL_PWM, DRIVE_FL_DIR);
MotorController motorDriveFR(2, DRIVE_FR_PWM, DRIVE_FR_DIR);
MotorController motorDriveRL(3, DRIVE_RL_PWM, DRIVE_RL_DIR);
MotorController motorDriveRR(4, DRIVE_RR_PWM, DRIVE_RR_DIR);

// Create motor controllers - Push
MotorController motorPushUp(5, PUSH_UP_PWM, PUSH_UP_DIR);
MotorController motorPushDown(6, PUSH_DOWN_PWM, PUSH_DOWN_DIR);

// Create encoders - Drive
Encoder encoderDriveFL(DRIVE_FL_ENC_A, DRIVE_FL_ENC_B);
Encoder encoderDriveFR(DRIVE_FR_ENC_A, DRIVE_FR_ENC_B);
Encoder encoderDriveRL(DRIVE_RL_ENC_A, DRIVE_RL_ENC_B);
Encoder encoderDriveRR(DRIVE_RR_ENC_A, DRIVE_RR_ENC_B);

// Create encoders - Push
Encoder encoderPushUp(PUSH_UP_ENC_A, PUSH_UP_ENC_B);
Encoder encoderPushDown(PUSH_DOWN_ENC_A, PUSH_DOWN_ENC_B);

// Robot controller
RobotController robot;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println("=======================================");
    Serial.println("  6 MOTORS ROBOT CONTROL - Teensy 4.1");
    Serial.println("  4 Drive Motors + 2 Push Motors");
    Serial.println("=======================================");
    
    // Initialize drive motors
    motorDriveFL.begin(); motorDriveFL.setPWMFrequency(20000);
    motorDriveFR.begin(); motorDriveFR.setPWMFrequency(20000);
    motorDriveRL.begin(); motorDriveRL.setPWMFrequency(20000);
    motorDriveRR.begin(); motorDriveRR.setPWMFrequency(20000);
    
    // Initialize push motors
    motorPushUp.begin(); motorPushUp.setPWMFrequency(20000);
    motorPushDown.begin(); motorPushDown.setPWMFrequency(20000);
    
    // Configure PIDs
    pidDriveFL.setOutputLimits(-255, 255);
    pidDriveFR.setOutputLimits(-255, 255);
    pidDriveRL.setOutputLimits(-255, 255);
    pidDriveRR.setOutputLimits(-255, 255);
    pidPushUp.setOutputLimits(-255, 255);
    pidPushDown.setOutputLimits(-255, 255);
    
    // Add motors to robot controller
    robot.addDriveMotor(0, &pidDriveFL, &motorDriveFL, &encoderDriveFL);
    robot.addDriveMotor(1, &pidDriveFR, &motorDriveFR, &encoderDriveFR);
    robot.addDriveMotor(2, &pidDriveRL, &motorDriveRL, &encoderDriveRL);
    robot.addDriveMotor(3, &pidDriveRR, &motorDriveRR, &encoderDriveRR);
    robot.addPushMotor(0, &pidPushUp, &motorPushUp, &encoderPushUp);
    robot.addPushMotor(1, &pidPushDown, &motorPushDown, &encoderPushDown);
    
    Serial.println("\nCOMMANDS:");
    Serial.println("  f<value>  - Move forward (e.g., f1000)");
    Serial.println("  b<value>  - Move backward");
    Serial.println("  l<value>  - Turn left");
    Serial.println("  r<value>  - Turn right");
    Serial.println("  u<value>  - Push up");
    Serial.println("  d<value>  - Push down");
    Serial.println("  m<d><p>   - Move and push (e.g., m1000 500)");
    Serial.println("  sd        - Stop drive motors");
    Serial.println("  sp        - Stop push motors");
    Serial.println("  sa        - Stop ALL motors");
    Serial.println("  x         - Reset ALL");
    Serial.println("  s         - Show status");
    Serial.println("=======================================");
    Serial.println("System ready!\n");
}

void processCommands() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        switch(cmd) {
            case 'f': { // Forward
                double value = Serial.parseFloat();
                robot.moveForward(value);
                break;
            }
            
            case 'b': { // Backward
                double value = Serial.parseFloat();
                robot.moveBackward(value);
                break;
            }
            
            case 'l': { // Turn left
                double value = Serial.parseFloat();
                robot.turnLeft(value);
                break;
            }
            
            case 'r': { // Turn right
                double value = Serial.parseFloat();
                robot.turnRight(value);
                break;
            }
            
            case 'u': { // Push up
                double value = Serial.parseFloat();
                robot.pushUp(value);
                break;
            }
            
            case 'd': { // Push down
                double value = Serial.parseFloat();
                robot.pushDown(value);
                break;
            }
            
            case 'm': { // Move and push
                double driveVal = Serial.parseFloat();
                double pushVal = Serial.parseFloat();
                robot.moveAndPush(driveVal, pushVal);
                break;
            }
            
            case 's': { // Status or Stop
                char subcmd = Serial.peek();
                if (subcmd == 'd') {
                    Serial.read();
                    robot.stopDrive();
                } else if (subcmd == 'p') {
                    Serial.read();
                    robot.stopPush();
                } else if (subcmd == 'a') {
                    Serial.read();
                    robot.stopDrive();
                    robot.stopPush();
                } else {
                    robot.printStatus();
                }
                break;
            }
            
            case 'x': { // Reset
                robot.resetAll();
                break;
            }
        }
    }
}

void loop() {
    processCommands();
    robot.update();
    
    // Optional: Auto print status every 500ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 500) {
        lastPrint = millis();
        // robot.printStatus(); // Uncomment untuk auto print
    }
}