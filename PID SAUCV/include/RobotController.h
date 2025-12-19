#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <Arduino.h>
#include <Encoder.h>
#include "PIDController.h"
#include "MotorController.h"

class RobotController {
private:
    // Drive motors (4 motor untuk maju/mundur/belok)
    PIDController* driveMotors[4];
    MotorController* driveControllers[4];
    Encoder* driveEncoders[4];
    double driveSetpoints[4];
    double drivePositions[4];
    
    // Push motors (2 motor untuk atas/bawah)
    PIDController* pushMotors[2];
    MotorController* pushControllers[2];
    Encoder* pushEncoders[2];
    double pushSetpoints[2];
    double pushPositions[2];
    
public:
    RobotController();
    
    // Setup motors
    void addDriveMotor(int index, PIDController* pid, MotorController* motor, Encoder* encoder);
    void addPushMotor(int index, PIDController* pid, MotorController* motor, Encoder* encoder);
    
    // Drive control (gerak maju/mundur/belok)
    void moveForward(double distance);
    void moveBackward(double distance);
    void turnLeft(double angle);
    void turnRight(double angle);
    void stopDrive();
    
    // Push control (gerak atas/bawah)
    void pushUp(double distance);
    void pushDown(double distance);
    void pushBoth(double upDistance, double downDistance);
    void stopPush();
    
    // Combined movements
    void moveAndPush(double driveDistance, double pushDistance);
    
    // Update functions
    void update();
    void updateDrive();
    void updatePush();
    
    // Reset functions
    void resetAll();
    void resetDrive();
    void resetPush();
    
    // Status getters
    double getDrivePosition(int index);
    double getPushPosition(int index);
    void printStatus();
};

#endif