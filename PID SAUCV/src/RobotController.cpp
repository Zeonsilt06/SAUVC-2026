#include "RobotController.h"

RobotController::RobotController() {
    for (int i = 0; i < 4; i++) {
        driveMotors[i] = nullptr;
        driveControllers[i] = nullptr;
        driveEncoders[i] = nullptr;
        driveSetpoints[i] = 0;
        drivePositions[i] = 0;
    }
    
    for (int i = 0; i < 2; i++) {
        pushMotors[i] = nullptr;
        pushControllers[i] = nullptr;
        pushEncoders[i] = nullptr;
        pushSetpoints[i] = 0;
        pushPositions[i] = 0;
    }
}

void RobotController::addDriveMotor(int index, PIDController* pid, MotorController* motor, Encoder* encoder) {
    if (index >= 0 && index < 4) {
        driveMotors[index] = pid;
        driveControllers[index] = motor;
        driveEncoders[index] = encoder;
    }
}

void RobotController::addPushMotor(int index, PIDController* pid, MotorController* motor, Encoder* encoder) {
    if (index >= 0 && index < 2) {
        pushMotors[index] = pid;
        pushControllers[index] = motor;
        pushEncoders[index] = encoder;
    }
}

void RobotController::moveForward(double distance) {
    // Set semua drive motor ke setpoint yang sama (maju)
    for (int i = 0; i < 4; i++) {
        driveSetpoints[i] = distance;
    }
    Serial.print("Moving forward: ");
    Serial.println(distance);
}

void RobotController::moveBackward(double distance) {
    // Set semua drive motor ke setpoint negatif (mundur)
    for (int i = 0; i < 4; i++) {
        driveSetpoints[i] = -distance;
    }
    Serial.print("Moving backward: ");
    Serial.println(distance);
}

void RobotController::turnLeft(double angle) {
    // Motor kiri mundur, motor kanan maju
    driveSetpoints[0] = -angle; // Front Left
    driveSetpoints[1] = angle;  // Front Right
    driveSetpoints[2] = -angle; // Rear Left
    driveSetpoints[3] = angle;  // Rear Right
    Serial.print("Turning left: ");
    Serial.println(angle);
}

void RobotController::turnRight(double angle) {
    // Motor kiri maju, motor kanan mundur
    driveSetpoints[0] = angle;  // Front Left
    driveSetpoints[1] = -angle; // Front Right
    driveSetpoints[2] = angle;  // Rear Left
    driveSetpoints[3] = -angle; // Rear Right
    Serial.print("Turning right: ");
    Serial.println(angle);
}

void RobotController::stopDrive() {
    for (int i = 0; i < 4; i++) {
        if (driveControllers[i] != nullptr) {
            driveControllers[i]->stop();
        }
        driveSetpoints[i] = drivePositions[i]; // Hold position
    }
    Serial.println("Drive STOPPED");
}

void RobotController::pushUp(double distance) {
    pushSetpoints[0] = distance;  // Push motor atas
    Serial.print("Push UP: ");
    Serial.println(distance);
}

void RobotController::pushDown(double distance) {
    pushSetpoints[1] = distance;  // Push motor bawah
    Serial.print("Push DOWN: ");
    Serial.println(distance);
}

void RobotController::pushBoth(double upDistance, double downDistance) {
    pushSetpoints[0] = upDistance;
    pushSetpoints[1] = downDistance;
    Serial.print("Push UP: ");
    Serial.print(upDistance);
    Serial.print(" | DOWN: ");
    Serial.println(downDistance);
}

void RobotController::stopPush() {
    for (int i = 0; i < 2; i++) {
        if (pushControllers[i] != nullptr) {
            pushControllers[i]->stop();
        }
        pushSetpoints[i] = pushPositions[i]; // Hold position
    }
    Serial.println("Push STOPPED");
}

void RobotController::moveAndPush(double driveDistance, double pushDistance) {
    moveForward(driveDistance);
    pushUp(pushDistance);
    Serial.println("Move and Push simultaneously");
}

void RobotController::update() {
    updateDrive();
    updatePush();
}

void RobotController::updateDrive() {
    for (int i = 0; i < 4; i++) {
        if (driveEncoders[i] != nullptr && driveMotors[i] != nullptr && driveControllers[i] != nullptr) {
            drivePositions[i] = driveEncoders[i]->read();
            double output = driveMotors[i]->compute(driveSetpoints[i], drivePositions[i]);
            driveControllers[i]->drive(output);
        }
    }
}

void RobotController::updatePush() {
    for (int i = 0; i < 2; i++) {
        if (pushEncoders[i] != nullptr && pushMotors[i] != nullptr && pushControllers[i] != nullptr) {
            pushPositions[i] = pushEncoders[i]->read();
            double output = pushMotors[i]->compute(pushSetpoints[i], pushPositions[i]);
            pushControllers[i]->drive(output);
        }
    }
}

void RobotController::resetAll() {
    resetDrive();
    resetPush();
    Serial.println("ALL systems RESET");
}

void RobotController::resetDrive() {
    for (int i = 0; i < 4; i++) {
        if (driveEncoders[i] != nullptr) driveEncoders[i]->write(0);
        if (driveMotors[i] != nullptr) driveMotors[i]->reset();
        driveSetpoints[i] = 0;
    }
    Serial.println("Drive RESET");
}

void RobotController::resetPush() {
    for (int i = 0; i < 2; i++) {
        if (pushEncoders[i] != nullptr) pushEncoders[i]->write(0);
        if (pushMotors[i] != nullptr) pushMotors[i]->reset();
        pushSetpoints[i] = 0;
    }
    Serial.println("Push RESET");
}

double RobotController::getDrivePosition(int index) {
    return (index >= 0 && index < 4) ? drivePositions[index] : 0;
}

double RobotController::getPushPosition(int index) {
    return (index >= 0 && index < 2) ? pushPositions[index] : 0;
}

void RobotController::printStatus() {
    Serial.println("========== ROBOT STATUS ==========");
    Serial.println("DRIVE MOTORS:");
    for (int i = 0; i < 4; i++) {
        Serial.print("  M");
        Serial.print(i + 1);
        Serial.print(" SP:");
        Serial.print(driveSetpoints[i], 0);
        Serial.print(" Pos:");
        Serial.print(drivePositions[i], 0);
        Serial.print(" Err:");
        Serial.println(driveSetpoints[i] - drivePositions[i], 0);
    }
    Serial.println("PUSH MOTORS:");
    Serial.print("  UP   SP:");
    Serial.print(pushSetpoints[0], 0);
    Serial.print(" Pos:");
    Serial.println(pushPositions[0], 0);
    Serial.print("  DOWN SP:");
    Serial.print(pushSetpoints[1], 0);
    Serial.print(" Pos:");
    Serial.println(pushPositions[1], 0);
    Serial.println("==================================");
}