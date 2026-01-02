#include "Thrustercontrol.h"

/* ================= GLOBAL PWM ================= */
static int currentPWM[4] = {1500, 1500, 1500, 1500};
static int targetPWM[4]  = {1500, 1500, 1500, 1500};

#define PWM_MIN 1100
#define PWM_MAX 1900

// Kecepatan transisi PWM (Slew Rate)
// Teensy sangat cepat, nilai 5-10 biasanya pas untuk kehalusan motor
static const int step_size = 8; 

/* ================= SERVOS ================= */
static Servo thrus_1, thrus_2, // horizontal (Pin 2, 3)
            thrus_3, thrus_4; // vertical (Pin 4, 5)

/* ================= INIT ================= */
void initMotor() {
    // Set awal ke netral
    thrus_1.writeMicroseconds(1500);
    thrus_2.writeMicroseconds(1500);
    thrus_3.writeMicroseconds(1500);
    thrus_4.writeMicroseconds(1500);

    thrus_1.attach(2); 
    thrus_2.attach(3); 
    thrus_3.attach(4); 
    thrus_4.attach(5);
    
    delay(100); 
}

/* ================= CORE (NON-BLOCKING) ================= */
// FUNGSI BARU: Wajib dipanggil di loop() agar motor bergerak halus ke target
void updateMotors() {
    for(int i = 0; i < 4; i++) {
        if (currentPWM[i] < targetPWM[i]) currentPWM[i] += step_size;
        else if (currentPWM[i] > targetPWM[i]) currentPWM[i] -= step_size;

        // Kunci nilai jika selisih sudah sangat kecil
        if (abs(currentPWM[i] - targetPWM[i]) < step_size) currentPWM[i] = targetPWM[i];
    }

    thrus_1.writeMicroseconds(constrain(currentPWM[0], PWM_MIN, PWM_MAX));
    thrus_2.writeMicroseconds(constrain(currentPWM[1], PWM_MIN, PWM_MAX));
    thrus_3.writeMicroseconds(constrain(currentPWM[2], PWM_MIN, PWM_MAX));
    thrus_4.writeMicroseconds(constrain(currentPWM[3], PWM_MIN, PWM_MAX));
}

/* ================= MOVEMENTS ================= */
void stopAll() {
    for(int i=0; i<4; i++) targetPWM[i] = 1500;
}

void forward(int offset) {
    targetPWM[0] = 1500 + offset; // Motor Horiz L
    targetPWM[1] = 1500 + offset; // Motor Horiz R
}

void reverse(int offset) {
    targetPWM[0] = 1500 - offset;
    targetPWM[1] = 1500 - offset;
}

void rotateRight(int offset) {
    targetPWM[0] = 1500 + offset; // Kiri maju
    targetPWM[1] = 1500 - offset; // Kanan mundur
}

void rotateLeft(int offset) {
    targetPWM[0] = 1500 - offset; // Kiri mundur
    targetPWM[1] = 1500 + offset; // Kanan maju
}

void up(int offset) {
    targetPWM[2] = 1500 + offset; // Motor Vert 1
    targetPWM[3] = 1500 + offset; // Motor Vert 2
}

void down(int offset) {
    targetPWM[2] = 1500 - offset;
    targetPWM[3] = 1500 - offset;
}

// Tambahkan implementasi rotate atau reverse sesuai pola targetPWM di atas