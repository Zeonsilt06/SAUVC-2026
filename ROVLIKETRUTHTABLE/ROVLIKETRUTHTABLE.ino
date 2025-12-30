#include <Servo.h>

Servo T1, T2, T3, T4;

const int STOP = 1500;
const int SLOW = 100;

void setup() {
  T1.attach(2);
  T2.attach(3);
  T3.attach(4);
  T4.attach(5);

  stopAll();
  delay(3000); // ESC arming
}

void loop() {

  // 1. DIAM
  move(STOP, STOP, STOP, STOP);

  // 2. MAJU
  move(CW(), CW(), STOP, STOP);

  // 3. MUNDUR
  move(CCW(), CCW(), STOP, STOP);

  // 4. ROTASI KANAN (Yaw +)
  move(CW(), CCW(), STOP, STOP);

  // 5. ROTASI KIRI (Yaw -)
  move(CCW(), CW(), STOP, STOP);

  // 6. NAIK
  move(STOP, STOP, CW(), CW());

  // 7. TURUN
  move(STOP, STOP, CCW(), CCW());

  // 8. MAJU + NAIK
  move(CW(), CW(), CW(), CW());

  // 9. MAJU + TURUN
  move(CW(), CW(), CCW(), CCW());

  // 10. MUNDUR + NAIK
  move(CCW(), CCW(), CW(), CW());

  // 11. MUNDUR + TURUN
  move(CCW(), CCW(), CCW(), CCW());

  // 12. ROTASI KANAN + NAIK
  move(CW(), CCW(), CW(), CW());

  // 13. ROTASI KANAN + TURUN
  move(CW(), CCW(), CCW(), CCW());

  // 14. ROTASI KIRI + NAIK
  move(CCW(), CW(), CW(), CW());

  // 15. ROTASI KIRI + TURUN
  move(CCW(), CW(), CCW(), CCW());
}

//////////////////// FUNCTIONS ////////////////////

int CW() {
  return STOP + SLOW;
}

int CCW() {
  return STOP - SLOW;
}

void move(int t1, int t2, int t3, int t4) {
  T1.writeMicroseconds(t1);
  T2.writeMicroseconds(t2);
  T3.writeMicroseconds(t3);
  T4.writeMicroseconds(t4);
  delay(2000);
  stopAll();
}

void stopAll() {
  T1.writeMicroseconds(STOP);
  T2.writeMicroseconds(STOP);
  T3.writeMicroseconds(STOP);
  T4.writeMicroseconds(STOP);
  delay(1000);
}