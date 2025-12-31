/* ================= LIBRARIES ================= */
#include <Servo.h>

/* ================= CONSTANTS ================= */
#define baud_rate 9600

/* ================= GLOBAL VARIABLES ================= */
// Current PWM values
int thrusterPWM_1 = 1500;
int thrusterPWM_2 = 1500;
int thrusterPWM_3 = 1500;
int thrusterPWM_4 = 1500;

// Smooth movement parameters (SLOW)
int step_size  = 2;     // smaller = slower
int step_delay = 25;    // ms delay per step

// Horizontal motors
int cw_H  = 1600;       // slow forward
int ccw_H = 1400;       // slow reverse

// Vertical motors
int cw_V  = 1600;       // slow up
int ccw_V = 1400;       // slow down

// Stop
int stop = 1500;

/* ================= SERVOS ================= */
Servo thrus_1, thrus_2, thrus_3, thrus_4;

/* ================= SETUP ================= */
void setup()
{
  Serial.begin(baud_rate);
  initMotor();
  stopAll();
}

/* ================= LOOP (DEMO) ================= */
void loop()
{
  forward(0);
  delay(3000);

  rotateRight(0);
  delay(3000);

  up(0);
  delay(3000);

  rotateLeftUp(0, 0);
  delay(3000);

  stopAll();
  delay(4000);
}

/* ================= MOTOR INIT ================= */
void initMotor()
{
  thrus_1.attach(2); // Horizontal Left
  thrus_2.attach(3); // Horizontal Right
  thrus_3.attach(4); // Vertical 1
  thrus_4.attach(5); // Vertical 2
}

/* ================= SMOOTH MOVE CORE ================= */
void smoothMove(int t1, int t2, int t3, int t4)
{
  while (thrusterPWM_1 != t1 ||
         thrusterPWM_2 != t2 ||
         thrusterPWM_3 != t3 ||
         thrusterPWM_4 != t4)
  {
    stepTo(&thrusterPWM_1, t1);
    stepTo(&thrusterPWM_2, t2);
    stepTo(&thrusterPWM_3, t3);
    stepTo(&thrusterPWM_4, t4);

    thrus_1.writeMicroseconds(thrusterPWM_1);
    thrus_2.writeMicroseconds(thrusterPWM_2);
    thrus_3.writeMicroseconds(thrusterPWM_3);
    thrus_4.writeMicroseconds(thrusterPWM_4);

    delay(step_delay);
  }
}

void stepTo(int *current, int target)
{
  if (*current < target) *current += step_size;
  else if (*current > target) *current -= step_size;
}

/* ================= BASIC MOVEMENTS ================= */
void stopAll()
{
  smoothMove(stop, stop, stop, stop);
}

void forward(int offset)
{
  smoothMove(cw_H + offset, cw_H + offset, stop, stop);
}

void reverse(int offset)
{
  smoothMove(ccw_H - offset, ccw_H - offset, stop, stop);
}

/* ================= SLOW ROTATION (YAW) ================= */
void rotateRight(int offset)
{
  // Smaller differential for slow yaw
  smoothMove(cw_H + offset, ccw_H - offset, stop, stop);
}

void rotateLeft(int offset)
{
  smoothMove(ccw_H - offset, cw_H + offset, stop, stop);
}

/* ================= VERTICAL ================= */
void up(int offset)
{
  smoothMove(stop, stop, cw_V + offset, cw_V + offset);
}

void down(int offset)
{
  smoothMove(stop, stop, ccw_V - offset, ccw_V - offset);
}

/* ================= COMBINED ================= */
void rotateRightUp(int offsetH, int offsetV)
{
  smoothMove(cw_H + offsetH, ccw_H - offsetH,
             cw_V + offsetV, cw_V + offsetV);
}

void rotateLeftUp(int offsetH, int offsetV)
{
  smoothMove(ccw_H - offsetH, cw_H + offsetH,
             cw_V + offsetV, cw_V + offsetV);
}

void rotateRightDown(int offsetH, int offsetV)
{
  smoothMove(cw_H + offsetH, ccw_H - offsetH,
             ccw_V - offsetV, ccw_V - offsetV);
}

void rotateLeftDown(int offsetH, int offsetV)
{
  smoothMove(ccw_H - offsetH, cw_H + offsetH,
             ccw_V - offsetV, ccw_V - offsetV);
}
