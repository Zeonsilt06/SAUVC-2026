#include "ThrusterControl.h"

/* ================= GLOBAL PWM ================= */
static int thrusterPWM_1 = 1500;
static int thrusterPWM_2 = 1500;
static int thrusterPWM_3 = 1500;
static int thrusterPWM_4 = 1500;

/* ================= PARAMETERS ================= */
static const int step_size  = 2;
static const int step_delay = 25;

/* Horizontal */
static const int cw_H  = 1600;
static const int ccw_H = 1400;

/* Vertical */
static const int cw_V  = 1600;
static const int ccw_V = 1400;

/* ================= SERVOS ================= */
static Servo thrus_1, thrus_2, thrus_3, thrus_4;

/* ================= INTERNAL ================= */
static void smoothMove(int t1, int t2, int t3, int t4);
static void stepTo(int *current, int target);

/* ================= INIT ================= */
void initMotor()
{
    thrus_1.attach(2); // Horizontal Left
    thrus_2.attach(3); // Horizontal Right
    thrus_3.attach(4); // Vertical 1
    thrus_4.attach(5); // Vertical 2
}

/* ================= CORE ================= */
static void smoothMove(int t1, int t2, int t3, int t4)
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

static void stepTo(int *current, int target)
{
    if (*current < target) *current += step_size;
    else if (*current > target) *current -= step_size;
}

/* ================= MOVEMENTS ================= */
void stopAll()
{
    smoothMove(STOP_PWM, STOP_PWM, STOP_PWM, STOP_PWM);
}

void forward(int offset)
{
    smoothMove(cw_H + offset, cw_H + offset, STOP_PWM, STOP_PWM);
}

void reverse(int offset)
{
    smoothMove(ccw_H - offset, ccw_H - offset, STOP_PWM, STOP_PWM);
}

/* ================= ROTATION ================= */
void rotateRight(int offset)
{
    smoothMove(cw_H + offset, ccw_H - offset, STOP_PWM, STOP_PWM);
}

void rotateLeft(int offset)
{
    smoothMove(ccw_H - offset, cw_H + offset, STOP_PWM, STOP_PWM);
}

/* ================= VERTICAL ================= */
void up(int offset)
{
    smoothMove(STOP_PWM, STOP_PWM, cw_V + offset, cw_V + offset);
}

void down(int offset)
{
    smoothMove(STOP_PWM, STOP_PWM, ccw_V - offset, ccw_V - offset);
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