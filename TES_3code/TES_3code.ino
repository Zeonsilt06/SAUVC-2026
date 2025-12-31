/* Libraries */
#include <Servo.h>

/* Constants */
#define baud_rate 9600

/* Global Vars */
// The PWM value at which the motor stops spinning
int thrusterPWM_1 = 1500, thrusterPWM_2 = 1500, thrusterPWM_3 = 1500, thrusterPWM_4 = 1500;
// add xx ms delay after one cycle
int step_delay = 10;
// initial PWM values for horizontal motors
int cw_H   = 1700; 
int ccw_H = 1300;  
// initial PWM values for vertical motors
int cw_V   = 1700; 
int ccw_V = 1300; 
// stop the motor at some point
int stop   = 1500;

/* Classes */
// Horizontal Motor
Servo thrus_1, thrus_2, thrus_3, thrus_4;

void setup()
{
  initComm();
  initMotor();
}

void loop()
{
 // 1. forward
  Serial.println("Gerakan: forward");
  forward(0);
  delay(10000);
  stopall();
  delay(2000);

  // 2. reverse
  Serial.println("Gerakan: reverse");
  reverse(0);
  delay(10000);
  stopall();
  delay(2000);

  // 3. turnRight
  Serial.println("Gerakan: turnRight");
  turnRight(0);
  delay(10000);
  stopall();
  delay(2000);

  // 4.  turnLeft
  Serial.println("Gerakan:  turnLeft");
  turnLeft(0);
  delay(10000);
  stopall();
  delay(2000);

  // 5. up
  Serial.println("Gerakan: up");
  up(0);
  delay(10000);
  stopall();
  delay(2000);

  // 6. down
  Serial.println("Gerakan: down");
  down(0);
  delay(10000);
  stopall();
  
  Serial.println("--- Selesai, Mengulang ---");
  delay(5000);

}


void initComm()
{
  Serial.begin(baud_rate);
}

void initMotor()
{
  // Pin Motor Horizontal (cw_H, ccw_H, Belok)
  thrus_1.attach(2); // Motor Kiri
  thrus_2.attach(3); // Motor Kanan
  
  // Pin Motor Vertikal (Naik, down)
  thrus_3.attach(4); // Vertikal 1
  thrus_4.attach(5); // Vertikal 2
}

// --- FUNGSI INTI PERPINDAHAN HALUS ---
void smoothMove(int thrus1, int thrus2, int thrus3, int thrus4) {
  while (thrusterPWM_1 != thrus1 || thrusterPWM_2 != thrus2 || thrusterPWM_3 != thrus3 || thrusterPWM_4 != thrus4) {
    // Thruster 1
    if (thrusterPWM_1 < thrus1)
    { 
      thrusterPWM_1 += 5;
    }
    else if (thrusterPWM_1 > thrus1)
    {
      thrusterPWM_1 -= 5;
    }
    else
    {
      //do nothing
    }
     
    // Thruster 2
    if (thrusterPWM_2 < thrus2)
    {
      thrusterPWM_2 += 5;
    }
    else if (thrusterPWM_2 > thrus2)
    {
      thrusterPWM_2 -= 5;
    }
    else
    {
      //do nothing
    }

    // Thruster 3
    if (thrusterPWM_3 < thrus3)
    {
      thrusterPWM_3 += 5;
    }
    else if (thrusterPWM_3 > thrus3)
    {
      thrusterPWM_3 -= 5;
    }
    else
    {
      //do nothing
    }
    
    // Thruster 4
    if (thrusterPWM_4 < thrus4)
    {
      thrusterPWM_4 += 5;
    }
    else if(thrusterPWM_4 > thrus4)
    {
      thrusterPWM_4 -= 5;
    }
    else
    {
      //do nothing
    }
    thrus_1.writeMicroseconds(thrusterPWM_1);
    thrus_2.writeMicroseconds(thrusterPWM_2);
    thrus_3.writeMicroseconds(thrusterPWM_3);
    thrus_4.writeMicroseconds(thrusterPWM_4);
    delay(step_delay); 
  }
}

void forward(int offset)
{
  smoothMove(cw_H, ccw_H, stop, stop);
}

void reverse(int offset)
{
  smoothMove(ccw_H, cw_H, stop, stop);
}

void turnLeft(int offset)
{
  smoothMove(ccw_H, ccw_H, stop, stop);
}

void turnRight(int offset)
{
  smoothMove(cw_H, cw_H, stop, stop);
}

void rotateLeft(int offset)
{

}

void rotateRight(int offset)
{
  
}

void up(int offset)
{
  smoothMove(stop, stop, cw_V, cw_V);
}

void down(int offset)
{
  smoothMove(stop, stop, ccw_V, ccw_V);
}
void stopall()
{
    smoothMove(stop, stop, stop, stop);
}
