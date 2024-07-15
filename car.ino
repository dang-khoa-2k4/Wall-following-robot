#include "carMazeOfOz.h"

#define STEP_TIME_64 4e-6
#define TIMER1_STEP_CYCLE 65536

const byte trig = 8;
const float BaseSpeed = 70;
const float distanceSetPoint = 7;
const float rateSetPoint = 0;


struct countPulse {
  byte value = 0;
  bool status = false;
};

float PWM_LEFT_SAMPLE[11];
float PWM_RIGHT_SAMPLE[11];
bool getSample = true;
volatile countPulse turn;
bool turnRight = false;
bool turnFinish = true;

carMazeOfOz car;

volatile float speedValueLeft, speedValueRight;
volatile float speedValueLeft_SAMPLE[11], speedValueRight_SAMPLE[11];
volatile unsigned long timerPoint = 0, currentEncoderLeft = 0, currentEncoderRight = 0;

// Your variables is in the area below
//--------------------------------------------------//
byte setSpeedLeft;
byte setSpeedRight;
float setDistance;
float setRate;
float PID;
float pre_error = 0;

//--------------------------------------------------//

//----PID set-----------//

float Kp = 10, Ki = 0.05, Kd = 1.2; // 3 0.05 1 // 10 0.05 1.2
float P;
float I = 0;
float D;


void setup() {
  Serial.begin(9600);
  car.setPin();
  car.setInterrupt();
  attachInterrupt(0, ENC_LEFT_ISR, RISING);
  attachInterrupt(1, ENC_RIGHT_ISR, RISING);

}

void ENC_LEFT_ISR() {
  static byte i = 0;
  speedValueLeft_SAMPLE[10] -= speedValueLeft_SAMPLE[i];
  speedValueLeft_SAMPLE[i] = (TCNT1 - currentEncoderLeft + TIMER1_STEP_CYCLE) % TIMER1_STEP_CYCLE;
  speedValueLeft_SAMPLE[10] += speedValueLeft_SAMPLE[i];
  speedValueLeft = 255254.4 / (speedValueLeft_SAMPLE[10] / 10);
  car.setSpeedLeft(speedValueLeft);
  i = (i + 1) % 10;
  currentEncoderLeft = TCNT1;
  if (turn.status) {
    turn.value++;
  }
}

void ENC_RIGHT_ISR() {
  static byte i = 0;
  speedValueRight_SAMPLE[10] -= speedValueRight_SAMPLE[i];
  speedValueRight_SAMPLE[i] = (TCNT1 - currentEncoderRight + TIMER1_STEP_CYCLE) % TIMER1_STEP_CYCLE;
  speedValueRight_SAMPLE[10] += speedValueRight_SAMPLE[i];
  speedValueRight = 255254.4 / (speedValueRight_SAMPLE[10] / 10);
  car.setSpeedRight(speedValueRight);
  i = (i + 1) % 10;
  currentEncoderRight = TCNT1;
  if (turn.status) {
    turn.value++;
  }
}

// Your functions is in the area below
//--------------------------------------------------//

float KpL = 3.5, KiL = 0.3, KdL = 1;
float IL = 0, DL = 0;
float PL;
float pre_errorL = 0;
float KpR = 3.5, KiR = 0.3, KdR = 1; // 3.5 0.3 1
float IR = 0, DR = 0;
float PR;
float pre_errorR = 0;

void speedLeftPID(int speedSetPoint) { // hoi lai
 float e = speedSetPoint - car.getSpeedLeft();
 PL = KpL * e;
 if (e < -0.5 && e > 0.5 ) {
  IL = 0;
 }
 else IL += KiL * e;
 DL = pre_errorL * KdL;
 pre_errorL = e;
 int PID_left = constrain(PL + IL + DL, -100 , 100);
 setSpeedLeft = constrain(speedSetPoint + PID_left, -150, 150);
 if (setSpeedLeft < 0)
 {
  setSpeedLeft = -1 * setSpeedLeft;
  car.setMotorLeft(setSpeedLeft, 0);
 }
 else 
  car.setMotorLeft(setSpeedLeft, 1);
}

void speedRightPID(int speedSetPoint) {
 float e = speedSetPoint - car.getSpeedRight();
 PR = KpR * e;
 if (e < -0.5 && e > 0.5) {
  IR = 0;
 }
 else IR += KiR * e;
 DR = pre_errorR * KdR;
 pre_errorR = e;
 int PID_right = constrain(PR + IR + DR, -100, 100);
 setSpeedRight = constrain(speedSetPoint + PID_right, -150, 150);

 if (setSpeedRight < 0)
 {
  setSpeedRight = -1 * setSpeedRight;
  car.setMotorRight(setSpeedRight, 0);
 }
 else 
  car.setMotorRight(setSpeedRight, 1);
}

void distancePID() {

  float e = distanceSetPoint - car.getDistanceLeft();
  P = Kp * e;
  if (e > -0.5 && e < 0.5) {
    I = 0;
  } else {
    I += Ki * e;
  }
  D = pre_error * Kd;
  pre_error = e;
  // setDistance = constrain(P + I, -50, 50);
  PID = constrain(P + I + D, -100, 100);
  int left = BaseSpeed + PID;
  int right =  BaseSpeed - PID; 
  speedLeftPID(left); 
  speedRightPID(right);

  if (left < 0)
  {
      left = - 1 * left;
      car.setMotorLeft(left, 0);
  }
  else 
    car.setMotorLeft(left, 1);
  if (right < 0)
  {
      right = - 1 * right;
      car.setMotorRight(right, 0);
  }
  else 
    car.setMotorRight(right, 1);
}

// void rateOfChangeDistancePID() {
//   float Kp = 0.1, Ki = 0.1;
//   float e = rateSetPoint - car.getRateOfChangeDistanceLeft();
//   float P = Kp * e;
//   static float I = 0;
//   if (e > -0.5 && e < 0.5) {
//     I = 0;
//   } else {
//     I += Ki * e;
//   }
//   setRate = constrain(P + I, -50, 50);

// }

void stopMotor() {
  car.setMotorLeft(0, 1);
  car.setMotorRight(0, 1);
}


//--------------------------------------------------//

void loop() {
  digitalWrite(trig, LOW);
  // car.setSpeedLeft(speedValueLeft);
  // car.setSpeedRight(speedValueRight);
  // car.configureSpeed(speedValueLeft, speedValueRight);

  // Your code is in the area below
  //--------------------------------------------------//
  distancePID();
  // speedRightPID(70); 
//  speedLeftPID(40);
//  car.setMotorRight(setSpeedRight, 1);
//  car.setMotorLeft(setSpeedLeft, 1);        for testing
  // Serial.println(car.getSpeedRight());
//  Serial.print(" ");
//  Serial.println(car.getSpeedLeft());
//  car.setMotorLeft(150, 1);
  if (car.getDistanceHead() < 10 && car.getDistanceLeft() < 10)
  {
    car.setMotorRight(175, 0);
    car.setMotorLeft(255, 1);
    delay(150);
  }
  else if (car.getDistanceHead() < 5)
  {
      car.setMotorRight(175, 0);
      car.setMotorLeft(255, 1);
      delay(200);
  }
//  Serial.print(" ");
//  Serial.print(car.getDistanceHead());
//  Serial.print(" ");
//  Serial.println(car.getDistanceRight());
  
  //--------------------------------------------------//
}
