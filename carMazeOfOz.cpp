#include <Arduino.h>
#include <avr/interrupt.h>
#include "carMazeOfOz.h"

#define SOUND_SPEED 34320
#define MAX_PWM 255
#define MIN_PWM 0
#define STEP_TIME_64 4e-6
#define TIMER1_STEP_CYCLE 65536

const byte IN1 = 4;
const byte IN2 = 5;
const byte IN3 = 6;
const byte IN4 = 7;
const byte encLeft = 2;
const byte encRight = 3;
const byte trig = 8;
const byte echoHead = 9;
const byte echoLeft = 10;
const byte echoRight = 11;

volatile float distanceHead, distanceLeft, distanceRight;
volatile float rateOfChangeDistanceHead, rateOfChangeDistanceLeft, rateOfChangeDistanceRight;
volatile float lastDistanceHead = 0, lastDistanceLeft = 0, lastDistanceRight = 0;
volatile float distanceHead_SAMPLE[11], distanceLeft_SAMPLE[11], distanceRight_SAMPLE[11];
volatile float rateOfChangeDistanceHead_SAMPLE[11], rateOfChangeDistanceLeft_SAMPLE[11], rateOfChangeDistanceRight_SAMPLE[11];
volatile unsigned long currentSensorHead = 0, currentSensorLeft = 0, currentSensorRight = 0;
volatile byte allSensor = 0;
volatile bool headHigh = false, leftHigh = false, rightHigh = false;
volatile bool headIsRec = false;

carMazeOfOz::carMazeOfOz() {
  ;
}

void carMazeOfOz::setPin() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(encLeft, INPUT);
  pinMode(encRight, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echoHead, INPUT);
  pinMode(echoLeft, INPUT);
  pinMode(echoRight, INPUT);
}

void carMazeOfOz::setInterrupt() {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;

  TCCR1B |= (1 << CS11) | (1 << CS10);
  TCNT1 = 0;
  TIMSK1 = (1 << TOIE1);

  PCICR = 0;
  PCMSK0 = 0;

  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT1) | (1 << PCINT2) | (1 << PCINT3);
  sei();

  digitalWrite(trig, LOW);
  distanceHead_SAMPLE[10] = 100;
  distanceLeft_SAMPLE[10] = 100;
  distanceRight_SAMPLE[10] = 100;
}

ISR (TIMER1_OVF_vect) {
  TCNT1 = 0;
}

ISR (PCINT0_vect) {
  if (digitalRead(echoHead) != headHigh) {
    if (headHigh){
      static byte i = 0;
      distanceHead_SAMPLE[10] -= distanceHead_SAMPLE[i];
      distanceHead_SAMPLE[i] = (TCNT1 - currentSensorHead + TIMER1_STEP_CYCLE) % TIMER1_STEP_CYCLE;
      distanceHead_SAMPLE[10] += distanceHead_SAMPLE[i];
      distanceHead = ((SOUND_SPEED * STEP_TIME_64) * (distanceHead_SAMPLE[10] / 10)) / 2;
      rateOfChangeDistanceHead_SAMPLE[10] -= rateOfChangeDistanceHead_SAMPLE[i];
      rateOfChangeDistanceHead_SAMPLE[i] = distanceHead - lastDistanceHead;
      rateOfChangeDistanceHead_SAMPLE[10] += rateOfChangeDistanceHead_SAMPLE[i];
      rateOfChangeDistanceHead = rateOfChangeDistanceHead_SAMPLE[10];
      lastDistanceHead = distanceHead;
      i = (i + 1) % 10;
      ++allSensor;
    } else {
      currentSensorHead = TCNT1;
    }
    headHigh = !headHigh;
  }
  if (digitalRead(echoLeft) != leftHigh) {
    if (leftHigh) {
      static byte i = 0;
      distanceLeft_SAMPLE[10] -= distanceLeft_SAMPLE[i];
      distanceLeft_SAMPLE[i] = (TCNT1 - currentSensorLeft + TIMER1_STEP_CYCLE) % TIMER1_STEP_CYCLE;
      distanceLeft_SAMPLE[10] += distanceLeft_SAMPLE[i];
      distanceLeft = ((SOUND_SPEED * STEP_TIME_64) * (distanceLeft_SAMPLE[10] / 10)) / 2;
      rateOfChangeDistanceLeft_SAMPLE[10] -= rateOfChangeDistanceLeft_SAMPLE[i];
      rateOfChangeDistanceLeft_SAMPLE[i] = distanceLeft - lastDistanceLeft;
      rateOfChangeDistanceLeft_SAMPLE[10] += rateOfChangeDistanceLeft_SAMPLE[i];
      rateOfChangeDistanceLeft = rateOfChangeDistanceLeft_SAMPLE[10];
      lastDistanceLeft = distanceLeft;
      i = (i + 1) % 10;
      ++allSensor;
    } else {
      currentSensorLeft = TCNT1; 
    }
    leftHigh = !leftHigh;
  }
  if (digitalRead(echoRight) != rightHigh) {
    if (rightHigh) {
      static byte i = 0;
      distanceRight_SAMPLE[10] -= distanceRight_SAMPLE[i];
      distanceRight_SAMPLE[i] = (TCNT1 - currentSensorRight + TIMER1_STEP_CYCLE) % TIMER1_STEP_CYCLE;
      distanceRight_SAMPLE[10] += distanceRight_SAMPLE[i];
      distanceRight = ((SOUND_SPEED * STEP_TIME_64) * (distanceRight_SAMPLE[10] / 10)) / 2;
      rateOfChangeDistanceRight_SAMPLE[10] -= rateOfChangeDistanceRight_SAMPLE[i];
      rateOfChangeDistanceRight_SAMPLE[i] = distanceRight - lastDistanceRight;
      rateOfChangeDistanceRight_SAMPLE[10] += rateOfChangeDistanceRight_SAMPLE[i];
      rateOfChangeDistanceRight = rateOfChangeDistanceRight_SAMPLE[10];
      lastDistanceRight = distanceRight;
      i = (i + 1) % 10;
      ++allSensor;
    } else {
      currentSensorRight = TCNT1; 
    }
    rightHigh = !rightHigh;
  }
  if (allSensor == 3) {
    allSensor = 0;
    digitalWrite(trig, HIGH);
  }
}

void carMazeOfOz::setMotorLeft(byte speed, bool direction) {
  speed = constrain(speed, MIN_PWM, MAX_PWM);
  digitalWrite(IN4, direction);
  analogWrite(IN3, abs(255 * direction - speed));
}
void carMazeOfOz::setMotorRight(byte speed, bool direction) {
  speed = constrain(speed, MIN_PWM, MAX_PWM);
  digitalWrite(IN1, direction);
  analogWrite(IN2, abs(255 * direction - speed));
}

void carMazeOfOz::setSpeedLeft(volatile float speed) {
  speedValueLeft = speed;
}

void carMazeOfOz::setSpeedRight(volatile float speed) {
  speedValueRight = speed;
}

float carMazeOfOz::getSpeedLeft() {
  return speedValueLeft;
}

float carMazeOfOz::getSpeedRight() {
  return speedValueRight;
}

float carMazeOfOz::getDistanceHead() {
  return distanceHead;
}

float carMazeOfOz::getDistanceLeft() {
  return distanceLeft;
}

float carMazeOfOz::getDistanceRight() {
  return distanceRight;
}

float carMazeOfOz::getRateOfChangeDistanceHead() {
  return rateOfChangeDistanceHead;
}
float carMazeOfOz::getRateOfChangeDistanceLeft() {
  return rateOfChangeDistanceLeft;
}
float carMazeOfOz::getRateOfChangeDistanceRight() {
  return rateOfChangeDistanceRight;
}

void carMazeOfOz::configureSpeed(volatile float &speedValueLeft, volatile float &speedValueRight) {
  static byte countStopLeft = 0, countStopRight = 0;
  static float lastSpeedValueLeft = speedValueLeft;
  static float lastSpeedValueRight = speedValueRight;
  if (lastSpeedValueLeft == speedValueLeft) countStopLeft++;
  else countStopLeft = 0;
  if (lastSpeedValueRight == speedValueRight) countStopRight++;
  else countStopRight = 0;
  if (countStopLeft > 10) speedValueLeft = 0;
  if (countStopRight > 10) speedValueRight = 0;
  lastSpeedValueLeft = speedValueLeft;
  lastSpeedValueRight = speedValueRight;
}
