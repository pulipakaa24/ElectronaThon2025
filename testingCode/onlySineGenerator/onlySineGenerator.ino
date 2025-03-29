#include <TimerOne.h>

#define SINEWAVE_STEP 3200

uint8_t dacOut[32] = {16, 19, 22, 24, 27, 28, 30, 31, 31, 31, 30, 28, 27, 24, 22, 19, 16, 13,
10, 8, 5, 4, 2, 1, 1, 1, 2, 4, 5, 8, 10, 13};

volatile uint8_t sineIndex = 0;
void stepSine() {
  uint8_t out = dacOut[sineIndex%32];
  digitalWrite(5, out / 16);
  out = out % 16;
  digitalWrite(6, out / 8);
  out = out % 8;
  digitalWrite(7, out / 4);
  out = out % 4;
  digitalWrite(8, out / 2);
  out = out % 2;
  digitalWrite(9, out);
  sineIndex++;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  Timer1.initialize(1000000/SINEWAVE_STEP);
  Timer1.attachInterrupt(stepSine);
}

void loop() {

}