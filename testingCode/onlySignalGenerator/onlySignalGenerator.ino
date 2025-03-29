#include <TimerOne.h>

#define SQUAREWAVE_FREQ 200

volatile bool signalState = LOW;
void toggleSignal() {
  signalState = !signalState;
  digitalWrite(4, signalState);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(4, OUTPUT);
  Timer1.initialize(1000000/(SQUAREWAVE_FREQ*2));
  Timer1.attachInterrupt(toggleSignal);
}

void loop() {
  // put your main code here, to run repeatedly:

}
