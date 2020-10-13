#include <Wire.h>
#include <stdlib.h>
#include <arduino.h>

// Reciever Channel 1
volatile int rec_input_ch_1 = 0;
volatile int rec_input_ch_1_timer = 0;
volatile int last_ch_1 = 0;

const int ledpin = 13;
const int interruptPin = 24;
volatile byte state = LOW;

int i = 0;

// ISR's
void blink() {

  if (last_ch_1 == 0 && digitalRead(interruptPin)){
    last_ch_1 = 1;
    rec_input_ch_1_timer = micros();
  }
  else if (last_ch_1 == 1 && !digitalRead(interruptPin)){
    last_ch_1 = 0;
    rec_input_ch_1 = micros() - rec_input_ch_1_timer;
  }

}

// Main (setup & loop)
void setup() {
  Serial.begin(9600);

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
}

void loop(){
  Serial.print(rec_input_ch_1);
  Serial.print("\n");
  //digitalWrite(ledpin, state);
}
