#include <Wire.h>
#include <stdlib.h>
#include <arduino.h>

// Radio Reciever Pins:
const int rec_input_1_pin = 22;
const int rec_input_2_pin = 23;
const int rec_input_3_pin = 24;
const int rec_input_4_pin = 25;

// Radio Reciever
volatile int rec_input_ch_1, rec_input_ch_1_timer, rec_last_ch_1;
volatile int rec_input_ch_2, rec_input_ch_2_timer, rec_last_ch_2;
volatile int rec_input_ch_3, rec_input_ch_3_timer, rec_last_ch_3;
volatile int rec_input_ch_4, rec_input_ch_4_timer, rec_last_ch_4;

// ISR's
void radio_reciever_input() {
  // Channel 1: Roll
  if (rec_last_ch_1 == 0 && digitalRead(rec_input_1_pin)){
    rec_last_ch_1 = 1;
    rec_input_ch_1_timer = micros();
  }
  else if (rec_last_ch_1 == 1 && !digitalRead(rec_input_1_pin)){
    rec_last_ch_1 = 0;
    rec_input_ch_1 = micros() - rec_input_ch_1_timer;
  }

  // Channel 2: Pitch
  if (rec_last_ch_2 == 0 && digitalRead(rec_input_2_pin)){
    rec_last_ch_2 = 1;
    rec_input_ch_2_timer = micros();
  }
  else if (rec_last_ch_2 == 1 && !digitalRead(rec_input_2_pin)){
    rec_last_ch_2 = 0;
    rec_input_ch_2 = micros() - rec_input_ch_2_timer;
  }

  // Channel 3: Throttle
  if (rec_last_ch_3 == 0 && digitalRead(rec_input_3_pin)){
    rec_last_ch_3 = 1;
    rec_input_ch_3_timer = micros();
  }
  else if (rec_last_ch_3 == 1 && !digitalRead(rec_input_3_pin)){
    rec_last_ch_3 = 0;
    rec_input_ch_3 = micros() - rec_input_ch_3_timer;
  }

  // Channel 4: Yaw
  if (rec_last_ch_4 == 0 && digitalRead(rec_input_4_pin)){
    rec_last_ch_4 = 1;
    rec_input_ch_4_timer = micros();
  }
  else if (rec_last_ch_4 == 1 && !digitalRead(rec_input_4_pin)){
    rec_last_ch_4 = 0;
    rec_input_ch_4 = micros() - rec_input_ch_4_timer;
  }

}

// Main (setup & loop)
void setup() {
  Serial.begin(9600);

  pinMode(rec_input_1_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rec_input_1_pin), radio_reciever_input, CHANGE);
  pinMode(rec_input_2_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rec_input_2_pin), radio_reciever_input, CHANGE);
  pinMode(rec_input_3_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rec_input_3_pin), radio_reciever_input, CHANGE);
  pinMode(rec_input_4_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rec_input_4_pin), radio_reciever_input, CHANGE);
}

void loop(){
  Serial.print(rec_input_ch_1);
  Serial.print(" ");
  Serial.print(rec_input_ch_2);
  Serial.print(" ");
  Serial.print(rec_input_ch_3);
  Serial.print(" ");
  Serial.print(rec_input_ch_4);
  Serial.print("\n");
  //digitalWrite(ledpin, state);
}
