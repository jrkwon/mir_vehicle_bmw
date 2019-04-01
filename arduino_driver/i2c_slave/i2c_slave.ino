//this arduino script is reading the value from the encoder after being mapped from 0 to 255 ,the overflow is zero .
//moving the motor in reverse will result decrement from 255 to zero.
//MiR LAB 2019

#include <Wire.h>

// this device number must be same as the number in the slave Wire.begin()
#define DEVICE_NUMBER 8
//the interrupt is avaliable in pin 2 and pin 3 arduino uno
#define INT0_PIN  2
#define INT1_PIN  3

#define INT0      0
#define INT1      1

#define DELAY     60

#define MAX_ENCODER_VALUE   65535

//This variable will increase or decrease depending on the rotation of encoder
int counter = 0;
byte count;
void setup() {
  Wire.begin(DEVICE_NUMBER);
  Wire.onRequest(requestEvent);
  pinMode(INT0_PIN, INPUT);           // set pin to input
  pinMode(INT1_PIN, INPUT);           // set pin to input
  digitalWrite(INT0_PIN, HIGH);
  digitalWrite(INT1_PIN, HIGH);
  attachInterrupt(INT0, readChannelA, RISING);
  attachInterrupt(INT1, readChannelB, RISING);
  
}

void loop() {
  count = map(counter, 0, MAX_ENCODER_VALUE, 0, 255);
  delay(DELAY);
}

void readChannelA() {
  if (digitalRead(INT1_PIN) == LOW) {
    counter++;
  } else {
    counter--;
  }
}

void readChannelB() {

  if (digitalRead(INT0_PIN) == LOW) {
    counter--;
  } else {
    counter++;
  }
}

void requestEvent() {
  Wire.write(count);
}
