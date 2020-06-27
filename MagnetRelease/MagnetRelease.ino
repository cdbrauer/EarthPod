#include <Encoder.h>

float kP = 0.04;
long target = 0;

Encoder myEnc(2, 3);
int input = 4;
int pwm = 5;

void setup() {
  Serial.begin(9600);
  Serial.println("Twist-release Magnet Test");

  pinMode(input, INPUT_PULLUP);
  pinMode(pwm, OUTPUT);
  digitalWrite(pwm, 0);
}

long oldPosition  = -999;

void loop() {
  long newPosition = myEnc.read();
  if(newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }

  if(newPosition < (target-50)){
    analogWrite(pwm, ((target-newPosition)*kP));
  } else {
    digitalWrite(pwm, 0);
  }

  if(!digitalRead(input)){
      target = target + 2000;
      delay(500);
  }
}
