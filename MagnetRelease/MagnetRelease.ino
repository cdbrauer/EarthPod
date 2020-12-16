#include <Encoder.h>

float kP = 0.04;
long target = 0;
const int numReadings = 300;
int loopCount = 0;

Encoder myEnc(2, 3);
int input = 4;
int pwm = 5;
int rightKnob = A0;
int leftKnob = A1;

int readIndex = 0;              // the index of the current reading

int readingsR[numReadings];      // the readings from the analog input
int totalR = 0;                  // the running total
int averageR = 0;                // the average

int readingsL[numReadings];      // the readings from the analog input
int totalL = 0;                  // the running total
int averageL = 0;                // the average

void setup() {
  Serial.begin(9600);
  Serial.println("Twist-release Magnet Test");

  pinMode(input, INPUT_PULLUP);
  pinMode(pwm, OUTPUT);
  digitalWrite(pwm, 0);
  
  // Set all readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsR[thisReading] = 0;
    readingsL[thisReading] = 0;
  }
}

long oldPosition  = -999;

void loop() {
  // Read encoder
  long newPosition = myEnc.read();

  // If encoder value changed, print the new value
  if(newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }

  // Update PWM output based on position
  if(newPosition < (target-50)){
    analogWrite(pwm, ((target-newPosition)*kP));
  } else {
    digitalWrite(pwm, 0);
  }

  // Read remote knobs
  totalR = totalR - readingsR[readIndex];
  readingsR[readIndex] = analogRead(rightKnob);
  totalR = totalR + readingsR[readIndex];
  
  totalL = totalL - readingsL[readIndex];
  readingsL[readIndex] = analogRead(leftKnob);
  totalL = totalL + readingsL[readIndex];
  
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  averageR = totalR / numReadings;
  averageL = totalL / numReadings;

  Serial.print(averageL);
  Serial.print(", ");
  Serial.println(averageR);

  // Update motor targets
  if(!digitalRead(input)){
      target = target + 2000;
      delay(500);
  }
  
  if(((loopCount % numReadings) == 0) && (averageR > 60)){
      target = target + 2000;
      delay(500);
  }

  if(((loopCount % numReadings) == 0) && (averageL > 60)){
      target = target + 100;
      delay(500);
  }

  // Increment loop counter
  loopCount++;
}
