const byte ledPin = 13;
const byte interruptPin = 0;
const byte enablePin = A5;
volatile byte state = LOW;
long count = 0;

void setup() {  
  pinMode(enablePin, INPUT);
  delay(5000);  
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);
  delay(3000);
  pinMode(enablePin, INPUT);
  
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);

  // Serial.begin(9600);
}

void loop() {
  digitalWrite(ledPin, state);
  // Serial.println(count);
}

void blink() {
  state = !state;
  count++;
}
