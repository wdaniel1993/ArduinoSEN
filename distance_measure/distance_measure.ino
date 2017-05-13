const int ledPin = 13;
const int pingPin = 7;

void setup() {
  Serial.begin(9600);
  Serial.println("Hi guys");

  pinMode(ledPin, OUTPUT);
}

void loop() {
  int cm = ping(pingPin);
  Serial.println(cm);

  triggerLed(ledPin);
}


int ping(int pingPin){
  unsigned long duration;

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);

  duration = pulseIn(pingPin, HIGH);
  return duration / 29 / 2;
}

void triggerLed(int ledPin){
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
  delay(500);
}

