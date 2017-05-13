const int tempPin = 5;

void setup() {
  Serial.begin(9600);

  pinMode(tempPin, INPUT);
}

void loop() {
  int sensor = 0;
  float voltage = 0;
  float celsius = 0;
  
  sensor = analogRead(tempPin);

  voltage = ((float)sensor * 5000) / 1024;

  celsius = (voltage - 500) / 10;

  Serial.println(celsius);
  delay(1000);
}
