

void setup() {
  Serial.begin(9600);
}

void loop() {
  float voltage = getVoltage(A0);

  float degreeC = (voltage - 0.5) * 100.0;
  Serial.println(degreeC);
  delay(100);
}

float getVoltage(int pin){
  return analogRead(pin) * 0.004882814;
}

