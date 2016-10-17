//www.elegoo.com
//2016.06.13

// Arduino pin numbers
const int P1_SW_pin = 4; // digital pin connected to switch output
const int P1_X_pin = A1; // analog pin connected to X output
const int P1_Y_pin = A0; // analog pin connected to Y output

const int P2_SW_pin = 2; // digital pin connected to switch output
const int P2_X_pin = A3; // analog pin connected to X output
const int P2_Y_pin = A2; // analog pin connected to Y output

void setup() {
  pinMode(P1_SW_pin, INPUT);
  digitalWrite(P1_SW_pin, HIGH);
  pinMode(P2_SW_pin, INPUT);
  digitalWrite(P2_SW_pin, HIGH);
  Serial.begin(9600);
}

void loop() {
  Serial.print("Player 1:  ");
  Serial.print("\n");
  Serial.print("Switch:  ");
  Serial.print(digitalRead(P1_SW_pin));
  Serial.print("\n");
  Serial.print("X-axis: ");
  Serial.print(analogRead(P1_X_pin));
  Serial.print("\n");
  Serial.print("Y-axis: ");
  Serial.println(analogRead(P1_Y_pin));
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("Player 2:  ");
  Serial.print("\n");
  Serial.print("Switch:  ");
  Serial.print(digitalRead(P2_SW_pin));
  Serial.print("\n");
  Serial.print("X-axis: ");
  Serial.print(analogRead(P2_X_pin));
  Serial.print("\n");
  Serial.print("Y-axis: ");
  Serial.println(analogRead(P2_Y_pin));
  Serial.print("\n");
  Serial.print("-------------------------");
  Serial.print("\n");
  delay(500);
}
