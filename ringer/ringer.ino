

int buzzer = 13; 
int button = 2;         

void setup() {
  pinMode(buzzer, OUTPUT);
  pinMode(button, INPUT);
  Serial.begin(9600);
}

void loop() {
  if(digitalRead(button) == LOW){
    digitalWrite(buzzer, HIGH);
    Serial.println(millis());
    
  }else{
     digitalWrite(buzzer, LOW);
  }
  
  delay(1);
}

