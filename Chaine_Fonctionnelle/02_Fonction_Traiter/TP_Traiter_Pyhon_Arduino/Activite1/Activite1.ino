int led = 13;
int tempo = 15;
void setup() { 
    pinMode(led, OUTPUT);     
}

void loop() {
  digitalWrite(led, HIGH);
  delay(tempo); 
  digitalWrite(led, LOW); 
  delay(tempo);
}
