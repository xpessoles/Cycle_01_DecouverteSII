int led = 13;
void setup() {                
    pinMode(led, OUTPUT);  
    Serial.begin(57600);   // Initialisation de la console série
}
void loop() {
  digitalWrite(led, HIGH);
  Serial.print("JOUR \n"); // Ecriture sur le port série
  delay(1000); 
  digitalWrite(led, LOW); 
  Serial.print("NUIT \n");
  delay(1000);
}

