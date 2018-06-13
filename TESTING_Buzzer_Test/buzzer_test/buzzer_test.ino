//A sketch to demonstrate the tone() function

//Specify digital pin on the Arduino that the positive lead of piezo buzzer is attached.
int piezoPin = 10;

void setup() {

}

void loop() {
  
  tone(piezoPin, 5500, 250);
  delay(1000);
  
}
