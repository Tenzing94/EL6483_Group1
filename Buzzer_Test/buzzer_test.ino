//A sketch to demonstrate the tone() function

//Specify digital pin on the Arduino that the positive lead of piezo buzzer is attached.
int piezoPin = 5;

void setup() {

}

void loop() {
  
  tone(piezoPin, 5000, 250);
  //delay(750);
  
}
