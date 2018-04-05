void setup() {
  // put your setup code here, to run once:
  analogWriteFrequency(3,50);      // set the frequency of pin 3 to 50Hz
  analogWriteFrequency(4,50);      // set the frequency of pin 3 to 50Hz
  analogWriteResolution(10);       // set the resolution to 10 bits
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(3,700);             // set the top to 700 out of 1023 on pin 3
  for(int i=0;i <= 1022; i++)     // for loop changing the duty cycle on pin 4
  {
    analogWrite(4,i);
    delay(10);
  }
}
