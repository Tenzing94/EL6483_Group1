void setup() {
  // put your setup code here, to run once:
  analogWriteFrequency(3,50);      // set the frequency of pin 3 to 50Hz
  analogWriteFrequency(4,50);      // set the frequency of pin 4 to 50Hz
  analogWriteResolution(10);       // set the resolution to 10 bits
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(3,58);
  analogWrite(4,60);
  }

