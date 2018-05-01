long duration;
int distance;

void setup() {
  // put your setup code here, to run once:
  
  analogWriteFrequency(5,50000);    //Set the period of the pwm to 20us
  analogWrite(5,512);               //trigger set to be on for 10us and off for 10us
  Serial.begin(115200);

}

void loop() {
digitalWrite(5, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(5, HIGH);
delayMicroseconds(10);
digitalWrite(5, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(6, HIGH);

Serial.println(distance);
}
