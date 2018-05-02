const int trig = 5;
const int echo = 6;
long duration;
int distance;

void setup() {
  // put your setup code here, to run once:
}

void loop() {

ultrasonic();

Serial.println(distance);
}

void ultrasonic()
{
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  
  duration = pulseIn(echo,HIGH);
  //convert to cm
  distance = duration * 0.034/2;
