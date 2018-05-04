
// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;
// defines variables
long duration;
int distance;


void check_ultrasonic()
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance= duration*0.034/2;
  // Prints the distance on the Serial Monitor
}

void setup() {
  analogWriteFrequency(3,50);      // set the frequency of pin 3 to 50Hz // left wheel
  analogWriteFrequency(4,50);      // set the frequency of pin 3 to 50Hz // right wheel
  analogWriteResolution(10);       // set the resolution to 10 bits
  
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(115200); // Starts the serial communication
}

void loop() {
  check_ultrasonic();
  if (distance < 30 && distance != 0)
  {
    Serial.println(distance);
    analogWrite(3,77);
    analogWrite(4,77);
  }
  else
  {
    analogWrite(3,65);
    analogWrite(4,65);
  }
 // Serial.print("Distance: ");
 // Serial.println(distance);
}
