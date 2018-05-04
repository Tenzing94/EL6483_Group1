#define TIME_MOVING_RIGHT_AFTER_DIR_FOUND 500

const int trigPin = 9;
const int echoPin = 10;
long duration;
int distance;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  analogWriteFrequency(3,50);      // set the frequency of pin 3 to 50Hz // *** Green Wire ***
  analogWriteFrequency(4,50);      // set the frequency of pin 4 to 50Hz // *** White Wire ***
  analogWriteResolution(10);       // set the resolution to 10 bits
}

void loop() {

  ultrasonic();
  double counter1 = millis();
  double counter2 = 0;
  while(distance > 10 )
  {
    robotForwardSlow();
    ultrasonic();
    Serial.println(distance);
    counter2 = millis();
  }
  robotStop();
  delay(100000000000000);
  
}

void ultrasonic()
{
  duration = 0;
  while (duration < 10)
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
    distance = duration*0.034/2;
  }
}

// This function moves the robot Forward
void robotForward()
{
  analogWrite(3,60);
  analogWrite(4,60);
}

void robotForwardSlow()
{
  analogWrite(3,66);
  analogWrite(4,66);
}

// This function Stops the robot
void robotStop()
{
  analogWrite(3,76);
  analogWrite(4,76);
}

