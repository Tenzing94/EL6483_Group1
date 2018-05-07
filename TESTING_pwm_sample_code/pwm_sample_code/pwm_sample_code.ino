
#define SIZE_OF_ARRAY 8

#define CIRCLE_DELAY 300

#define TIME_MOVING_RIGHT_AFTER_DIR_FOUND 10000000000

// Variables for Ultrasonic Sensor
const int trigPin = 9;
const int echoPin = 10;
long duration;
int distance;

#define TRUE 1
#define FALSE 0

// Flag that is set when the Ultrasonic detects an object. 
int ultrasonicFLAG = FALSE;

void setup() {
  // put your setup code here, to run once:
  analogWriteFrequency(3,50);      // set the frequency of pin 3 to 50Hz
  analogWriteFrequency(4,50);      // set the frequency of pin 4 to 50Hz
  analogWriteResolution(10);       // set the resolution to 10 bits

    // For Ultrasonic Sensor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

void loop(){
  unsigned long counter1 = 0, counter2 = 0;
  ultrasonic(); // Call ultrasonic() function once. This function will update the 'distance' global variable
  counter1 = millis();
  counter2 = millis();

  // As long as we do not detect anything while we are moving 
  // AND
  // We have moved less than value specified by TIME_MOVING_RIGHT_AFTER_DIR_FOUND macro.
  while(distance > 15 && ((counter2 - counter1) < (TIME_MOVING_RIGHT_AFTER_DIR_FOUND)))
  {
    robotForward();
    ultrasonic();
    Serial.println(distance);
    counter2 = millis();
    
    if (distance <= 15) // If somehow we detect the beacon, set the flag.
    {
      ultrasonicFLAG = TRUE;
    }   
    
  }
  robotStop();
  delay(100000000000);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Function that is called in order to update the 'distance' variable.
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
/////////////////////////////////////////////////////////////////////////////////////////

/*
void loop(){

  robotForward();
  delay(3000);
  robotStop();
  delay(1000);
  robotForward();
  delay(10000000000000000);

}
*/

// This function moves the robot in ClockWise Direction
void robotCW()
{
  analogWrite(3,90);
  analogWrite(4,60);
}

// This function moves the robot in Counter ClockWise Direction
void robotCCW()
{
  analogWrite(3,60);
  analogWrite(4,90);
}

// This function moves the robot Forward
void robotForward()
{
  analogWrite(3,60);
  analogWrite(4,60);
}

// This function Stops the robot
void robotStop()
{
  analogWrite(3,76);
  analogWrite(4,76);
}
