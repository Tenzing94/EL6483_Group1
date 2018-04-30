
#define SIZE_OF_ARRAY 8

#define CIRCLE_DELAY 300

void setup() {
  // put your setup code here, to run once:
  analogWriteFrequency(3,50);      // set the frequency of pin 3 to 50Hz
  analogWriteFrequency(4,50);      // set the frequency of pin 4 to 50Hz
  analogWriteResolution(10);       // set the resolution to 10 bits
}

/*
void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(3,60);
  analogWrite(4,60);
  }
*/

void loop(){

  robotCW();
  delay(300);
  robotStop();
  delay(500);
}

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
  analogWrite(3,58);
  analogWrite(4,60);
}

// This function Stops the robot
void robotStop()
{
  analogWrite(3,76);
  analogWrite(4,76);
}
