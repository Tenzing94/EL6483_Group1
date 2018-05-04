/*
 * GROUP: 1
 * MEMBERS: TENZING, JOHN, MOHAMMAD, DELIANG
 */
 
#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT();
#define CHANNEL A0

#define SAMPLES 128
#define SAMPLING_FREQUENCY 32000 // The sampling frequency has to be ATLEAST 2x larger than the largest signal.
#define SIZE_OF_ARRAY 12 

#define TIME_STOPPED_RIGHT_AFTER_MOVING 250 
#define TIME_MOVING_RIGHT_AFTER_DIR_FOUND 800 //After we found direction, how long we move before looking for another direction.

#define TRUE 1
#define FALSE 0


/**************************************CIRCLE SPEED**************************************
 * Size of Array  ---  Tested Speed (NOTE: You may need to play around with this number and tune it. Change it by at most +-15)
 *    8           ---    320
 *   12           ---    255
 *   16           ---    225
 */
#define SIZE_OF_ARRAY 12
#define CIRCLE_DELAY_IN_MS 250

/**************************************FREQUENCY BIN INDEX MAPPING**************************************
 * 
 * NOTE: This mapping only works when the number of samples = 128, and the number of bins = 64
 * 
 * Bin Index  ---  Frequency
 *    0       ---      0Hz
 *    1       ---    250Hz
 *    2       ---    500Hz
 *    3       ---    750Hz
 *    
 *    4       ---   1000Hz
 *    8       ---   2000Hz
 *   12       ---   3000Hz
 *   16       ---   4000Hz
 *   
 *   20       ---   5000Hz
 *   22       ---   5500Hz
 *   24       ---   6000Hz
 *   26       ---   6500Hz
 *   28       ---   7000Hz
 *   30       ---   7500Hz
 *   32       ---   8000Hz
 *   34       ---   8500Hz
 *   36       ---   9000Hz
 *   38       ---   9500Hz
 *   40       ---  10000Hz
 *    
 */
 #define FREQ_BIN_INDEX 20 // Robot starts by detecting the 5kHz Beacon

 int frequency_bin_index = FREQ_BIN_INDEX; // This variable is incremented by 2 once we reach the current beacon

 
// Variables for FFT 
const uint16_t samples = SAMPLES; // This value MUST ALWAYS be a power of 2
const double samplingFrequency = SAMPLING_FREQUENCY;
unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[samples]; // This array will hold the magnitudes. It is updated by SampleData();
double vImag[samples];


// Variables for Ultrasonic Sensor
const int trigPin = 9;
const int echoPin = 10;
long duration;
int distance;

// Flag that is set when the Ultrasonic detects an object. 
int ultrasonicFLAG = FALSE;


/********************************** FUNCTION PROTOTYPES **********************************/

// If you only have this in void loop, it will print all the values in serial monitor.
// If you want to use this function in void loop, comment out everything else in the void loop first.
void PrintAllValues(); 

// Function that is called in order to update the 'distance' variable.
void ultrasonic(); 

// Called inside sampleData(). We have disabled it for now.
void threePointMovingAverage(); 

// Reads, Computes FFT and places the magnitudes in global variable array 'vReal'.
// This function is called inside PrintAllValues() and read_Signal_One_Sec().
void sampleData(); 

// This function calls sampleData repeatedly for 1 second.
// The values are summed together and placed in vReal.
void read_Signal_One_Sec(); 

// Scans 360 degrees and moves toward the direction of signal.
void scan_full_circle();

// Scans 5 directions and moves toward the direction of signal.
void scan_five_directions();

// Scans 3 directions and moves toward the direction of signal.
// Once we have done a full circle, move forward a bit, then 5 directions, move forward at bit,
// we have found the general direction. So now we check 3 directions everytime. This goes on
// till the Ultrasonic sensor detects the beacon.
// As we move forward closer to the beacon, we might want to make the angle of 3 points smaller.
// That is why I placed an input paramter on this function. This is so that we can decrease the angle.
void scan_three_directions(double moving_Time);

// Robot Movements
void robotCW();
void robotCCW();
void robotForward();
void robotForwardSlow(); // Robot moves forward in a slower speed compared to robotForward()
void robotStop();

/********************************** VOID SETUP **********************************/
void setup() {
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  
  analogWriteFrequency(3,50);      // set the frequency of pin 3 to 50Hz // *** Green Wire ***
  analogWriteFrequency(4,50);      // set the frequency of pin 4 to 50Hz // *** White Wire ***
  analogWriteResolution(10);       // set the resolution to 10 bits

  // For Ultrasonic Sensor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  
}


/********************************** VOID LOOP **********************************/
void loop() {
  
  //PrintAllValues();

  double counter1 = 0, counter2 = 0;


  delay(1000); // Solves the problem of the for loop below skipping the first iteration when the board is reset

  scan_full_circle(); // Find the general direction of the signal.

  // After the general direction is found, stop for a bit.
  robotStop();
  delay(TIME_STOPPED_RIGHT_AFTER_MOVING);

  ultrasonic(); // Call ultrasonic() function once. This function will update the 'distance' global variable
  counter1 = millis();
  counter2 = 0;

  // As long as we do not detect anything while we are moving 
  // AND
  // We have moved less than value specified by TIME_MOVING_RIGHT_AFTER_DIR_FOUND macro.
  while(distance > 15 && ((counter2 - counter1) < (TIME_MOVING_RIGHT_AFTER_DIR_FOUND)))
  {
    robotForwardSlow();
    ultrasonic();
    Serial.println(distance);
    counter2 = millis();
    if (distance <= 15) // If somehow we detect the beacon, set the flag.
    {
      ultrasonicFLAG = TRUE;
    }   
  }
  
  // After moving, stop for a bit.
  robotStop();
  delay(TIME_STOPPED_RIGHT_AFTER_MOVING);

  // If the flag is set in the previous while loop, this if statement is skipped.
  if (ultrasonicFLAG == FALSE)
  {
    scan_five_directions(); // Scan 5 directions. 

    // After the direction is found, stop for a bit.
    robotStop();
    delay(TIME_STOPPED_RIGHT_AFTER_MOVING);
    
    ultrasonic(); // Call ultrasonic() function once. This function will update the 'distance' global variable
    counter1 = millis();
    counter2 = 0;
    while(distance > 15 && ((counter2 - counter1) < (TIME_MOVING_RIGHT_AFTER_DIR_FOUND)))
    {
      robotForwardSlow();
      ultrasonic();
      Serial.println(distance);
      counter2 = millis();
      if (distance <= 15)
      {
        ultrasonicFLAG = TRUE; // If somehow we detect the beacon, set the flag.
      }
    }
    // After moving, stop for a bit.
    robotStop();
    delay(TIME_STOPPED_RIGHT_AFTER_MOVING);
  }

  // If the flag is set in the previous while loop, this if statement is skipped.
  while (ultrasonicFLAG == FALSE)
  {
    // Till now, we did a full circle, move a bit, and 5 directions, move a bit.
    // The Ultrasonic still did not detect anything.
    // So now we keep doing: take 3 directions, move a bit, 3 directions, move a bit,...
    // unitl the Ultrasonic detect something in front of it
    
    scan_three_directions(CIRCLE_DELAY_IN_MS);

    // After the direction is found, stop for a bit.
    robotStop();
    delay(TIME_STOPPED_RIGHT_AFTER_MOVING);

    
    ultrasonic(); // Call ultrasonic() function once. This function will update the 'distance' global variable
    counter1 = millis();
    counter2 = 0;
    while(distance > 15 && ((counter2 - counter1) < (TIME_MOVING_RIGHT_AFTER_DIR_FOUND)))
    {
      robotForwardSlow();
      ultrasonic();
      Serial.println(distance);
      counter2 = millis();
      if (distance <= 15)
      {
        ultrasonicFLAG = TRUE; // If somehow we detect the beacon, set the flag. This will break the while loop
      }
    }
  }
  robotStop();
  delay(TIME_STOPPED_RIGHT_AFTER_MOVING);

  // Once we reach this point, it means that the Ultrasonic detected the beacon. 
  // So now we have to go to the next beacon 
  ultrasonicFLAG = FALSE; // reset the flag to FALSE
  frequency_bin_index = frequency_bin_index + 2; // Increment the index to the next beacon. Eg. 5K to 5.5K
  
  
}


/********************************** FUNCTIONS **********************************/

/////////////////////////////////////////////////////////////////////////////////////////
// If you only have this in void loop, it will print all the values in serial monitor.
// If you want to use this function in void loop, comment out everything else in the void loop first.
void PrintAllValues() 
{
  int currentTime = 0;
  double vTempReal[samples] = {0};
  
  currentTime = micros();
  while(micros() < (currentTime + 1000000)) // So, this while loop will run for 1 second
  {
    sampleData();
    for (int i = 0; i < samples; i++)
    {
      vTempReal[i] += vReal[i]; ///////////
    }
  }
  for (int j = 0; j < samples; j++)
  {
    vReal[j] = vTempReal[j];
  }

  for (uint16_t m = 20; m < 41; m++)               
  {                                                            
    Serial.println(m); //Bin Index                               
    double abscissa;                                            
    abscissa = ((m * 1.0 * SAMPLING_FREQUENCY) / SAMPLES);       
    Serial.print(abscissa, 2);                                 
    Serial.print("Hz: ");                                    
    Serial.println(vReal[m], 4); 
   m++;  
  }                        
                                    
  delay(100);   
}
/////////////////////////////////////////////////////////////////////////////////////////

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

/////////////////////////////////////////////////////////////////////////////////////////
// Called inside sampleData(). We have disabled it for now.
void threePointMovingAverage()
{
  for (int i = 1; i < 63; i++)
  {
    vReal[i] = ( (vReal[i] + vReal[i - 1] + vReal[i + 1]) / 3);    
  }
}
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
// Reads, Computes FFT and places the magnitudes in global variable array 'vReal'.
// This function is called inside PrintAllValues() and read_Signal_One_Sec().
void sampleData()
{
  double temp = 0, myReal = 0;
  
  for(int i=0; i<samples; i++)
  {
      microseconds = micros();   
      //analogReadAveraging(2);
      temp = analogRead(CHANNEL);
      vReal[i] =  ((temp * 3.3) / 2048) - 1.65;
      vImag[i] = 0;
      while(micros() < (microseconds + sampling_period_us)){
        //empty loop
      }
  }

  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); 
  FFT.ComplexToMagnitude(vReal, vImag, samples); 

  //threePointMovingAverage();
  
}
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
// This function calls sampleData repeatedly for 1 second.
// The values are summed together and placed in vReal.
void read_Signal_One_Sec()
{
  int currentTime = 0;
  double vTempReal[samples] = {0};
  
  currentTime = micros();
  while(micros() < (currentTime + 1000000)) // So, this while loop will run for 1 second
  {
    sampleData(); // Update the vReal global array with recently read magnitudes
    
    for (int i = 0; i < samples; i++) // Loop over the vReal and add them to vTempReal array.
    {
      vTempReal[i] += vReal[i]; // vTempReal is will accumulate the vReal data being updated by sampleData().
    }
  }
  for (int j = 0; j < samples; j++) // After 1 second is over, move the accumulated values to vReal global array.
  {
    vReal[j] = vTempReal[j];
  }
}
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
// Scans 360 degrees and moves toward the direction of signal.
void scan_full_circle()
{
  double realArray[SIZE_OF_ARRAY] = {0}; // This is the array where we will put the mag read from the sides
  
  // Spin 360 to read the signals from all the sides
  for (int i = 0; i < SIZE_OF_ARRAY; i++)
  { 
    //Serial.println(i);
    robotCCW();
    delay(CIRCLE_DELAY_IN_MS);
    robotStop();  
    read_Signal_One_Sec(); //Array index i will hold the largest value read during a period of 1s
    realArray[i] = vReal[frequency_bin_index];
    //Serial.println(realArray[i]);
  }
  delay(500);

  // Figure out which side of the signal gives the largest signal value
  double maxValue = 0; 
  int maxIndex = 0;
  for (int j = 0; j < SIZE_OF_ARRAY; j++)
  {
    if (realArray[j] >= maxValue)
    {
      maxValue = realArray[j];
      maxIndex = j;
    }
  }
  //Serial.println(maxIndex);

  // Once we have found the direction of the signal, we move towards it.
  for(int k = 0; k <= maxIndex; k++)
  {
    robotCCW();
    delay(CIRCLE_DELAY_IN_MS);
    robotStop();
    delay(250);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
// Scans 5 directions and moves toward the direction of signal.
void scan_five_directions()
{
  delay(500);
  double realArray[5] = {0}; // This is the array where we will put the mag read from the sides
  for ( int i = 0; i < 2; i++) // Move to the correct position
  {
    robotCCW();
    delay(CIRCLE_DELAY_IN_MS);
    robotStop();
    delay(250);   
  }
  delay(500);

  for (int i = 0; i < 4; i++) 
  { 
    // Serial.println(i);
    read_Signal_One_Sec(); //Array index i will hold the largest value read during a period of 1s
    realArray[i] = vReal[frequency_bin_index];
    robotCW();
    delay(CIRCLE_DELAY_IN_MS);
    robotStop();  
  }
  
  read_Signal_One_Sec();
  realArray[4] = vReal[frequency_bin_index];
  
  // Figure out which side of the signal gives the largest signal value
  double maxValue = 0; 
  int maxIndex = 0;
  for (int j = 0; j < 5; j++)
  {
    if (realArray[j] >= maxValue)
    {
      maxValue = realArray[j];
      maxIndex = j;
    }
  }
  delay(500);

  if(maxIndex != 4)
  {
    if (maxIndex == 3)
    {
      robotCCW();
      delay(CIRCLE_DELAY_IN_MS);
      robotStop();   
    }
    if (maxIndex == 2)
    {
      for (int i = 0; i < 2; i++)
      {
        robotCCW();
        delay(CIRCLE_DELAY_IN_MS);
        robotStop();
        delay(250);
      }    
    }
    if (maxIndex == 1)
    {
      for (int i = 0; i < 3; i++)
      {
        robotCCW();
        delay(CIRCLE_DELAY_IN_MS);
        robotStop();
        delay(250);
      }    
    }
    if (maxIndex == 0)
    {
      for (int i = 0; i < 4; i++)
      {
        robotCCW();
        delay(CIRCLE_DELAY_IN_MS);
        robotStop();
        delay(250);
      }
    }
  }  
}
/////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////
// Scans 3 directions and moves toward the direction of signal.
// Once we have done a full circle, move forward a bit, then 5 directions, move forward at bit,
// we have found the general direction. So now we check 3 directions everytime. This goes on
// till the Ultrasonic sensor detects the beacon.
// As we move forward closer to the beacon, we might want to make the angle of 3 points smaller.
// That is why I placed an input paramter on this function. This is so that we can decrease the angle.
void scan_three_directions(double moving_Time)
{
  delay(500);
  double realArray[3] = {0}; // This is the array where we will put the mag read from the sides

  robotCCW();
  delay(moving_Time);
  robotStop();
  delay(500);   


  for (int i = 0; i < 2; i++)
  { 
    // Serial.println(i);  
    read_Signal_One_Sec(); //Array index i will hold the largest value read during a period of 1s
    realArray[i] = vReal[frequency_bin_index];
    robotCW();
    delay(moving_Time);
    robotStop();
  }
  read_Signal_One_Sec();
  realArray[2] = vReal[frequency_bin_index];
  
  // Figure out which side of the signal gives the largest signal value
  double maxValue = 0; 
  int maxIndex = 0;
  for (int j = 0; j < 3; j++)
  {
    if (realArray[j] >= maxValue)
    {
      maxValue = realArray[j];
      maxIndex = j;
    }
  }
  delay(500);

  if(maxIndex != 2)
  {
    if (maxIndex == 1)
    {
      robotCCW();
      delay(moving_Time);
      robotStop();   
    }
    if (maxIndex == 0)
    {
      for (int i = 0; i < 2; i++)
      {
        robotCCW();
        delay(moving_Time);
        robotStop();
        delay(250);
      }  
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////


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

// This function moves the robot Forward in a slower speed compared to robotForward()
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


