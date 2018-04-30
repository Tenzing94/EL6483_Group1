#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT();
#define CHANNEL A0

#define SAMPLES 128
#define SAMPLING_FREQUENCY 32000 // The sampling frequency has to be ATLEAST 2x larger than the largest signal.
#define SIZE_OF_ARRAY 8
#define CIRCLE_DELAY_IN_MS 315

/**************************************FREQUENCY BIN INDEX MAPPING**************************************
 * 
 * NOTE: This mapping only works when the number of samples = 128, and the number of bins = 64
 * 
 * Bin Index  ---  Frequency
 *    4       ---   1000Hz
 *    8       ---   2000Hz
 *   12       ---   3000Hz
 *   16       ---   4000Hz
 *   20       ---   5000Hz
 *   24       ---   6000Hz
 *   28       ---   7000Hz
 *   32       ---   8000Hz
 *   36       ---   9000Hz
 *   40       ---  10000Hz
 *    
 */
 
#define FREQUENCY_BIN_INDEX 20

 /*******************************************************************************************************/


const uint16_t samples = SAMPLES; // This value MUST ALWAYS be a power of 2
const double samplingFrequency = SAMPLING_FREQUENCY;
unsigned int sampling_period_us;
unsigned long microseconds;

void setup() {
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  
  analogWriteFrequency(3,50);      // set the frequency of pin 3 to 50Hz // *** Green Wire ***
  analogWriteFrequency(4,50);      // set the frequency of pin 4 to 50Hz // *** White Wire ***
  analogWriteResolution(10);       // set the resolution to 10 bits
}

void loop() {

  double realArray[8] = {0}; // This is the array where we will put the magnitude read from each of the 8 sides
 
  delay(1000); // Solves the problem of the for loop below skipping the first iteration when the board is reset
 
  // Spin 360 to read the signals from all the sides
  for (int i = 0; i < SIZE_OF_ARRAY; i++)
  { 
    // Serial.println(i);
    robotCCW();
    delay(CIRCLE_DELAY_IN_MS);
    robotStop();  
    realArray[i] = read_Signal_One_Sec(); //Array index i will hold the largest value read during a period of 1s
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
  Serial.println(maxIndex);

  // Once we have found the direction of the signal, we move towards it.
  for(int k = 0; k <= maxIndex; k++)
  {
    robotCCW();
    delay(CIRCLE_DELAY_IN_MS);
    robotStop();
    delay(250);
  }

  robotStop();
  delay(1000);  
  robotForward();
  delay(1000);
  robotStop();
  delay(100000000000);
}


/********************************** FUNCTIONS **********************************/

// This function computes the FFT
double sampleData()
{
  double myReal = 0;
  double vReal[samples];
  double vImag[samples];
  for (int l = 0; l < SAMPLES; l++)
  {
    microseconds = micros();
    
    double temp = analogRead(CHANNEL); 
    vReal[l] =  ((temp * 3.3) / 1024) - 1.65;
    vImag[l] = 0;
    while(micros() < (microseconds + sampling_period_us))
    {
      //empty loop
    }
  }
  
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  myReal = vReal[FREQUENCY_BIN_INDEX]; 
  
  return myReal;
}


// This function reads the signal for 1 second and return the largest value read
double read_Signal_One_Sec()
{
  double currentReal = 0, largestReal = 0;
  int currentTime = 0;
  currentTime = micros();
  while(micros() < (currentTime + 1000000)) // So, this while loop will run for 1 second
  {
    currentReal = sampleData();
    if (largestReal < currentReal)
    {
      largestReal = currentReal;
    }
  }
  
  return largestReal;
}

// This function moves the robot in Counter ClockWise Direction
void robotCCW()
{
  analogWrite(3,90);
  analogWrite(4,60);
}

// This function moves the robot in ClockWise Direction
void robotCW()
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
