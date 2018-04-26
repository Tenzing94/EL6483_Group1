#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT();
#define CHANNEL A0

#define SAMPLES 128
#define SAMPLING_FREQUENCY 32000 // The sampling frequency has to be ATLEAST 2x larger than the largest signal.
#define SIZE_OF_ARRAY 8
#define CIRCLE_DELAY_IN_MS 330

const uint16_t samples = SAMPLES; // This value MUST ALWAYS be a power of 2
const double samplingFrequency = SAMPLING_FREQUENCY;

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[samples];
double vImag[samples];

double functionReal, largestReal;
double realArray[8]; // This is the array where we will put the magnitude read from each of the 8 sides

void setup() {
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  
  analogWriteFrequency(3,50);      // set the frequency of pin 3 to 50Hz
  analogWriteFrequency(4,50);      // set the frequency of pin 4 to 50Hz
  analogWriteResolution(10);       // set the resolution to 10 bits
}

void loop() {

  largestReal = 0, functionReal = 0; 
  realArray[8] = {0}; 
 
  /*
  sampleData();
  for (uint16_t m = 0; m < (SAMPLES / 2); m++)
  {
    Serial.println(m); //Bin Index
    double abscissa;
    abscissa = ((m * 1.0 * SAMPLING_FREQUENCY) / SAMPLES);
    Serial.print(abscissa, 2);
    Serial.print("Hz: ");
    Serial.println(vReal[m], 4);
  }
  Serial.println("");
  delay(500);
  
 */
  delay(1000); // Solves the problem of the for loop below skipping the first iteration when the microcontroller is reset
 
  // Spin 360 to read the signals from all the sides
  for (int i = 0; i < SIZE_OF_ARRAY; i++)
  { 
    Serial.println(i);
    robotCCW();
    delay(CIRCLE_DELAY_IN_MS);
    robotStop();  
    read_Signal_One_Sec();
    realArray[i] = largestReal;
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
void sampleData()
{
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
  functionReal = vReal[20]; // Corresponds to a 5K signal
}

// This function reads the signal for 1 second and return the largest value read
void read_Signal_One_Sec()
{
  largestReal = 0;
  int currentTime = 0;
  currentTime = micros();
  while(micros() < (currentTime + 1000000)) // So, this while loop will run for 1 second
  {
    sampleData();
    if (largestReal < functionReal)
    {
      largestReal = functionReal;
    }
  }
  
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
