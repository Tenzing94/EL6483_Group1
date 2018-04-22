#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT();
#define CHANNEL A0

#define SAMPLES 64
#define SAMPLING_FREQUENCY 40000 // The sampling frequency has to be ATLEAST 2x larger than the largest signal.

#define SIZEOFARRAY 8

const uint16_t samples = SAMPLES; // This value MUST ALWAYS be a power of 2
const double samplingFrequency = SAMPLING_FREQUENCY;

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[samples];
double vImag[samples];

double temp;


double functionReal, largestReal;
double firstReal[8], secondReal[8];
double Current, Previous;

double maxIndex, maxValue;

void setup() {
// put your setup code here, to run once
  //sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);

  sampling_period_us = round(1000000*(1.0/samplingFrequency)); ///////////////////////////////////////////
  
  analogWriteFrequency(3,50);      // set the frequency of pin 3 to 50Hz
  analogWriteFrequency(4,50);      // set the frequency of pin 4 to 50Hz
  analogWriteResolution(10);       // set the resolution to 10 bits

}

void loop() {

  largestReal = 0, functionReal = 0;

  maxIndex = 0, maxValue = 0;
  
  firstReal[8] = {0}; 
  secondReal[8] = {0};

  Previous = 0;
  Current = 1;

  int currentTime = 0;
  int foundDirection = 0;

  sampleData();
 
  /*
  for (uint16_t k = 0; k < (SAMPLES / 2); k++)
  {
    //Serial.println(k); //Bin Index
    double abscissa;
    abscissa = ((k * 1.0 * SAMPLING_FREQUENCY) / SAMPLES);
    Serial.print(abscissa, 2);
    Serial.print("Hz: ");
    Serial.println(vReal[k], 4);
  }
  Serial.println("");
  delay(500);
  */
  
  firstReal[0] = functionReal; 

  // Spin 360 to read the signals from all the sides
  for (int i = 1; i < SIZEOFARRAY; i++)
  {
    robotCCW();
    delay(300);
    robotStop();
    read_Signal_One_Sec();
    firstReal[i] = largestReal;   
  }

  // Figure out which side of the signal gives the largest signal value
  maxValue = firstReal[0];
  maxIndex = 0;
  for (int i = 1; i < SIZEOFARRAY; i++)
  {
    if (firstReal[i] > maxValue)
    {
      //maxValue = firstReal[i];
      maxIndex = i;
    }
  }

  // Go to the direction of the side with largest signal value
  robotCCW();
  delay(300*maxIndex);
  robotForward();
  delay(100000);  
}


/********************************** LIST OF FUNCTIONS **********************************/

// This function computes the FFT
void sampleData()
{
    for (int i = 0; i < SAMPLES; i++)
  {
    microseconds = micros();
    
    temp = analogRead(CHANNEL); 
    vReal[i] =  ((temp * 3.3) / 1024) - 1.65;
    vImag[i] = 0;
    while(micros() < (microseconds + sampling_period_us))
    {
      //empty loop
    }
  }
  
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  functionReal = vReal[8];
}

// This function reads the signal for 1 second and return the largest value read
void read_Signal_One_Sec()
{
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
