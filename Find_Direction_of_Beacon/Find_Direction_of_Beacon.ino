#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT();
#define CHANNEL A0

#define SAMPLES 128
#define SAMPLING_FREQUENCY 32000 // The sampling frequency has to be ATLEAST 2x larger than the largest signal.
#define MARGIN 3

const uint16_t samples = SAMPLES; // This value MUST ALWAYS be a power of 2
const double samplingFrequency = SAMPLING_FREQUENCY;

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[samples];
double vImag[samples];

double temp;

double firstReal, secondReal, functionReal;
double Current, Previous;

int state;

void setup() {
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  
  analogWriteFrequency(3,50);      // set the frequency of pin 3 to 50Hz
  analogWriteFrequency(4,50);      // set the frequency of pin 4 to 50Hz
  analogWriteResolution(10);       // set the resolution to 10 bits

}

void loop() {
  firstReal = 0;
  secondReal = 0;
  Previous = 0;
  Current = 0;
  state = 0;

  int currentTime = 0;
  int foundDirection = 0;

  // Taking the largest magnitude found during 1000ms and storing it on firstReal variable
  currentTime = micros();
  while(micros() < (currentTime + 1000000))
  {
    sampleData();
    if (firstReal < functionReal)
    {
      firstReal = functionReal;
    }
  }

  /*** Print out the values of ALL BINS ************************************************************/
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
  
  robotCCW();
  delay(310);
  robotStop();

  // Taking the largest magnitude found during 1000ms and storing it on secondReal variable
  currentTime = micros();
  while(micros() < (currentTime + 1000000))
  {
    sampleData();
    if (secondReal < functionReal) // Position Ref Point 1: Simply finding the largest value within a 1000ms time
    {
      secondReal = functionReal;
    }
  }

  if (secondReal > firstReal) // Meaning we are going in the right direction
  {
    state = 1;
  }
  else if (secondReal <= firstReal) // Meaning we are going in the wrong direction
  {
    state = 2;
  }

  if (state == 1) // Keep going in the same direction
  {
    Previous = secondReal; // Because the secondReal
    while(1)
    {
      robotCCW();
      delay(310);
      robotStop();
      
      currentTime = micros();
      while(micros() < (currentTime + 1000000))
      {
        sampleData();
        if (Current < functionReal) 
        {
          Current = functionReal;
        }
      }

      if (Current < Previous) // Meaning we have gone past the largest value
      {
        break; 
      }
      Previous = Current;
      Current = 0;             
    }    
  }
  
  else if (state == 2)
  {
    Previous = secondReal; // Because the secondReal
    while(1)
    {
      robotCW();
      delay(310);
      robotStop();
      
      currentTime = micros();
      while(micros() < (currentTime + 1000000))
      {
        sampleData();
        if (Current < functionReal) 
        {
          Current = functionReal;
        }
      }

      if (Current < Previous) // Meaning we have gone past the largest value
      {
        break; 
      }
      Previous = Current;
      Current = 0;             
    }       
  }

  // Now we need to correct the robot by going back by 1
  if (state == 1)
  {
    robotCW();
    delay(310);
    robotStop();
  }
  
  else if (state == 2)
  {
    robotCCW();
    delay(310);
    robotStop();  
  }
 
  delay(1000);  
  robotForward();
  delay(1000);
  robotStop();
  delay(100000000000); 

}

void sampleData()
{
    for (int i = 0; i < SAMPLES; i++)
  {
    microseconds = micros(); ////////////////////////////////////////////////////////
    
    temp = analogRead(CHANNEL); 
    vReal[i] =  ((temp * 3.3) / 1024) - 1.65;
    vImag[i] = 0;
    while(micros() < (microseconds + sampling_period_us)){
      //empty loop
      }
  }
  
  /*** Compute the FFT *********************************************************************/
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  functionReal = vReal[20];
}

void robotCCW()
{
  analogWrite(3,90);
  analogWrite(4,60);
}

void robotCW()
{
  analogWrite(3,60);
  analogWrite(4,90);
}

void robotForward()
{
  analogWrite(3,58);
  analogWrite(4,60);
}

void robotStop()
{
  analogWrite(3,76);
  analogWrite(4,76);
}


