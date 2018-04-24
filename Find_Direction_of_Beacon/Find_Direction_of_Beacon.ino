#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT();
#define CHANNEL A0

#define SAMPLES 64
#define SAMPLING_FREQUENCY 32000 // The sampling frequency has to be ATLEAST 2x larger than the largest signal.
#define MARGIN 3
#define TINY_MARGIN 0.1

const uint16_t samples = SAMPLES; // This value MUST ALWAYS be a power of 2
const double samplingFrequency = SAMPLING_FREQUENCY;

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[samples];
double vImag[samples];

double temp;


double firstReal, secondReal, functionReal;
double Current, Previous;

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
  firstReal = 0;
  secondReal = 0;
  Previous = 0;
  Current = 0;

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
  
  while(1)
  {
    Serial.println("Stage 2: Entered Second Stage");
    if (secondReal < (firstReal - TINY_MARGIN) )
    {
      robotCW();
    }
    else
    {
      robotCCW();
    }
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

    if (Current < Previous) // This should fail in the first loop because 
    {
      Serial.println("Stage 3: Found Direction.");
      break;
    }

    Previous = Current; 
    Serial.println(Previous);
    Current = 0;  
  }

  if (secondReal < firstReal)
   {
     robotCCW();
   }
   else
   {
     robotCW();
   }
   delay(250);
   
   robotForward();
   delay(100000000);

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
  functionReal = vReal[10];
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


