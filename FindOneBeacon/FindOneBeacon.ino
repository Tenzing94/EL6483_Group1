/* Regarding the FFT part of this code, most of it was taken from the FFT_03 Example */
#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT();
#define CHANNEL A0

const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 40000;

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[samples];
double vImag[samples];

int state = 0;
double previousvReal = 0;

void setup() {
  // put your setup code here, to run once:
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);

  analogWriteFrequency(3,50);      // set the frequency of pin 3 to 50Hz
  analogWriteFrequency(4,50);      // set the frequency of pin 4 to 50Hz
  analogWriteResolution(10);       // set the resolution to 10 bits
}

void loop() {
  // put your main code here, to run repeatedly:
  double freqmaxvReal = 0;
  double maxvReal = 0;
  double freq[] = 0;
  

  /* Sample the data */
  for(int i=0; i<samples; i++)
  {
      microseconds = micros();    //Overflows after around 70 minutes!

      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() < (microseconds + sampling_period_us)){
      }
  }

  /* Compute the FFT */
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */

  /* Print the magnitudes, and then put the abscissa (which is the frequency) in the freq array */
  Serial.println("Computed magnitudes:");
  freq[] = PrintVector(vReal, (samples >> 1) );

  /* The magnitudes in the bins with frequency less than 1000 are very high. We want to ignore these values. */
  for (int i = 0; freq[i] < 1000; i++)
  {
    vReal[i] = 0;
  }

  /* In this for loop, we are looking for the vReal element with the largest magnitude. */
  for(int j = 0; j < sizeof(freq); j++)
  {
    if (vReal[j]>maxvReal) // If we come across a magnitude that is larger than all the previous, do the following:
    {
      maxvReal = vReal[j]; // Assign that magnitude to the maxvReal variable
      freqmaxvReal = freq[j]; // Assign the frequency that is assoiciated with that magnitude to freqvmazReal variable.
    }
  }

  /* If maxvReal is larger than the previousvReal, then we have to keep turning the robot to find the with the strongest signal. 
  */
  if (maxvReal > previousvReal) && (state == 0)
  {
    state = 0;
  }
  else
  {
    state = 1;
    previousReal = 0;
  }

  if (state == 0)
  {
    analogWrite(3,76);      // state 0, so we have it turning until the previous point is higher than current point
    analogWrite(4,96);
  }
  elseif (state == 1); // state == 1 means that we have found the direction of the signal, so we move towards it
  {
    analogWrite(3,45);
    analogWrite(4,96);
  }

  previousvReal = maxVreal; // Assign the current maxVreal to the previousvReal so that that value can be used in the next loop
  
}

/* Printing the frequencies (abscissa) and the magnitudes associated with it. */
double PrintVector(double *vData, uint16_t bufferSize)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    abscissa[i] = ((i * 1.0 * samplingFrequency) / samples);
    
    Serial.print(abscissa, 6);
    Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();

  return abscissa;
}
