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
  analogWriteFrequency(4,50);      // set the frequency of pin 3 to 50Hz
  analogWriteResolution(10);       // set the resolution to 10 bits
}

void loop() {
  // put your main code here, to run repeatedly:
  double freqmaxvReal = 0;
  double maxvReal = 0;
  double freq[] = 0;
  

  // Sample the data
  for(int i=0; i<samples; i++)
  {
      microseconds = micros();    //Overflows after around 70 minutes!

      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() < (microseconds + sampling_period_us)){
      }
  }

  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */

  Serial.println("Computed magnitudes:");
  freq[] = PrintVector(vReal, (samples >> 1) );

  for (int i = 0; freq[i] < 1000; i++)
  {
    vReal[i] = 0;
  }

  for(int j = 0; j < sizeof(freq); j++)
  {
    if (vReal[j]>maxvReal)
    {
      maxvReal = vReal[j];
      freqmaxvReal = freq[j];
    }
  }

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
    analogWrite(3,76);      // state 0 we have it turning until the previous point is higher than current point
    analogWrite(4,96);
  }
  elseif (state == 1);
  {
    analogWrite(3,45);
    analogWrite(4,96);
  }

  previousvReal = maxVreal;
  
}

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

  return abscissa
}
