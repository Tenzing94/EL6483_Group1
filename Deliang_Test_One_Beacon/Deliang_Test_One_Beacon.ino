/* Regarding the FFT part of this code, most of it was taken from the FFT_03 Example */
#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT();
#define CHANNEL A0

#define SAMPLES 128 // WE HAVE TO SET THIS
#define NUM_BINS 64 // Number of Bins is always half of Number of Samples
#define SAMPLING_FREQUENCY 40000 // The sampling frequency has to be ATLEAST 2x larger than the largest signal. Since professor will give us at most 20kHz, the sampling frequency is set to 40kHz.


const uint16_t samples = SAMPLES; // This value MUST ALWAYS be a power of 2
const double samplingFrequency = SAMPLING_FREQUENCY;

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[samples];
double vImag[samples];

int state = 0;
static double previousvReal = 0;


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
  double freqmyReal = 0;
  double myReal = 0;
  double freq[NUM_BINS] = {0};
  

  /*** Sample the data ****************************************************************/
  for(int i=0; i<samples; i++)
  {
      microseconds = micros();    //Overflows after around 70 minutes!

      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() < (microseconds + sampling_period_us)){
      }
  }
  /*************************************************************************************************/
  

  /*** Compute the FFT *********************************************************************/
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  /*************************************************************************************************/


  myReal = vReal[16];
  Serial.println(vReal[16], 4);
  
  /*** If myReal is larger than the previousvReal, then we have to keep turning the robot to find the with the strongest signal. ***/
  if ((myReal > previousvReal) && (state == 0))
  {
    state = 0;
  }
  else
  {
    state = 1;
    previousvReal = 0;
  }

  if (state == 0)
  {
    analogWrite(3,61);      // state 0, so we have it turning until the previous point is higher than current point
    analogWrite(4,61);
    delay(500);
  }
  else if (state == 1); // state == 1 means that we have found the direction of the signal, so we move towards it
  {
    analogWrite(3,60);
    analogWrite(4,92);
    delay(500);
  }

  previousvReal = myReal; // Assign the current maxVreal to the previousvReal so that that value can be used in the next loop
  
}
