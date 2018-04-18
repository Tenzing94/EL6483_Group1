#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT();
#define CHANNEL A0

#define SAMPLES 64
#define SAMPLING_FREQUENCY 40000 // The sampling frequency has to be ATLEAST 2x larger than the largest signal.
#define SAMPLE_PERIOD (float) 11
#define MARGIN 1

const uint16_t samples = SAMPLES; // This value MUST ALWAYS be a power of 2
const double samplingFrequency = SAMPLING_FREQUENCY;

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[samples];
double vImag[samples];

int found_maximum = 0;
int chk = 0;
int found_direction = 0;
int turn = 0;
double prevMyReal = 0;
double max_value = 0;


double temp;

double myReal;

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
  myReal = 0;

  /*** Sample the data ****************************************************************/
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
  /*************************************************************************************************/
  

  /*** Compute the FFT *********************************************************************/
  // FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */


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
  delay(1000);
  */
  /**********************************************************************************************/

  
  myReal = vReal[8]; // Bin corresponding to 5k Hz.
  Serial.println(myReal);
  if (found_maximum == 0)
  {
    analogWrite(3,90);
    analogWrite(4,90);
    Serial.println("I'm inside the CCW");
  }
  else if (found_maximum == 1)
  {
    analogWrite(3,76);
    analogWrite(4,76);
    Serial.println("I'm inside the STOP");
  }

  if (found_maximum == 0)
  {
    if (myReal < (max_value - MARGIN))
    {
      found_maximum = 1;
    }
   else
   {
      max_value = myReal;
   }
  }
  
  //prevMyReal = myReal;
  Serial.println(max_value);
  Serial.println("//////////////////// THIS IS THE END OF THIS LOOP ////////////////////");
  delay(100);
  

}
