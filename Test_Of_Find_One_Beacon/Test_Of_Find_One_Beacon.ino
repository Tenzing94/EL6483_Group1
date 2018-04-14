/* *** FINAL PROJECT ***
 * COURSE: EL-GY 6483 "Real Time Embedded Systems"
 * SEMESTER: Spring 2018
 * GROUP: #1
 * MEMBERS: -> Deliang Wang
 *          -> John Lee
 *          -> Mohammad Abbasi
 *          -> Tenzing Rabgyal 
 */
 
#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT();
#define CHANNEL A0

#define SAMPLES 64
#define SAMPLING_FREQUENCY 40000 // The sampling frequency has to be ATLEAST 2x larger than the largest signal.
#define SAMPLE_PERIOD (float) 11
#define MARGIN 2

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
int max_value = 0;

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
  // put your main code here, to run repeatedly:
  myReal = 0;

  /*** Sample the data ****************************************************************/
  for (int i = 0; i < SAMPLES; i++)
  {
    microseconds = micros(); ////////////////////////////////////////////////////////
    
    temp = analogRead(CHANNEL); 
    vReal[i] =  ((temp * 3.3) / 1024) - 1.65;
    vImag[i] = 0;
    //delayMicroseconds(SAMPLE_PERIOD);
    while(micros() < (microseconds + sampling_period_us)){
      //empty loop
      }
  }
  /*************************************************************************************************/
  

  /*** Compute the FFT *********************************************************************/
  // FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  /*************************************************************************************************/

  /*** Print out the values of ALL BINS ************************************************************/
  /* 
  for (uint16_t k = 0; k < (SAMPLES / 2); k++)
  {
    Serial.println(k); //Bin Index
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

  /*** Print out the value of ONE BIN ************************************************************/ 
  /*
  double abscissa;
  abscissa = ((128 * 1.0 * SAMPLING_FREQUENCY) / SAMPLES);
    Serial.print(abscissa, 2);
    Serial.print("Hz: ");
    Serial.println(vReal[128], 4);
    delay(1000);
  */
 /**********************************************************************************************/

  myReal = vReal[8]; // Bin corresponding to 5k Hz.
  
  // find the direction of the frequency
  if (chk == 0)
  {
    analogWrite(3,58);
    analogWrite(4,58);
    delay(1000);
    chk = 1;
    Serial.println("1. I'm in the CHECK if statement.");
  }
  else if ((found_direction == 0) && (found_maximum == 0) && (chk == 1))
  {
    if (prevMyReal > (myReal + MARGIN))
    {
      turn = 2;
      found_direction = 1;
    }
    else if (prevMyReal < (myReal - MARGIN))
    {
      turn = 1;
      found_direction = 1;
    }
    Serial.println("2. I'm in the FOUND_DIRECTION if statement.");
  }
 

  if ((found_direction == 1) && (found_maximum == 0) && (chk == 1) && (turn != 0))
  {
    if (turn == 1)
    {
      analogWrite(3,58);
      analogWrite(4,58);
    }
    else if (turn == 2)
    {
      analogWrite(3,95);
      analogWrite(4,95);
    }
    Serial.println("3. I'm in the FINDING THE MAXIMUM if statement.");

    if(prevMyReal > (myReal+ MARGIN))
    {
      if (turn == 1)
      {
        analogWrite(3,95);
        analogWrite(4,95);
        delay(1000);
      }
      else if (turn == 2)
      {
        analogWrite(3,58);
        analogWrite(4,58);
        delay(1000);
      }
      found_maximum = 1;
      Serial.println("4. I'm in the FOUND THE MAXIMUM if statement.");
    }
  }

  if ((found_maximum == 1) && (found_direction == 1) && (chk == 1))
  { //Once the direction of the beacon is found, move straight towards it.
    analogWrite(3,60);
    analogWrite(4,94);
    Serial.println("5. I'm in the GOING TOWARDS THE LAST BEACON if statement.");
  }
  
  prevMyReal = myReal;
  Serial.println("//////////////////// THIS IS THE END OF THIS LOOP ////////////////////");
}
