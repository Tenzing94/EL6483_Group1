#include "arduinoFFT.h"

#define PORTD_PCR1 (volatile int *)0x4004C004
#define ADC0_SC1A (volatile int*) 0x4003B000
#define ADC0_SC1B (volatile int*) 0x4003B004
#define ADC0_CFG1 (volatile int*) 0x4003B008
#define ADC0_CFG2 (volatile int*) 0x4003B00C
#define ADC0_SC2  (volatile int*) 0x4003B020
#define ADC0_RA   (volatile int*) 0x4003B010
#define ADC0_RB   (volatile int*) 0x4003B014
#define SAMPLES   (uint16_t) 64
#define SAMPLE_PERIOD (float) 11
#define SAMPLE_FREQUENCY (float) 1/(0.000025)

arduinoFFT FFT = arduinoFFT();

volatile int * set_PORTD_PCR1 = PORTD_PCR1;
volatile int * set_ADC0_SC1A  = ADC0_SC1A;
volatile int * set_ADC0_SC1B  = ADC0_SC1B;
volatile int * set_ADC0_CFG1  = ADC0_CFG1;
volatile int * set_ADC0_CFG2  = ADC0_CFG2;
volatile int * set_ADC0_SC2   = ADC0_SC2;
volatile int * set_ADC0_RA    = ADC0_RA;
volatile int * set_ADC0_RB    = ADC0_RB;

float conversion;
double vReal[SAMPLES], vImag[SAMPLES];

int index_i = 0;
int i, j;
double temp;
void setup() 
{
  while (!Serial);
  Serial.print("Finished Setup\n");
}

void loop()
{
  for (i = 0; i < SAMPLES; i++)
  {
    temp = analogRead(A0);
    vReal[i] =  ((temp * 3.3) / 1024) - 1.65;
    vImag[i] = 0;
    delayMicroseconds(SAMPLE_PERIOD);
  }
  
  //FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_FLT_TOP, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES); /* Compute magnitudes */

  //Serial.println("------------------------");
  for (uint16_t k = 0; k < (SAMPLES / 2); k++)
  {
    double abscissa;
    abscissa = ((k * 1.0 * SAMPLE_FREQUENCY) / SAMPLES);
    Serial.print(abscissa, 2);
    Serial.print("Hz: ");
    Serial.println(vReal[k], 4);
    //if (k % 9 == 0 /*&& k != 0*/)
    //{
     // Serial.println("");
    //}
  }
  Serial.println("");
}
