#include "arduinoFFT.h"

#define SAMPLES   (uint16_t) 64
#define SAMPLE_PERIOD (float) 31.25  // 1 / 32000
#define SAMPLE_FREQUENCY (float) 32000

arduinoFFT FFT = arduinoFFT();
IntervalTimer sampler;

double vReal[SAMPLES], vImag[SAMPLES], vReal2[SAMPLES], vImag2[SAMPLES], temp;
int index_i = 0, flag = 0, setbit = 0, i, j;

void setup()
{
  Serial.begin(115200);
  Serial.print("Finished Setup\n");
  sampler.begin(sampleInput, 31.25);
}

void sampleInput()
{
  temp = analogRead(A0);
  vReal[index_i] =  ((temp * 3.3) / 1024) - 1.65;
  vImag[index_i] = 0;
  index_i++;
  if (index_i == 64)
  {
    index_i = 0;
    flag = 1;
  }
}

void loop()
{
  int j = 0;
  if (flag == 1)
  {
    noInterrupts();
    for (j = 0; j < 64; j++)
    {
      vReal2[j] = vReal[j];
      vImag2[j] = vImag[j];
    }
    interrupts();

    FFT.Compute(vReal2, vImag2, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal2, vImag2, SAMPLES);
    Serial.println("------------------------");

    delay(250);
    for (uint16_t k = 0; k < (SAMPLES / 2); k++)
    {
      double abscissa;
      abscissa = ((k * 1.0 * SAMPLE_FREQUENCY) / SAMPLES);
      Serial.print(abscissa, 2);
      Serial.print("Hz: ");
      Serial.println(vReal2[k], 4);
    }
    flag = 0;
  }
}
