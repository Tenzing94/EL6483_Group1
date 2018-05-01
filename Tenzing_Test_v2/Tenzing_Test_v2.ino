#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT();
#define CHANNEL A0

#define SAMPLES 128
#define SAMPLING_FREQUENCY 32000 // The sampling frequency has to be ATLEAST 2x larger than the largest signal.
#define SIZE_OF_ARRAY 12 



/**************************************CIRCLE SPEED**************************************
 * Size of Array  ---  Tested Speed (NOTE: You may need to play around with this number and tune it)
 *    8           ---    320
 *   12           ---    255
 *   16           ---    225
 */
#define SIZE_OF_ARRAY 12
#define CIRCLE_DELAY_IN_MS 255  



/**************************************FREQUENCY BIN INDEX MAPPING**************************************
 * 
 * NOTE: This mapping only works when the number of samples = 128, and the number of bins = 64
 * 
 * Bin Index  ---  Frequency
 *    0       ---      0Hz
 *    1       ---    250Hz
 *    2       ---    500Hz
 *    3       ---    750Hz
 *    
 *    4       ---   1000Hz
 *    8       ---   2000Hz
 *   12       ---   3000Hz
 *   16       ---   4000Hz
 *   
 *   20       ---   5000Hz
 *   22       ---   5500Hz
 *   24       ---   6000Hz
 *   26       ---   6500Hz
 *   28       ---   7000Hz
 *   30       ---   7500Hz
 *   32       ---   8000Hz
 *   34       ---   8500Hz
 *   36       ---   9000Hz
 *   38       ---   9500Hz
 *   40       ---  10000Hz
 *    
 */
#define FREQUENCY_BIN_INDEX 30


 /*******************************************************************************************************/


const uint16_t samples = SAMPLES; // This value MUST ALWAYS be a power of 2
const double samplingFrequency = SAMPLING_FREQUENCY;
unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[samples];
double vImag[samples];

long duration;
int distance;

void setup() {
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  
  analogWriteFrequency(3,50);      // set the frequency of pin 3 to 50Hz // *** Green Wire ***
  analogWriteFrequency(4,50);      // set the frequency of pin 4 to 50Hz // *** White Wire ***
  analogWriteResolution(10);       // set the resolution to 10 bits

  analogWriteFrequency(5,50000);    //Set the period of the pwm to 20us
  analogWrite(5,512);               //trigger set to be on for 10us and off for 10us
  
}

void loop() {

/*
  sampleData();                                                   
  for (uint16_t m = 0; m < (SAMPLES / 3); m++)  // Changed the 2 to 3             
  {                                                            
    Serial.println(m); //Bin Index                               
    double abscissa;                                            
    abscissa = ((m * 1.0 * SAMPLING_FREQUENCY) / SAMPLES);       
    Serial.print(abscissa, 2);                                 
    Serial.print("Hz: ");                                    
    Serial.println(vReal[m], 4);                           
  }                                                  
  Serial.println("");                                      
  delay(500);  
*/




  double realArray[SIZE_OF_ARRAY] = {0}; // This is the array where we will put the magnitude read from each of the 8 sides
 
  delay(1000); // Solves the problem of the for loop below skipping the first iteration when the board is reset
 
  // Spin 360 to read the signals from all the sides
  for (int i = 0; i < SIZE_OF_ARRAY; i++)
  { 
    // Serial.println(i);
    robotCCW();
    delay(CIRCLE_DELAY_IN_MS);
    robotStop();  
    realArray[i] = read_Signal_One_Sec(); //Array index i will hold the largest value read during a period of 1s
  }
  delay(500);

  // Figure out which side of the signal gives the largest signal value
  double maxValue = 0; 
  int maxIndex = 0;
  for (int j = 0; j < SIZE_OF_ARRAY; j++)
  {
    if (realArray[j] >= maxValue)
    {
      maxValue = realArray[j];
      maxIndex = j;
    }
  }
  Serial.println(maxIndex);

  // Once we have found the direction of the signal, we move towards it.
  for(int k = 0; k <= maxIndex; k++)
  {
    robotCCW();
    delay(CIRCLE_DELAY_IN_MS);
    robotStop();
    delay(250);
  }

  robotStop();
  delay(250);  
  robotForward();
  delay(1000);
  robotStop();
  delay(100000000000);
  
}


/********************************** FUNCTIONS **********************************/

// This function computes the FFT
double sampleData()
{
  double myReal = 0;
  for (int l = 0; l < SAMPLES; l++)
  {
    microseconds = micros();
    
    double temp = analogRead(CHANNEL); 
    vReal[l] =  ((temp * 3.3) / 1024) - 1.65;
    vImag[l] = 0;
    while(micros() < (microseconds + sampling_period_us))
    {
      //empty loop
    }
  }
  
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  myReal = vReal[FREQUENCY_BIN_INDEX]; 
  
  return myReal;
}


// This function reads the signal for 1 second and return the largest value read
double read_Signal_One_Sec()
{
  double currentReal = 0, largestReal = 0;
  int currentTime = 0;
  currentTime = micros();
  while(micros() < (currentTime + 1250000)) // So, this while loop will run for 1 second
  {
    currentReal = sampleData();
    if (largestReal < currentReal)
    {
      largestReal = currentReal;
    }
  }
  
  return largestReal;
}

// This function moves the robot in ClockWise Direction
void robotCW()
{
  analogWrite(3,90);
  analogWrite(4,60);
}

// This function moves the robot in Counter ClockWise Direction
void robotCCW()
{
  analogWrite(3,60);
  analogWrite(4,90);
}

// This function moves the robot Forward
void robotForward()
{
  analogWrite(3,60);
  analogWrite(4,60);
}

// This function Stops the robot
void robotStop()
{
  analogWrite(3,76);
  analogWrite(4,76);
}

// This functin is to read from the ultrasonic
void ultrasonic()
{
  duration = pulseIn(6, HIGH);    //measure time that is high
  distance = duration*0.034/2;    //convert to centimeters
}

