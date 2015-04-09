/*
  ATtiny85 Heart earring
  Jeff Glancy
  3/29/15
  Blinks 2 (charliepexed) LED's in a heart beat pattern, sleeps, and repeats
  The BPM (beats per minute) of the heartbeat is determined by the temperature sensor
  -----
  Much of the power conservation code comes from Nick Gammon:
  http://gammon.com.au/power
  And forum discussion here:
  http://forum.arduino.cc/index.php?topic=154217.0
  Temp reading code from:
  http://andrey.mikhalchuk.com/2011/06/20/reading-attiny854525-internal-temperature-sensor.html
 */
 
/*
Copyright (c) 2015 Jeff Glancy

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
 
#include <avr/sleep.h>    // Sleep Modes
#include <avr/power.h>    // Power management
#include <avr/wdt.h>      // Watchdog timer

//pins - charlieplexed LED
// 3 - LEDs 1,2
// 4 - LEDs 1,2
short pinHigh[7] = {0,4,3};
short pinLow[7]  = {0,3,4};

// Flash LED differently based on temperature:
// use initial reading as calibration temp
int refTemp;      // degrees K
// what was our last reading
int tempDiff;  // degrees offset from refTemp

float ledFrac;  //fraction of led beat compared to 60bpm

// how many samples to take and average, more takes longer but is more 'smooth'
const int NUMSAMPLES = 5;

// Setup ADC to read core temperature sensor
void setupTemp()
{
  //analogReference( INTERNAL1V1 );
  // ATTiny85 datasheet p140 (17.13.2), p137 (17.12)
  // Configure ADMUX
  ADMUX = B1111; // Select temperature sensor
  ADMUX &= ~_BV( ADLAR ); // Right-adjust result
  ADMUX |= _BV( REFS1 ); // Set Ref voltage
  ADMUX &= ~( _BV( REFS0 ) | _BV( REFS2 ) ); // to 1.1V
} //end setupTemp()

// when ADC completed, take an interrupt 
EMPTY_INTERRUPT (ADC_vect);

// Read ATtiny85 core temperature
int getTemp()
{
  byte i;
  int average = 0;
  
  // power savings
  power_adc_enable();
  power_timer0_enable();  // Timer 0 is needed for delay() command
  ADCSRA |= _BV(ADEN); // Enable ADC
  delay (1); //wait for ref voltage to settle
  power_timer0_disable();  // Timer 0 is needed for delay() command
  
  // take N samples in a rowy 
  for (i = 0; i < NUMSAMPLES; i++) 
  {
    // ensure not interrupted before we sleep
    noInterrupts ();
    
    // start the conversion
    ADCSRA |= bit (ADSC) | bit (ADIE);
    set_sleep_mode (SLEEP_MODE_ADC);    // sleep during sample
    interrupts ();
    sleep_mode (); 

    // reading should be done, but better make sure
    // maybe the timer interrupt fired 

    // ADSC is cleared when the conversion finishes
    while (bit_is_set (ADCSRA, ADSC))
      { }
      
    average += ADCL | ( ADCH << 8 ); // Get the previous conversion result

  }  // end of for each of NUMSAMPLES
  
  // power savings
  ADCSRA = 0;            // turn off ADC
  power_adc_disable();
    
  average /= NUMSAMPLES; 
  
  return average;
}  // end getTemp()

// flash the LEDs in 1st part of heartbeat pattern
void lubb (float frac)
{
  short led = 1; //1
  power_timer0_enable();  // Timer 0 is needed for delay() command
  
  //lubb
  ledPulse(led,35*frac);
  ledPulse(led,29*frac);
  ledPulse(led,5*frac);
  ledPulse(led,8*frac);
  ledPulse(led,17*frac);
  ledPulse(led,17*frac);
  ledPulse(led,26*frac);
  ledPulse(led,32*frac);
  ledPulse(led,35*frac);

  power_timer0_disable();  // timer0 used for delay() 
}  // end lubb()

// flash the LEDs in 2nd part of heartbeat pattern
void dubb (float frac)
{
  short led = 2; //2
  power_timer0_enable();  // Timer 0 is needed for delay() command
  
  //dubb
  ledPulse(led,35*frac);
  ledPulse(led,29*frac);
  ledPulse(led,8*frac);
  ledPulse(led,5*frac);
  ledPulse(led,8*frac);
  ledPulse(led,17*frac);
  ledPulse(led,29*frac);
  ledPulse(led,35*frac);

  power_timer0_disable();  // timer0 used for delay() 
}  // end dubb()

// Turn on specified LED in the charlieplexed matrix for specified amount of time
void ledPulse(short led, short wait)
{
  //all pins start as inputs, no PU
  //set two pins to output
  pinMode(pinHigh[led], OUTPUT);
  pinMode(pinLow[led], OUTPUT);

  //set pin high to turn led on
  digitalWrite(pinHigh[led], HIGH);
  delay(1);
  //set high pin to low
  digitalWrite(pinHigh[led], LOW);
  delay(wait - 1);

  //set two pins back to inputs
  pinMode(pinHigh[led], INPUT);
  pinMode(pinLow[led], INPUT);
 
} // end ledPulse()


// watchdog interrupt
ISR (WDT_vect) 
{
   wdt_disable();  // disable watchdog
}  // end of WDT_vect

void SleepyTime(int TimeToSleep) {  // This will power down the MCU

  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  // Configure sleep to full power down mode
//  ADCSRA = 0;            // turn off ADC
//  power_all_disable ();  // power off ADC, Timer 0 and 1, serial interface
  noInterrupts ();       // timed sequence coming up
  
  MCUSR = 0;     // clear various "reset" flags
  WDTCR = bit (WDCE) | bit (WDE) | bit (WDIF);  // allow changes, disable reset, clear existing interrupt

  switch (TimeToSleep) 
  {
    case 16:
      WDTCR = bit (WDIE);    // set WDIE, and 16 ms delay
      break;  
    case 32:
      WDTCR = bit (WDIE) | bit (WDP0);    // set WDIE, and 32 ms delay
      break;
    case 64:
      WDTCR = bit (WDIE) | bit (WDP1);    // set WDIE, and 64 ms delay
      break;
    case 125:
      WDTCR = bit (WDIE) | bit (WDP1) | bit (WDP0);    // set WDIE, and 0.125 s delay
      break;
    case 250:
      WDTCR = bit (WDIE) | bit (WDP2);    // set WDIE, and 0.25 s delay
      break;
    case 500:
      WDTCR = bit (WDIE) | bit (WDP2) | bit (WDP0);    // set WDIE, and 0.5 s delay
      break;
    case 1000:
      WDTCR = bit (WDIE) | bit (WDP2) | bit (WDP1);    // set WDIE, and 1 s delay
      break;
    case 2000:
      WDTCR = bit (WDIE) | bit (WDP2) | bit (WDP1) | bit (WDP0);    // set WDIE, and 2 s delay
      break;
    case 4000:
      WDTCR = bit (WDIE) | bit (WDP3);    // set WDIE, and 4 s delay
      break;
    case 8000:
      WDTCR = bit (WDIE) | bit (WDP3) | bit (WDP0);    // set WDIE, and 8 s delay
      break; 
    default:
      WDTCR = bit (WDIE) | bit (WDP3);    // set WDIE, and 4 s delay
  }
  
/* Reference chart for Watchdog Timer
 --------------------------------------------
 WDP3      WDP2      WDP1      WDP0      Time
 --------------------------------------------
 0        0        0        0        16ms
 0        0        0        1        32ms
 0        0        1        0        64ms
 0        0        1        1        0.125s
 0        1        0        0        0.25s
 0        1        0        1        0.5s
 0        1        1        0        1.0s
 0        1        1        1        2.0s
 1        0        0        0        4.0s
 1        0        0        1        8.0s
 */

  wdt_reset();          // pat the dog
  sleep_enable ();       // ready to sleep
  interrupts ();         // interrupts are required now
  sleep_cpu ();          // sleep  
  sleep_disable ();      // precaution
  
//  power_all_enable ();   // power everything back on
}


// the setup routine runs once when you press reset:
void setup() 
{  
  // initialize all pins as an inputs, no pull-up.
  for (int i = 0; i <=4; i++)
  {
    pinMode(i, INPUT);
    digitalWrite(i, LOW);
  }
  
  //setup temperature readings, take initial reading
  setupTemp();
  refTemp = getTemp();
  tempDiff = 0;
  ledFrac = 1.0;

  // power savings
  power_all_disable ();  // power off ADC, Timer 0 and 1, serial interface

}


// the loop routine runs over and over again forever:
void loop()
{
  
  // 1st beat of heartbeat
  lubb(ledFrac);
    
    
  //delay between 1st and 2nd part
  if (tempDiff <= -13)
  {
    // 30 BPM
    SleepyTime(250);
    SleepyTime(32);
  }
  else if (tempDiff <= -10)
  {
    // 40 BPM
    SleepyTime(125);
    SleepyTime(64);
    SleepyTime(16);
  }
  else if (tempDiff <= -7)
  {
    // 50 BPM
    SleepyTime(125);
    SleepyTime(32);
    SleepyTime(16);
  }
  else if (tempDiff <= -4)
  {
    // 60 BPM
    SleepyTime(125);
    SleepyTime(16);
  }
  else if (tempDiff <= -1)
  {
    // 70 BPM
    SleepyTime(125);
  }
  else if (tempDiff <= 2)
  {
    // 80 BPM
    SleepyTime(64);
    SleepyTime(32);
    SleepyTime(16);
  }
  else if (tempDiff <= 5)
  {
    // 90 BPM
    SleepyTime(64);
    SleepyTime(32);
  }
  else if (tempDiff <= 8)
  {
    // 100 BPM
    SleepyTime(64);
    SleepyTime(16);
  }
  else if (tempDiff <= 11)
  {
    // 110 BPM
    SleepyTime(64);
    SleepyTime(16);
  }
  else if (tempDiff <= 14)
  {
    // 120 BPM
    SleepyTime(64);
  }
  else if (tempDiff <= 17)
  {
    // 130 BPM
    SleepyTime(64);
  }
  else // tempDiff <= 20
  {
    // 140 BPM
    SleepyTime(64);
  }
  
  
  // 2nd beat of heartbeat
  dubb(ledFrac);
  
    
  //take temp readings
  tempDiff = getTemp() - refTemp;
  
    
  //delay between heartbeats
  if (tempDiff <= -13)
  {
    // 30 BPM
    SleepyTime(500);
    SleepyTime(250);
    SleepyTime(125);
    SleepyTime(64);
    SleepyTime(32);
    SleepyTime(16);
    ledFrac = 1.25;
  }
  else if (tempDiff <= -10)
  {
    // 40 BPM
    SleepyTime(500);
    SleepyTime(250);
    SleepyTime(64);
    SleepyTime(32);
    ledFrac = 1.2;
  }
  else if (tempDiff <= -7)
  {
    // 50 BPM
    SleepyTime(500);
    SleepyTime(125);
    ledFrac = 1.1;
  }
  else if (tempDiff <= -4)
  {
    // 60 BPM
    SleepyTime(250);
    SleepyTime(125);
    SleepyTime(64);
    SleepyTime(32);
    SleepyTime(16);
    ledFrac = 1.0;
  }
  else if (tempDiff <= -1)
  {
    // 70 BPM
    SleepyTime(250);
    SleepyTime(125);
    SleepyTime(32);
    ledFrac = 0.9;
  }
  else if (tempDiff <= 2)
  {
    // 80 BPM
    SleepyTime(250);
    SleepyTime(64);
    SleepyTime(32);
    ledFrac = 0.8;
  }
  else if (tempDiff <= 5)
  {
    // 90 BPM
    SleepyTime(250);
    SleepyTime(64);
    ledFrac = 0.7;
  }
  else if (tempDiff <= 8)
  {
    // 100 BPM
    SleepyTime(250);
    SleepyTime(32);
    SleepyTime(16);
    ledFrac = 0.6;
  }
  else if (tempDiff <= 11)
  {
    // 110 BPM
    SleepyTime(250);
    SleepyTime(32);
    ledFrac = 0.5;
  }
  else if (tempDiff <= 14)
  {
    // 120 BPM
    SleepyTime(250);
    SleepyTime(32);
    ledFrac = 0.4;
  }
  else if (tempDiff <= 17)
  {
    // 130 BPM
    SleepyTime(250);
    SleepyTime(32);
    ledFrac = 0.3;
  }
  else // tempDiff <= 20
  {
    // 140 BPM
    SleepyTime(250);
    SleepyTime(16);
    ledFrac = 0.2;
  }

} // end of loop
