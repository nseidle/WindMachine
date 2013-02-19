// Wind Machine exhibit software
// (C) WOW Children's Museum, Louisville CO
// Mike Grusin, SparkFun Electronics, 1/2011
// mgrusin@sparkfun.com

// This software runs the Wind Machine exhibit
// This exhibit allows a child to create a windmill by adding blades to a hub, and adjusting the angle of the blades
// The child can then test the windmill by activating a fan, and seeing how fast his or her windmill turns
// Instantaneous and peak values are shown, which are zeroed between children, along with a "best" value that is reset when the machine loses power

// Operation
// The child can build and adjust their windmill at any time
// The child should first press the "reset" button to clear the previous child's peak values
// The child can then press and hold the "start" button to start the fan
// As the windmill turns, the current, peak, and best RPM values will be shown on the displays.

// Functions:
// Read a tachometer input to D2
// Use an interrupt to time the delay between tachometer pulses, and calculate the current RPM
// Display the current RPM on a 7-segment display (green) and 30-segment bargraph
// Display the child's peak value on an additional 7-segment display (yellow) and as an additional point on the bargraph
// Display the best value from all children on a 7-segment display (red)
// Read a momentary pushbutton to reset the peak value for the next child
// Read a momentary pushbutton to run the fan

// Select Arduino Pro 16MHz @ 5V w/ ATmega328 to load new code

#include <avr/wdt.h> //We need watch dog for this program

#include <SPI.h> // serial-peripheral interface used by displays

// BEHAVIOR CONSTANTS

// delay for setting RPM to zero if no input
const long ZERODELAY = 30ul * 1000; // ms, Reset current and peak RPM after some amount of time. 60 seconds = 60,000ms
const long ZERODELAY_BESTRPM = 10ul * 60 * 1000; // ms, reset maxRPM after 10 minutes
const boolean PULSELEDS = true; // set true to pulse discrete LEDs when 7-segment displays update
const int LEDSTATE = HIGH; // if PULSELEDS = false, LEDSTATE is state of LEDs, HIGH or LOW
const int lowestbargraphdivisor = 3; // initial / reset bar graph divisor (e.g. for RPM/3, 30 LED bargraph has range of 90 (30 * 3))
const int fanRunTime = 15000; // ms, time fan will be on after fan button is released (stays on if held down)

// I/O pins to external hardware
const int STATUSLED = 13; // green LED on back of Arduino Pro board
const int BUTTON_RESET = 12; // reset button, aka button 1
//const int MOSI = 11; // SPI output to all displays
const int CS_CURRENT_DISPLAY = 10; // CS to 7-segment display
const int CS_PEAK_DISPLAY = 9; // CS to 7-segment display
const int CS_BEST_DISPLAY = 8; // CS to 7-segment display
const int BARGRAPH_LATCH = 7; // latch to bargraph display
const int BEST_LED = 6;
const int PEAK_LED = 5;
const int BUTTON_START = 4; // start button, aka button 2
const int CURRENT_LED = 3;
const int TACH = 2; // input from RPM sensor board
const int RELAY = A0; // output to fan relay

// global variables
unsigned int bargraphdivisor;
long relayTimer, timeLastInteraction, timeResetBestRPM; //variables connected to millis()
unsigned int currentRPM, peakRPM, bestRPM;
unsigned int currentLEDPWM, peakLEDPWM, bestLEDPWM;
boolean fanOn;
// volatiles are subject to modification by IRQ
volatile unsigned long tempRPM, interval;
volatile unsigned long timeCurrentTick, timeSinceLastTick;
volatile boolean gotint = false;

void setup()
// this subroutine runs once upon reboot
{
  wdt_reset(); //Pet the dog
  wdt_disable(); //We don't want the watchdog during init

  // set up inputs and outputs (all outputs power up LOW)
  pinMode(RELAY,OUTPUT);
  digitalWrite(RELAY,LOW); //Just to be sure, turn off fan

  pinMode(STATUSLED,OUTPUT);
  pinMode(CS_CURRENT_DISPLAY,OUTPUT);
  pinMode(CS_PEAK_DISPLAY,OUTPUT);
  pinMode(CS_BEST_DISPLAY,OUTPUT);
  pinMode(BARGRAPH_LATCH,OUTPUT);
  pinMode(CURRENT_LED,OUTPUT);
  pinMode(PEAK_LED,OUTPUT);
  pinMode(BEST_LED,OUTPUT);

  pinMode(BUTTON_START,INPUT);
  digitalWrite(BUTTON_START,HIGH); // turn on pullup

  pinMode(BUTTON_RESET,INPUT);
  digitalWrite(BUTTON_RESET,HIGH); // turn on pullup

  pinMode(TACH,INPUT);
  digitalWrite(TACH,LOW); // turn off pullup

  if (!PULSELEDS) // optional constant state for discrete LEDs
  {
    digitalWrite(CURRENT_LED,LEDSTATE);
    digitalWrite(PEAK_LED,LEDSTATE);
    digitalWrite(BEST_LED,LEDSTATE);
  }

  // set up serial peripheral interface (max speed for connected devices)
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.begin(); 

  // initialize SPI displays
  serial7segmentInit(CS_CURRENT_DISPLAY);
  serial7segmentInit(CS_PEAK_DISPLAY);
  serial7segmentInit(CS_BEST_DISPLAY);

  // initialize serial port
  Serial.begin(9600);
  delay(100);
  Serial.println("Hi!");
  Serial.println("Wind Machine Exhibit");

  // init globals (set high to test LEDs on reset, will reset to zero using resetpeak()
  currentRPM = 1000;
  peakRPM = 1000;
  bestRPM = 1000;
  currentLEDPWM = 255;
  peakLEDPWM = 255;
  bestLEDPWM = 255;
  bargraphdivisor = 34;

  timeLastInteraction = 0;
  timeSinceLastTick = 0;
  timeResetBestRPM = millis();
  
  fanOn = false;

  // reset working values to zero
  resetPeak(true);

  wdt_enable(WDTO_250MS); //Unleash the beast

  // turn on interrupts
  attachInterrupt(0,irTick,FALLING);
  interrupts();
}

void loop() // ths subroutine runs continuously after setup() ends
{
  wdt_reset(); //Pet the dog

  delay(1); // slow down this loop

  // an interrupt occurred in the previous loop, handle it now
  // the interrupt calculates tempRPM internally, copy to currentRPM so it doesn't change unexpectedly
  if (gotint)
  {
    Serial.print("I");

    gotint = false;

    currentRPM = word(tempRPM); // grab the RPM value calculated by the interrupt routine

    // Inrease the peak values if necessary and pulse LEDs
    if(currentRPM < 9999) { //Mild error checking
      if (currentRPM > peakRPM)
      {
        peakRPM = currentRPM;
        peakLEDPWM = 255;
      }
      if (currentRPM > bestRPM)
      {
        bestRPM = currentRPM;
        bestLEDPWM = 255;
      }
    }

    // pulse LED
    currentLEDPWM = 255;
    updateDisplays();

    //Machine is being used so update the timeLastInteraction variable to the current time
    timeLastInteraction = millis();
  }

  // zero currentRPM and displays if we don't get a reading in ZERODELAY ms
  // this also zeros the displays every minute (1000ms) in case they're corrupted
  if (millis() - timeLastInteraction > ZERODELAY)
  {
    Serial.println("No machine use after 1 minute - resetting vars");

    resetPeak(false);
    timeLastInteraction = millis(); //Bring the variable up to current time
  }
  //zero maxRPM and update the displays displays every ZERODELAY_MAXRPM ms
  //Only reset max if no one is using the exhibit (fan is off)
  if (fanOn == false && (millis() - timeResetBestRPM) > ZERODELAY_BESTRPM)
  {
    Serial.println("Resetting bestRPM");
    resetPeak(true);
    
    timeResetBestRPM = millis(); //Bring the variable up to current time
  }

  // reset button pressed
  if (digitalRead(BUTTON_RESET) == 0)
  {
    //Machine is being used so update the timeLastInteraction variable to the current time
    timeLastInteraction = millis();

    // reduce the displays to zero in an entertaining way
    resetPeak(false);
  }

  // fan button pressed, turn on fan and leave on for at least one second (to reduce wear)
  // (fan will stay on if button is held down)
  if (digitalRead(BUTTON_START) == 0)
  {
    Serial.println("Turning on fan!");

    //Machine is being used so update the timeLastInteraction variable to the current time
    timeLastInteraction = millis();

    relayTimer = millis();
    digitalWrite(RELAY,HIGH); //Turn on fan
    fanOn = true;
  }
  if (fanOn == true && abs(millis() - relayTimer) > fanRunTime) //Turn off after some amount of time (probably 15 seconds)
  {
    Serial.println("Turning off fan");
    digitalWrite(RELAY,LOW); //Turn off fan

    fanOn = false;
  }

  // pulse LEDs if value increases
  if (PULSELEDS)
  {
    if (currentLEDPWM > 0)
    {
      currentLEDPWM--;
      analogWrite(CURRENT_LED,currentLEDPWM);
    }
    if (peakLEDPWM > 0)
    {
      peakLEDPWM--;
      analogWrite(PEAK_LED,peakLEDPWM);
    }
    if (bestLEDPWM > 0)
    {
      bestLEDPWM--;
      analogWrite(BEST_LED,bestLEDPWM);
    }
  }

  // increase range of bargraph if necessary
  if ((peakRPM / bargraphdivisor) > 30)
    bargraphdivisor *= 2;
}

void updateDisplays() // update all displays
{
  serial7segmentWrite(currentRPM, CS_CURRENT_DISPLAY);
  serial7segmentWrite(peakRPM, CS_PEAK_DISPLAY);
  serial7segmentWrite(bestRPM, CS_BEST_DISPLAY);
  bargraphWrite(currentRPM/bargraphdivisor, peakRPM/bargraphdivisor);

  wdt_reset(); //Pet the dog
}

void irTick()
// runs asynchronously on falling edge of D2
// takes timing values and sets flag for main loop
{
  wdt_reset(); //Pet the dog

  timeCurrentTick = micros();
  // check if last > 0 (we have a previous reading) and
  //          time > last (we're not measuring across the microsecond rollover)
  if ((timeSinceLastTick > 0) && (timeCurrentTick > timeSinceLastTick))
  {
    interval = timeCurrentTick - timeSinceLastTick; //This can not be negative
    tempRPM = 60000000ul / interval;
    gotint = true;
  }
  timeSinceLastTick = timeCurrentTick;      // else skip this reading (next one will work OK)
}

void serial7segmentWrite(unsigned int number, byte sspin)
// put a 4-digit number on a serial 7-segment display
// leading zeroes blanked
// number can be between -999 and +9999 but we're doing unsigned here
{
  char string[5]; // 4 characters plus trailing null

  if(number > 9999) number = 9999;
  //  if(number < -999) number = -999;

  // convert number to a string (4 digits, no leading zeroes)
  // (to have leading zeroes, use "%04d")
  sprintf(string,"%4d",number);

  digitalWrite(sspin,LOW);
  SPI.transfer(string[0]);
  digitalWrite(sspin,HIGH);

  delay(1);

  digitalWrite(sspin,LOW);
  SPI.transfer(string[1]);
  digitalWrite(sspin,HIGH); 

  delay(1);

  digitalWrite(sspin,LOW);
  SPI.transfer(string[2]);
  digitalWrite(sspin,HIGH); 

  delay(1);

  digitalWrite(sspin,LOW);
  SPI.transfer(string[3]);
  digitalWrite(sspin,HIGH); 

  wdt_reset(); //Pet the dog
}

void resetPeak(boolean resetBest)
// reduce displays to zero in a hopefully entertaining way
{
  //Setup step down rates
  const int attackRate = 100; //Tweak this up or down to adjust speed that graph drops 
  int bestSDR = bestRPM / attackRate; 
  if(bestSDR == 0) bestSDR = 1; //Catch a fringe case where sdr is zero

  int currentSDR = currentRPM / attackRate;
  if(currentSDR == 0) currentSDR = 1;

  int peakSDR = peakRPM / attackRate;
  if(peakSDR == 0) peakSDR = 1;


  while ((peakRPM > 0) || (currentRPM > 0) || (resetBest == true && bestRPM > 0))
  {
    wdt_reset(); //Pet the dog

    if (resetBest){
      //We may be in a situation where bargraphdivsor is 1
      if (bestRPM > bestSDR){
        bestRPM -= bestSDR;
      }
      else if (bestRPM > 0) bestRPM--;
    }

    if (peakRPM > peakSDR){
      peakRPM -= peakSDR;
    }
    else if(peakRPM > 0) peakRPM--;

    if (currentRPM > currentSDR){
      currentRPM -= currentSDR;
    }
    else if (currentRPM > 0) currentRPM--;

    updateDisplays();
    delay(1);
  }

  timeCurrentTick = micros(); //Bring both these to current time to negate any interrupts we received during peak resetting
  timeSinceLastTick = 0;
  tempRPM = 0;
  gotint = false;
  
  bargraphdivisor = lowestbargraphdivisor;
}

void serial7segmentInit(byte sspin)
// initialize a SFE 7-segment serial display
// input is CS pin to display
{
  pinMode (sspin, OUTPUT);
  // turn off all decimal points
  digitalWrite(sspin,LOW);
  SPI.transfer(0x77);
  digitalWrite(sspin,HIGH); 
  digitalWrite(sspin,LOW);
  SPI.transfer(0x00);
  digitalWrite(sspin,HIGH); 
  // clear display
  digitalWrite(sspin,LOW);
  SPI.transfer(0x76);
  digitalWrite(sspin,HIGH); 
}

void bargraphWrite(int number,int peakSegment)
// light up "number" bargraph segments, plus single "peak" segment
// inputs are 0-32, but only LEDs 0-30 are connected. 0 for no LED lit
// (rewrite using long int?)
{
  // we'll send 4 bytes to the display
  // each bit of each byte corresponds to an LED being on or off
  byte b1,b2,b3,b4;
  b1 = 0;
  b2 = 0;
  b3 = 0;
  b4 = 0;

  // fill b1-b4 per input "number"
  // do this by calculating 2^number-1 = 1<<(number)-1
  // example for number 4: 2^4-1 = 15 = B00001111 (four LEDs lit)  
  if (number < 8)
  {
    b1 = (1 << number)-1;
  }
  else
  {
    if (number < 16)
    {
      b1 = 255;
      b2 = (1 << (number-8))-1;
    }
    else
    {
      if (number < 24)
      {
        b1 = 255;
        b2 = 255;
        b3 = (1 << (number-16))-1;
      }
      else
      {
        if (number < 32)
        {
          b1 = 255;
          b2 = 255;
          b3 = 255;
          b4 = (1 << (number-24))-1;
        }
        else
        {
          b1 = 255;
          b2 = 255;
          b3 = 255;
          b4 = 255;
        }
      }
    }
  }

  // overlay "peak" dot on top of b1-b4
  // do this by calculating 2^(peak-1) = 1<<(peak-1)
  // example for number 4: 2^(4-1) = 8 = B00001000 (fourth LEDs lit)  
  if (peakSegment > 0)
  {
    if (peakSegment < 9)
    {
      b1 |= (1 << (peakSegment-1));
    }
    else
    {
      if (peakSegment < 17)
      {
        b2 |= (1 << (peakSegment-9));
      }
      else
      {
        if (peakSegment < 25)
        {
          b3 |= (1 << (peakSegment-17));
        }
        else
        {
          if (peakSegment < 33)
          {
            b4 |= (1 << (peakSegment-25));
          }
        }
      }
    }
  }

  // write b1-b4 to SPI
  bargraphRawWrite(b1,b2,b3,b4);

  wdt_reset(); //Pet the dog
}

void bargraphRawWrite(byte b1, byte b2, byte b3, byte b4)
{
  digitalWrite(BARGRAPH_LATCH,LOW);
  SPI.transfer(b4);
  SPI.transfer(b3);
  SPI.transfer(b2);
  SPI.transfer(b1);
  digitalWrite(BARGRAPH_LATCH,HIGH);
}




