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

#include <SPI.h> // serial-peripheral interface used by displays

// BEHAVIOR CONSTANTS

// delay for setting RPM to zero if no input
const int ZERODELAY = 1000; // ms
const boolean PULSELEDS = true; // set true to pulse discrete LEDs when 7-segment displays update
const int LEDSTATE = HIGH; // if PULSELEDS = false, LEDSTATE is state of LEDs, HIGH or LOW
const int lowestbargraphdivisor = 3; // initial / reset bar graph divisor (e.g. for RPM/3, 30 LED bargraph has range of 90 (30 * 3))
const int minfantime = 15000; // ms, time fan will be on after fan button is released (stays on if held down)

// I/O pins to external hardware
const int STATUSLED = 13; // green LED on back of Arduino Pro board
const int BUTTON1 = 12; // reset button
//const int MOSI = 11; // SPI output to all displays
const int CURRENT = 10; // CS to 7-segment display
const int PEAK = 9; // CS to 7-segment display
const int BEST = 8; // CS to 7-segment display
const int BARGRAPH_LATCH = 7; // latch to bargraph display
const int BEST_LED = 6;
const int PEAK_LED = 5;
const int BUTTON2 = 4; // start button
const int CURRENT_LED = 3;
const int TACH = 2; // input from RPM sensor board
const int RELAY = A0; // output to fan relay

// global variables
unsigned int RPM, peak, best, stopped, relaytimer, bargraphdivisor;
unsigned int currentLEDPWM, peakLEDPWM, bestLEDPWM;
// volatiles are subject to modification by IRQ
volatile unsigned long tempRPM, time, last, interval;
volatile boolean gotint = false;

void setup()
// this subroutine runs once upon reboot
{
  // set up inputs and outputs (all outputs power up LOW)
  pinMode(STATUSLED,OUTPUT);
  pinMode(CURRENT,OUTPUT);
  pinMode(PEAK,OUTPUT);
  pinMode(BEST,OUTPUT);
  pinMode(BARGRAPH_LATCH,OUTPUT);
  pinMode(CURRENT_LED,OUTPUT);
  pinMode(PEAK_LED,OUTPUT);
  pinMode(BEST_LED,OUTPUT);
  pinMode(RELAY,OUTPUT);

  pinMode(BUTTON2,INPUT);
  digitalWrite(BUTTON2,HIGH); // turn on pullup

  pinMode(BUTTON1,INPUT);
  digitalWrite(BUTTON1,HIGH); // turn on pullup

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
  serial7segmentInit(CURRENT);
  serial7segmentInit(PEAK);
  serial7segmentInit(BEST);

  // initialize serial port
  Serial.begin(9600);
  delay(10);
  Serial.println("RESET");

  // init globals (set high to test LEDs on reset, will reset to zero using resetpeak()
  RPM = 1000;
  peak = 1000;
  best = 1000;
  stopped = ZERODELAY;
  last = 0;
  currentLEDPWM = 255;
  peakLEDPWM = 255;
  bestLEDPWM = 255;
  bargraphdivisor = 34;

  // reset working values to zero
  resetpeak(true);

  // turn on interrupts
  attachInterrupt(0,interrupt,FALLING);
  interrupts();
}

void loop() // ths subroutine runs continuously after setup() ends
{
  delay(1); // slow down this loop

  // an interrupt occurred in the previous loop, handle it now
  // the interrupt calculates tempRPM internally, copy to RPM so it doesn't change unexpectedly
  if (gotint)
  {
    Serial.println("I");
    gotint = false;

    RPM = word(tempRPM); // grab the RPM value calculated by the interrupt routine

    stopped = 0; // reset the stopped timer

    // reset the peak values if necessary and pulse LEDs
    if (RPM > peak)
    {
      peak = RPM;
      peakLEDPWM = 255;
    }
    if (RPM > best)
    {
      best = RPM;
      bestLEDPWM = 255;
    }

    // pulse LED
    currentLEDPWM = 255;
    update();
  }

  // zero RPM and displays if we don't get a reading in ZERODELAY ms
  // this also zeros the displays every minute (63353ms) in case they're corrupted
  if (stopped > ZERODELAY)
  {
    RPM = 0; 
    last = 0ul;
    update();
  }
  stopped++;

  // reset button pressed
  if (digitalRead(BUTTON1) == 0)
  {
    // reduce the displays to zero in an entertaining way
    resetpeak(false);
  }

  // fan button pressed, turn on fan and leave on for at least one second (to reduce wear)
  // (fan will stay on if button is held down)
  if (digitalRead(BUTTON2) == 0)
  {
    relaytimer = 1000; // ms, 1 second
  }
  if (relaytimer > 0)
  {
    relaytimer--;
    digitalWrite(RELAY,HIGH);
  }
  else
  {
    digitalWrite(RELAY,LOW);
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
  if ((peak / bargraphdivisor) > 30)
    bargraphdivisor *= 2;
}

void update() // update all displays
{
  serial7segmentWrite(RPM,CURRENT);
  serial7segmentWrite(peak,PEAK);
  serial7segmentWrite(best,BEST);
  bargraphWrite(RPM/bargraphdivisor,peak/bargraphdivisor);
}

void interrupt()
// runs asynchronously on falling edge of D2
// takes timing values and sets flag for main loop
{
  time = micros();
  // check if last > 0 (we have a previous reading) and
  //          time > last (we're not measuring across the microsecond rollover)
  if ((last > 0) && (time > last))
  {
    interval = time - last;
    tempRPM = 60000000ul / interval;
    gotint = true;
  }
  last = time;      // else skip this reading (next one will work OK)
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
}

void resetpeak(boolean resetbest)
// reduce displays to zero in a hopefully entertaining way
{
  while ((peak > 0) || (RPM > 0))
  {
    if (resetbest) if (best > 0) best--;
    if (peak > 0) peak--;
    if (RPM > 0) RPM--;
    update();
    delay(1);
  }
  last = 0ul;
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

void bargraphWrite(int number,int peak)
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
  if (peak > 0)
  {
    if (peak < 9)
    {
      b1 |= (1 << (peak-1));
    }
    else
    {
      if (peak < 17)
      {
        b2 |= (1 << (peak-9));
      }
      else
      {
        if (peak < 25)
        {
          b3 |= (1 << (peak-17));
        }
        else
        {
          if (peak < 33)
          {
            b4 |= (1 << (peak-25));
          }
        }
      }
    }
  }

  // write b1-b4 to SPI
  bargraphRawWrite(b1,b2,b3,b4);
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



