/*
    ATtiny Water Alarm
    Copyright (C) 2019 Toby Boreham.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/sleep.h>            // Needed for sleep_mode
#include <avr/wdt.h>              // Needed to enable/disable watch dog timer
#include <avr/power.h>
#include <SoftwareSerial.h>

#define RX    0                   // *** Not used
#define TX    4                   // *** D4, Pin 3
#define ALARM_THRESHOLD 300       // ADC value that triggers an alarm

#define CYCLE_DELAY 100           // time to pulse LED/piezo at end of loop, if necessary
#define LOW_BATTERY_MULTIPLE 8    // This is the number of sleep cycles between low battery beeps
#define LOW_BATTERY_VOLTAGE 3.0f  // In volts

// Refer to the ATtiny datasheet for the source of these values
//0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
//6=1sec, 7=2sec, 8=4sec, 9=8sec
#define WATCHDOG_DELAY 9

// Setup software serial (the ATtiny has no hardware serial)
SoftwareSerial mySerial(RX, TX);

const uint8_t piezo = 1;
const uint8_t waterSensor = A3;
uint16_t waterValue = 0;
uint16_t vccValue = 0;
float voltage = 0;
bool alarmState = false;
uint16_t cycle = 0;

//This runs each time the watch dog wakes us up from sleep
ISR(WDT_vect) {
  //Don't do anything. This is just here so that we wake up.
}

void setup()
{
  mySerial.begin(9600);
  mySerial.print("Initialize... ");
  pinMode(piezo, OUTPUT);
  digitalWrite(piezo, HIGH);
  delay(100);
  digitalWrite(piezo, LOW);
  mySerial.println("Complete");

  //Power down various bits of hardware to lower power usage  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //Power down everything, wake up from WDT
  sleep_enable();
}

void loop()
{
  if(!alarmState)
  {
    ADCSRA &= ~(1<<ADEN); //Disable ADC, saves ~230uA
    setup_watchdog(WATCHDOG_DELAY); //Setup watchdog to go off after 1sec
    mySerial.print("Sleep... ");
    sleep_mode(); //Go to sleep! Wake up 1sec later and check water
    mySerial.print("Wake. ");
  }
  mySerial.print("Loop... ");
  ADCSRA |= (1 << ADEN); // Enable ADC
  waterValue = analogRead(waterSensor);
  mySerial.print("[VIN: ");
  vccValue = readVcc();
  mySerial.print(vccValue);
  voltage = (float) vccValue / 1000;
  mySerial.print(" (");
  mySerial.print(voltage);
  mySerial.print("V), ADC: ");
  mySerial.print(waterValue);
  mySerial.print(" ");
  mySerial.print("Alarm: ");
  
  if(waterValue > ALARM_THRESHOLD) {
    mySerial.print("On");
    alarmState = true;
      digitalWrite(piezo, alarmState);
      delay(CYCLE_DELAY);
      digitalWrite(piezo, LOW);
  }
  else {
    mySerial.print("Off");
    alarmState = false;
  }
  mySerial.println("] End");

  if(voltage < LOW_BATTERY_VOLTAGE) {
    cycle++;
    //mySerial.println(cycle);
    if(cycle == LOW_BATTERY_MULTIPLE) {
      cycle = 0;
      digitalWrite(piezo, HIGH);
      delay(CYCLE_DELAY);
      digitalWrite(piezo, LOW);    
    }
  }
  else {
    cycle = 0;
  }
}

// Get internal voltage
// https://forum.arduino.cc/index.php?topic=222847.0
uint16_t readVcc(void) 
{
  uint16_t result;
  // Read 1.1V reference against Vcc
  ADMUX = (0<<REFS0) | (12<<MUX0);
  delay(2); // Wait for Vref to settle
  ADCSRA |= (1<<ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCW;
  return 1125300L / result; // Back-calculate AVcc in mV
}

//Sets the watchdog timer to wake us up, but not reset
//0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
//6=1sec, 7=2sec, 8=4sec, 9=8sec
//From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
void setup_watchdog(int timerPrescaler) {

  if (timerPrescaler > 9 ) timerPrescaler = 9; //Limit incoming amount to legal settings

  byte bb = timerPrescaler & 7; 
  if (timerPrescaler > 7) bb |= (1<<5); //Set the special 5th bit if necessary

  //This order of commands is important and cannot be combined
  MCUSR &= ~(1<<WDRF); //Clear the watch dog reset
  WDTCR |= (1<<WDCE) | (1<<WDE); //Set WD_change enable, set WD enable
  WDTCR = bb; //Set new watchdog timeout value
  WDTCR |= _BV(WDIE); //Set the interrupt enable, this will keep unit from resetting after each int
}
