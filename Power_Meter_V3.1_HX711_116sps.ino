/*
Bicycle Power Meter V3 for Arduino Nano by Edward Rose 2022

Includes code based on arduino bike speedometer w serial.print() by Amanda Ghassaei 2012
http://www.instructables.com/id/Arduino-Bike-Speedometer/

 rising edge detect 612, 612/98 = 6.25kg
 code timings work with 116sps using a 16MHz external crystal on HX711 XO, XI pins
 calibration works for an ADC value of 98 per 1kg on crank
*/
#include <SPI.h>
#include <NRFLite.h> //https://github.com/dparson55/NRFLite
#include "HX711.h"   //https://github.com/bogde/HX711

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 4; //2;
const int LOADCELL_SCK_PIN = 5; //3;
int ledPin = 2;
int button = 3;
int batVolt = A7; // Battery voltage via potential divider 3.3k/ 2.7k

//Important defines
#define calibrationValue 0.0342  // x = 9.81/ADC value for 1kg, total torque = x*0.1725m(crank lenght)*2
#define cadenceDivideValue 6945  //calculate cadence 60 = (6945/116) 
#define risingEdgeValue 612      //detect rising edge 612/98(ADC value for 1kg) = 6.25kg

//storage variables
const int numReadings = 7;       // changed from 5 due to higher sample rate
long readings[numReadings];      // the readings from the HX711
int readIndex = 0;               // the index of the current reading
long total = 0;                  // the running total
long sensorValue = 0;            // the average
int avgValue = 0;
int cadence = 0;
int Power = 0;
long minValue = 0;
int prevSensorV = 0;
int counter;                     // time between one full rotation (in ms)
long totalValue = 0;
int maxReedCounter = 25;         //(for debouncing) changed from 10 due to cadence spikes
int reedCounter;
int codeTimer = 0;
int zeroError = 0;               // diffence between min value and sensorValue
int lipoPercent = 0;             // percentage left in battery calculated from voltage
volatile int  resultButton = 0;  // global value set by checkButton()
// Data to be sent by radio
struct dataStruct {
  int value1;
  int value2;
  int value3;
} myData;
dataStruct _myData;

NRFLite _radio;
HX711 scale;

void setup() {
  // initialize serial communication with computer:
  //Serial.begin(115200);
  pinMode(button, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay (500);
  digitalWrite(ledPin, LOW);
  // pinMode(batVolt, INPUT);
  attachInterrupt(digitalPinToInterrupt(button), checkButton, CHANGE);
  _radio.init(1, 9, 10); // radio id, CE pin, CSN pin
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN, 128);   // (Dout PIN, SCK PIN, gain) Channel A 128 or 64, Channel B 32.

  reedCounter = maxReedCounter;

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  // power up radio before auto zero
  myData.value1 = Power;
  myData.value2 = zeroError;
  myData.value3 = lipoPercent;

  _radio.send(0, &myData, sizeof(dataStruct)); // send _data to radio id 0

  zero();

}
void loop() {
  switch (resultButton) {
    case 0: {
        if (scale.is_ready()) {   //sample rate approx 116Hz

          total = total - readings[readIndex];        // subtract the last reading
          readings[readIndex] = (scale.read() / 100); // read from the HX711
          total = total + readings[readIndex];        // add the reading to the total
          readIndex = readIndex + 1;                  // advance to the next position in the array

          if (readIndex >= numReadings) {             // if we're at the end of the array...
            readIndex = 0;                            // ...wrap around to the beginning
          }
          sensorValue = total / numReadings;          // calculate the average

          if (sensorValue >= minValue + risingEdgeValue && prevSensorV < minValue + risingEdgeValue) { //detect rising edge 
            if (reedCounter == 0) { //min time between pulses has passed
              avgValue = (totalValue / counter); //calculate average
              cadence = (cadenceDivideValue / counter); //calculate cadence
              totalValue = 0; // reset total
              counter = 0; //reset counter
              reedCounter = maxReedCounter; //reset reedCounter
            }
          }
          else { 
            if (reedCounter > 0) { //don't let reedCounter go negative
              reedCounter -= 1; //decrement reedCounter
            }
          }
          if (counter > 174) { //if no new pulses from strain gauges, set everything to 0
            cadence = 0;
            totalValue = 0; //changed from avgValue, stops power spikes
            codeTimer ++;
          }
          else {
            totalValue = totalValue + (sensorValue - minValue);
            counter ++;
            codeTimer ++;
          }
          prevSensorV = sensorValue;

          if (codeTimer == 22) {
            digitalWrite(ledPin, HIGH);   // turn the LED on
          }
          else if (codeTimer == 27) {
            digitalWrite(ledPin, LOW);    // turn the LED off
          }
          else if (codeTimer == 49) {     // 2nd LED flash if zeroed
            if (zeroError < 10 && zeroError > -10) {
              digitalWrite(ledPin, HIGH);  // turn the LED on
            }
          }
          else if (codeTimer == 54) { // output data every 54 samples, approx 470ms if 116Hz sample rate
            Power = ((avgValue * calibrationValue * cadence) / 9.5488);  //calculate power
            zeroError = sensorValue - minValue;                 
            lipoPercent = ((analogRead(batVolt) * 0.6) - 400); // approx battery %, better done with map function
            //transmit data
            myData.value1 = Power;
            myData.value2 = zeroError;
            myData.value3 = cadence; //lipoPercent;

            _radio.send(0, &myData, sizeof(dataStruct)); // send _data to radio id 0

            digitalWrite(ledPin, LOW);    // turn the LED off

            codeTimer = 0;

            /*
              //Serial.print("E");
              Serial.print(sensorValue);
              Serial.print(",");
              Serial.print(minValue);
              Serial.print(",");
              Serial.print(zeroError);
              Serial.print(",");
              Serial.print(totalValue);
              Serial.print(",");
              Serial.print(counter);
              Serial.print(",");
              Serial.print(avgValue);
              Serial.print(",");
              Serial.print(cadence);
              Serial.print(",");
              Serial.println(Power);
            */
          }
        }
        break;
      }
    case 1: {    // short button press
        codeTimer = 0;
        zero();
        resultButton = 0;
        break;
      }
    case 2: {   // long button press
        digitalWrite(ledPin, HIGH);
        delay (500);
        pinMode(button, OUTPUT);
        digitalWrite(button, LOW);
      }
  }
}

void zero() {
  long minValueT = 0;
  while (codeTimer < 20) {
    if (scale.is_ready()) {
      minValueT = minValueT + (scale.read() / 100);
      codeTimer++;
    }
  }
  minValue = minValueT / 20;
  return minValue;
}
void checkButton() {
  /*
    This function implements software debouncing for a two-state button.
    It responds to a short press and a long press and identifies between
    the two states. Your sketch can continue processing while the button
    function is driven by pin changes.
  */

  const unsigned long LONG_DELTA = 1000ul;               // hold seconds for a long press
  const unsigned long DEBOUNCE_DELTA = 30ul;        // debounce time
  static int lastButtonStatus = HIGH;                                   // HIGH indicates the button is NOT pressed
  int buttonStatus;                                                                    // button atate Pressed/LOW; Open/HIGH
  static unsigned long longTime = 0ul, shortTime = 0ul; // future times to determine is button has been poressed a short or long time
  boolean Released = true, Transition = false;                  // various button states
  boolean timeoutShort = false, timeoutLong = false;    // flags for the state of the presses

  buttonStatus = digitalRead(button);                // read the button state on the pin "BUTTON_PIN"
  timeoutShort = (millis() > shortTime);                          // calculate the current time states for the button presses
  timeoutLong = (millis() > longTime);

  if (buttonStatus != lastButtonStatus) {                          // reset the timeouts if the button state changed
    shortTime = millis() + DEBOUNCE_DELTA;
    longTime = millis() + LONG_DELTA;
  }

  Transition = (buttonStatus != lastButtonStatus);        // has the button changed state
  Released = (Transition && (buttonStatus == HIGH)); // for input pullup circuit

  lastButtonStatus = buttonStatus;                                     // save the button status

  if ( ! Transition) {                                                                //without a transition, there's no change in input
    // if there has not been a transition, don't change the previous result
    resultButton =  0 | resultButton;
    return;
  }

  if (timeoutLong && Released) {                                      // long timeout has occurred and the button was just released
    resultButton = 2 | resultButton;       // ensure the button result reflects a long press
  } else if (timeoutShort && Released) {                          // short timeout has occurred (and not long timeout) and button was just released
    resultButton = 1 | resultButton;     // ensure the button result reflects a short press
  } else {                                                                                  // else there is no change in status, return the normal state
    resultButton = 0 | resultButton; // with no change in status, ensure no change in button status
  }
}
