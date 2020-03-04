/* Analog Read to LED
 * ------------------ 
 *
 * turns on and off a light emitting diode(LED) connected to digital  
 * pin 13. The amount of time the LED will be on and off depends on
 * the value obtained by analogRead(). In the easiest case we connect
 * a potentiometer to analog pin 2.
 *
 * Created 1 December 2005
 * copyleft 2005 DojoDave <http://www.0j0.org>
 * http://arduino.berlios.de
 *
 */

#include "Inputs.h"
int potPin = A2;    // select the input pin for the potentiometer
int buttPin = 2;
int ledPin = 13;   // select the pin for the LED
int val1 = 0;       // variable to store the value coming from the sensor

Potentiometer Pot1(potPin);
Button Butt1(buttPin);

void setup() {
  pinMode(ledPin, OUTPUT);  // declare the ledPin as an OUTPUT

}

void loop() {
//  val1 = Pot1.ReadVal();    // read the value from the sensor
//  digitalWrite(ledPin, HIGH);  // turn the ledPin on
//  delay(val1);                  // stop the program for some time
//  digitalWrite(ledPin, LOW);   // turn the ledPin off
//  delay(val1);                  // stop the program for some time

  if (Butt1.ReadVal() == HIGH) //Normally open, i.e. when press button, LED goes off
  {
    digitalWrite(ledPin, HIGH);
  }
  else
  {
    digitalWrite(ledPin, LOW);
  }
}
 
