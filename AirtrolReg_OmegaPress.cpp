/*  AIRTROL REGULATOR AND OMEGA PRESSURE SENSOR  */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// Chip Select pin is tied to pin 8 on the SparkFun SD Card Shield
const int chipSelect = 8;  

//Stepper Driver pin connections
const int enablePin = 5;    // Pin to enable the motor driver
const int dirPin = 3;       // Pin to set the direction
const int stepPin = 4;      // Pin to send the step pulse

//OMEGA-PressureSensor
int poxPin = A5;
int poxValue = 0;
int poxMmHg = 0; 

//number of steps per revolution
const int pulsesPerStep = 8; 
/*At 16, this motor moves at ~3.25s/rotation */ 
int speed = 500;
long circuit_time; 

int cycleCount = 0;

void setup() {

circuit_time = millis();

 pinMode(chipSelect, OUTPUT);

  // Pin modes for stepper driver
  pinMode(enablePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  
  // Enable the motor driver
  digitalWrite(enablePin, LOW); // LOW to enable the driver

  Serial.begin(9600); // Start serial communication
  Serial.println("card initialized.");
while (true) {
        poxValue = analogRead(poxPin); 
        poxMmHg = poxValue / 1.3; 
        //Serial.println("|poxMmHg, "); 
        Serial.println(poxMmHg);
        if (poxMmHg > 20) {
          poxValue = analogRead(poxPin); 
          poxMmHg = poxValue / 1.3; 
            digitalWrite(dirPin, LOW); // Set direction to counterclockwise (CLOSE)
            for (int i = 0; i < pulsesPerStep; i++) {
                digitalWrite(stepPin, HIGH); // Create a step pulse
                delayMicroseconds(speed); // Short delay for pulse width
                digitalWrite(stepPin, LOW); // Turn off the pulse
                delayMicroseconds(speed); // Short delay for pulse
            }
        } else {
            // Pressure is below 20 mmHg, stop moving the valve
            break; // Exit the loop if pressure is below threshold
        }
    }

    delay(1000);
}

void loop() {
  String dataString = "";
  File dataFile = SD.open("AirtrolT1.txt", FILE_WRITE);

    if (poxMmHg <= 20) {
      while(poxMmHg <= 260){
      poxValue = analogRead(poxPin); 
      poxMmHg = poxValue/1.3; 
    // Move the motor to increase pressure
    digitalWrite(dirPin, HIGH); // Set direction to clockwise
    for (int i = 0; i < pulsesPerStep; i++) {
      digitalWrite(stepPin, HIGH); // Create a step pulse
      delayMicroseconds(speed); // Short delay for pulse width
      digitalWrite(stepPin, LOW); // Turn off the pulse
      delayMicroseconds(speed); // Short delay for pulse width
        }
      }
    circuit_time = millis();
    Serial.print("Opened,"); dataFile.print("Opened,");
    Serial.print(circuit_time); dataFile.print(circuit_time);
    Serial.println(","); dataFile.println(",");
    dataFile.close();
    }

  // Check if the pressure is above the lower limit
  if (poxMmHg >= 260) {
    while(poxMmHg >= 20 ){
    poxValue = analogRead(poxPin); 
      poxMmHg = poxValue/1.3; 
    // Move the motor to decrease pressure
    digitalWrite(dirPin, LOW); // Set direction to counterclockwise
    for (int i = 0; i < pulsesPerStep; i++) {
      digitalWrite(stepPin, HIGH); // Create a step pulse
      delayMicroseconds(speed); // Short delay for pulse width //SHOULD BE 500 microseconds on and off
      digitalWrite(stepPin, LOW); // Turn off the pulse
      delayMicroseconds(speed); // Short delay for pulse width
      }
    }
    circuit_time = millis();
    Serial.print("Closed,"); dataFile.print("Closed,");
    Serial.print(circuit_time); dataFile.print(circuit_time);
    Serial.print(","); dataFile.println(",");
    cycleCount++; Serial.println(cycleCount); dataFile.print(cycleCount);
    dataFile.close();
  }
}
