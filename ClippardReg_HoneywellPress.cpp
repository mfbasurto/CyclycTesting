/*  CLIPPARD REGULATOR AND HONEYWELL PRESSURE SENSOR  */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// Chip Select pin is tied to pin 8 on the SparkFun SD Card Shield
const int chipSelect = 8;  

//Stepper Driver pin connections
const int enablePin = 5;    // Pin to enable the motor driver
const int dirPin = 3;       // Pin to set the direction
const int stepPin = 4;      // Pin to send the step pulse

//CLIPPARD-PressureSensor
int poxPin = A5;
const float Vsupply = 5.0;  // Supply voltage in volts 
const float Pmax = 5.0;   // Maximum measurable pressure 
const float Pmin = 0.0;     // Minimum measurable pressure 
const float Vmin = 0.1 * Vsupply;  // Minimum output voltage 
int sensorValue = 0.0;
float sensorVoltage = 0.0;
float Pressure_applied = 0.0;
float poxMmHg = 0.0;

//number of steps per revolution
const int pulsesPerStep = 26; 
/*At 16, this motor is not able to move. tried 26 as the lowest number to get the motor to move which it did at a rate of ~1.53s/rotation*/ 

int speed = 450;
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
        sensorValue = analogRead(poxPin); 
        sensorVoltage = (sensorValue / 1023.0) * Vsupply;
        Pressure_applied = ((sensorVoltage - Vmin) * (Pmax - Pmin)) / (0.8 * Vsupply) + Pmin;
        poxMmHg = Pressure_applied*51.7;
        Serial.println(poxMmHg);
        if (poxMmHg > 20) {
          sensorValue = analogRead(poxPin); 
          sensorVoltage = (sensorValue / 1023.0) * Vsupply;
          Pressure_applied = ((sensorVoltage - Vmin) * (Pmax - Pmin)) / (0.8 * Vsupply) + Pmin;
          poxMmHg = Pressure_applied*51.7;
            digitalWrite(dirPin, LOW); // Set direction to counterclockwise (CLOSE)
            for (int i = 0; i < pulsesPerStep; i++) {
                digitalWrite(stepPin, HIGH); // Create a step pulse
                delayMicroseconds(speed); // Short delay for pulse width
                digitalWrite(stepPin, LOW); // Turn off the pulse
                delayMicroseconds(speed); // Short delay for pulse
            }
        } else {
            // Pressure is below 10 mmHg, stop moving the valve
            break; // Exit the loop if pressure is below threshold
        }
    }
    delay(1000);
}

void loop() {
   String dataString = "";
   File dataFile = SD.open("ClippardT1.txt", FILE_WRITE);

    if (poxMmHg <= 20) {
      while(poxMmHg <= 260){
      sensorValue = analogRead(poxPin); 
      sensorVoltage = (sensorValue / 1023.0) * Vsupply;
      Pressure_applied = ((sensorVoltage - Vmin) * (Pmax - Pmin)) / (0.8 * Vsupply) + Pmin;
      poxMmHg = Pressure_applied*51.7; 
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
    Serial.print(","); Serial.println(poxMmHg);
    dataFile.println(","); 
    dataFile.close(); 
    }

  // Check if the pressure is above the lower limit
  if (poxMmHg >= 260) {
    while(poxMmHg >= 20 ){
    sensorValue = analogRead(poxPin); 
    sensorVoltage = (sensorValue / 1023.0) * Vsupply;
    Pressure_applied = ((sensorVoltage - Vmin) * (Pmax - Pmin)) / (0.8 * Vsupply) + Pmin;
    poxMmHg = Pressure_applied*51.7;
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
    Serial.print(","); Serial.println(poxMmHg);
    Serial.print(circuit_time); dataFile.print(circuit_time);
    Serial.print(","); dataFile.println(",");
    cycleCount++; Serial.println(cycleCount); dataFile.print(cycleCount);
    dataFile.close(); 
  }
}
