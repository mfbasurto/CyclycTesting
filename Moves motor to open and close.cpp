/* MOVES MOTOR TO OPEN AND CLOSE*/


// // Define the pin connections
const int enablePin = 5;    // Pin to enable the motor driver
const int dirPin = 3;       // Pin to set the direction
const int stepPin = 4;      // Pin to send the step pulse

// // Define the number of steps per revolution
const int pulsesPerRevolution = 2000; // Adjust this for your motor
int speed = 1000; 

void setup() {
  // Set pin modes
  pinMode(enablePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  
  // Enable the motor driver
  digitalWrite(enablePin, LOW); // LOW to enable the driver

  Serial.begin(9600); // Start serial communication
}

void loop() {
  //Move the motor one revolution in one direction
  Serial.println("Moving clockwise");
  digitalWrite(dirPin, HIGH); // Set direction to clockwise OPEN 
  for (int i = 0; i < pulsesPerRevolution; i++) {
    digitalWrite(stepPin, HIGH); // Create a step pulse
    delayMicroseconds(speed); // Short delay for pulse width
    digitalWrite(stepPin, LOW); // Turn off the pulse
    delayMicroseconds(speed); // Short delay for pulse width
  }
  delay(speed); // Wait for a second

 //Move the motor one revolution in the opposite direction
  Serial.println("Moving counterclockwise");
  digitalWrite(dirPin, LOW); // Set direction to counterclockwise CLOSE
  for (int i = 0; i < pulsesPerRevolution; i++) {
    digitalWrite(stepPin, HIGH); // Create a step pulse
    delayMicroseconds(speed); // Short delay for pulse width
    digitalWrite(stepPin, LOW); // Turn off the pulse
    delayMicroseconds(speed); // Short delay for pulse width
  }
  delay(speed); // Wait for a second
 }
