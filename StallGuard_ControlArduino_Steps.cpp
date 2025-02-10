
//Controls Arduino - Main function is to send steps, this replaces fast accel library

const int stepPin = 4; 

unsigned int  pulsesPerStep = 8; 
int Step_Pulse_Width_us = 500;

// Setup function
void setup() {
  Serial.begin(115200);
  pinMode(stepPin, OUTPUT);
  Serial.println("Steps Set up");
  delay(1000);
}

void loop() {
    for (int i = 0; i < pulsesPerStep; i++) {
      digitalWrite(stepPin, HIGH); // Create a step pulse
      delayMicroseconds(Step_Pulse_Width_us); // Short delay for pulse width
      digitalWrite(stepPin, LOW); // Turn off the pulse
      delayMicroseconds(Step_Pulse_Width_us); // Short delay for pulse width
    }
}