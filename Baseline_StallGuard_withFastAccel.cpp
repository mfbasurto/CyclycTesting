// BASELINE CODE FOR STALLGUARD CONTROL USING ESP32
// VREF is set to 0.25V and power supply current consuption is 0.22A with 15V applied, may need to retune stallguard values because moved things a bit

#include <TMCStepper.h>
#include <FastAccelStepper.h>

// Pin definitions
#define DIR_PIN 5
#define STEP_PIN 18
#define ENABLE_PIN 26
#define RX_PIN 19
#define TX_PIN 12
#define STALLGUARD_PIN 21
// const int poxPin = 34;     // Pressure sensor pin

// Pressure sensor variables
// int poxValue = 0;
// int poxValue2 = 0;
// int poxValue3 = 0;
// int poxValue4 = 0;
// int poxValue5 = 0;
// int poxMmHg = 0; 

long circuit_time;
//long circuit_time_s;
int cycleCount = 0;

// Stepper motor settings
uint16_t motor_microsteps = 8;

//Per FastAccel Library, esp32 allows up to 200,000 generated steps/s 
int32_t set_accel = 1000*100;  // steps/s^2  
//int32_t set_current = 180; 
int32_t set_velocity = 1000;  // steps/s

int32_t set_tcools = 454; // SG_Result can be 0-510, so set_cools can be 0 - 255
int32_t set_stall = 30;  // Stall detection threshold, 0 - 255


// Flags and objects
bool stalled_motor = false;
bool motor_moving = false; 

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
TMC2209Stepper driver(&Serial1, 0.11f, 0);

// Interrupt function for stall detection
void IRAM_ATTR stalled_position() {
  stalled_motor = true;  // Set flag when a stall is detected
}

// Setup function
void setup() {
  circuit_time = millis();
  // Initialize serial communications
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // ESP32 serial communication

  // Pin modes for stepper driver
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STALLGUARD_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(STALLGUARD_PIN), stalled_position, RISING);

  // Initialize TMC2209 driver
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);

  //These were the original settings from the book 
  //driver.I_scale_analog(false);
  //driver.internal_Rsense(false);

//Setting driver to control current through Vref potentiometer
  driver.I_scale_analog(true);
  driver.internal_Rsense(false);

  driver.mstep_reg_select(true);
  driver.microsteps(motor_microsteps);
  driver.TPWMTHRS(0);
  driver.semin(0);
  driver.shaft(false);
  driver.en_spreadCycle(false);
  driver.pdn_disable(true);
  driver.VACTUAL(0);
  //driver.rms_current(set_current); 
  driver.SGTHRS(set_stall);
  driver.TCOOLTHRS(set_tcools);

  // Initialize FastAccelStepper
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(ENABLE_PIN);
  stepper->setAutoEnable(true);
  stepper->setSpeedInHz(set_velocity);
  stepper->setAcceleration(set_accel);
  delay(1000);

  // Create a task to control the motor independently
  xTaskCreatePinnedToCore(
    MotorTask,  // Motor Task
    "MotorTask",  // A name just for humans
    1024 * 4,  // Stack size
    NULL,  // Parameters
    3,  // Priority
    NULL,  // Task handle
    0);  // Core ID

  Serial.println("Setup Complete");
  delay(1000);
}

void loop() {
  while (stepper->isRunning() == true) {
  //Serial.print(driver.TSTEP());
  //Serial.print(",");
  //Serial.println(driver.SG_RESULT());
// poxValue = analogRead(poxPin);
// poxValue2 = analogRead(poxPin); 
// poxValue3 = analogRead(poxPin); 
// poxValue4 = analogRead(poxPin); 
// poxValue5 = analogRead(poxPin); 
// poxMmHg = ((poxValue + poxValue2 + poxValue3 + poxValue4 + poxValue5)/5)*0.14; //This number is for when using the esp32, this is approx number

    delay(3000);
  }
}

//Motor control task
void MotorTask(void *pvParameters) {
  while(true) {
    stepper->runForward();  // We tell the motor to move to a specific position. Use move() to move a certain distance rather than to a position.
    while (stepper->isRunning() == true) {
      if (stalled_motor == true) {
        stepper->forceStop();
        Serial.print("Stalled-CW,"); //Staled when going to open regulator
        // Serial.print(poxMmHg);
        // Serial.print(",");
        circuit_time = (millis()/1000);
        Serial.println(circuit_time);
        stalled_motor = false; // We'll set the stall flag to false, so it does not trigger this again
        delay(1000);  // Delay for testing
        break;        // Break from the while loop
      }
      delay(1);  // We need to kill some time while the motor moves
    }

    delay(1000);  // Delay 3 seconds for testing
    stepper->runBackward();

    while (stepper->isRunning() == true)  // isRunning() is true while the stepper is moving to position. We can use this time to wait for a stall
    {
      if (stalled_motor == true) {
        stepper->forceStop();
        Serial.print("Stalled-CCW,"); //stalled when going to closed regulator 
        // Serial.print(poxMmHg);
        // Serial.print(",");
        circuit_time = (millis()/1000);
        Serial.print(circuit_time);
        Serial.print(",");
        cycleCount++;
        Serial.println(cycleCount);
        stalled_motor = false; // We'll set the stall flag to false, so it does not trigger this again
        delay(1000);  // Delay for testing
        break;        // Break from the while loop
      }
      delay(1);  // We need to kill some time while the motor moves
    }

    delay(1000);  // Delay 3 seconds for testing
  }
}