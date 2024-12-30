// Test

/**
 * Author Teemu Mäntykallio and Kushagra Keshari
 * Initializes the library and runs the stepper motor with AccelStepper for testing sensorless homing.
 * 
 * 
 * //Notes - link to stall guard features

https://www.reddit.com/r/arduino/comments/17gbrgi/finally_got_the_sensorless_homing_to_work_with/?rdt=43559
https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf
file:///C:/Users/mfbas/Downloads/Stepper%20Driver%20-%20TMC2209_SilentStepStick_Rev110.pdf
https://learn.watterott.com/silentstepstick/pinconfig/tmc2209/
https://github.com/KushagraK7/TMC2209_sensorless_homing_test/blob/main/TMC_AccelStepper_Sensorless_Homing.ino


//George Motor
https://catalog.orientalmotor.com/item/2-phase-unipolar-stepper-motors/shop-pkp-series-2-phase-unipolar-stepper-motors/pkp223mu09b

Own hand written notes:
TMC2209 paired wiht 0.9deg oriental motor, vref on stepper driver is 0.25V, at 24V supplied voltage, and current draw is 40-50mA

 * 
 */

#include <TMCStepper.h>

#define EN_PIN           5 // Enable
#define DIR_PIN          3 // Direction
#define STEP_PIN         4 // Step
#define STALL_PIN_X      2 // Connected to DIAG pin on the TMC2209

//#define CS_PIN           42 // Chip select
//#define SW_MOSI          66 // Software Master Out Slave In (MOSI)
//#define SW_MISO          44 // Software Master In Slave Out (MISO)
//#define SW_SCK           64 // Software Slave Clock (SCK)
#define SW_RX            1 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            2 // TMC2208/TMC2224 SoftwareSerial transmit pin

int aMin = 50;
int aMax = 9500;

//#define SERIAL_PORT Serial1 // TMC2209 HardwareSerial port

//The numbers after 'b' are determined by the state of pins MS2 and MS1 pins of
//TMC2209 respectively. 1->VCC 2->GND
//Both the driver's UART pins are connected to the same UART pins of the microcontroller.
//The distinct addresses are used to indentify each and control the parameters individually.
#define driverA_ADDRESS 0b00 //Pins MS1 and MS2 connected to GND.

//Stallguard values for each driver(0-255), higher number -> higher sensitivity.
#define STALLA_VALUE 255

#define R_SENSE 0.11f // Sense resistor value, match to your driverA

TMC2209Stepper driverA(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);
//TMC2209Stepper driverA(&Serial, RA_SENSE, driverA_ADDRESS);

constexpr uint32_t steps_per_round = 400;//Claculated for the belt and pulley system.

#include <AccelStepper.h>
AccelStepper stepperA = AccelStepper(stepperA.DRIVER, STEP_PIN, DIR_PIN);

bool startup = true; // set false after homing
bool stalled_A = false;

void stallInterruptX(){ // flag set for motor A when motor stalls
stalled_A = true;
}

void motorAHome()
{
  stepperA.move(-100*steps_per_round); // Move 100mm
  while(1)
    {
      stepperA.run();

      if(stalled_A){
        stepperA.setCurrentPosition(0);
        break; 
      }
      
    }

    digitalWrite(LED_BUILTIN, HIGH);

    delay(250);

    digitalWrite(LED_BUILTIN, LOW);

    stepperA.setMaxSpeed(10*steps_per_round);
    stepperA.setAcceleration(50*steps_per_round); // 2000mm/s^2
    stepperA.moveTo(2000);

    while(stepperA.distanceToGo())
    stepperA.run();
}


void setup() {
    
    //Serial.begin(9600);
    //while(!Serial);
    //Serial.println("Start...");
    
    Serial.begin(9600);      // HW UART driverAs

    pinMode(LED_BUILTIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(STALL_PIN_X), stallInterruptX, RISING);
  
    driverA.begin();             // Initiate pins and registeries
    driverA.rms_current(400);    // Set stepperA current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    //driverA.en_pwm_mode(1);      // Enable extremely quiet stepping
    driverA.pwm_autoscale(1);
    driverA.microsteps(16);
    driverA.TCOOLTHRS(0xFFFFF); // 20bit max
    driverA.SGTHRS(STALLA_VALUE);

    //driverA.en_pwm_mode(1);      // Enable extremely quiet stepping

    stepperA.setMaxSpeed(1.25*steps_per_round); // 100mm/s @ 80 steps/mm
    stepperA.setAcceleration(10*steps_per_round); // 2000mm/s^2
    stepperA.setEnablePin(EN_PIN);
    stepperA.setPinsInverted(false, false, true);
    stepperA.enableOutputs();

    
    motorAHome();
    
}

void loop() {
  stepperA.run();
    
}