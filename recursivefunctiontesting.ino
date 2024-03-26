#include <SPI.h>
#include <DRV8434S.h>

const uint8_t csPin = 22;
const uint16_t stepPeriodsUs = 2000; 
const uint16_t currentLimits =1500; 

int counter = 0;

int stepcount = 200;

DRV8434S stepper;

const DRV8434SStepMode stepMode = DRV8434SStepMode::MicroStep2;

void setup() {
  SPI.begin;

  pinMode(csPin, OUTPUT);                             // Set CS pins as outputs
  digitalWrite(csPin, HIGH);                          // Deselect the driver
    
  delay(1);                                               // Delay to allow driver some time to power up

  stepper.setChipSelectPin(csPin);                // Configures object to use specified CS pin
  stepper.resetSettings();                            // Reset driver settings to default values
  stepper.clearFaults();                              // Clear fault conditions latched in drivers
  stepper.setCurrentMilliamps(currentLimits);      // Set SCALED current limits (scaling is a function of reference current, 2000 mA by default)
  stepper.setStepMode(stepMode);                  // Initialize step mode per motor
  stepper.enableSPIDirection();                       // Enables SPI direction control by overriding need for DIR pin use
  stepper.enableSPIStep();                            // Enables SPI step control by overriding need for STEP pin use
  stepper.enableDriver();                             // Enables driver
}

void loop() {
  if (counter==0){
    testFunction(counter, stepcount);
  } else {
    Serial.println("nah fam");
  }
} 

void testFunction(int iter, int stepcount){
  runMotor(stepcount);
  iter++;
  if (iter<10){
    testFunction(iter, -stepcount);
  } else {
    Serial.println("test done");
  }
};

void runMotor(int stepsToRun) {  
  if (stepsToRun == 0) {
    Serial.println("No steps specified.");
    return;
  }

  if (stepsToRun > 0) {
    steppers[stepperID].setDirection(1); // Set direction to forward
  } else {
    steppers[stepperID].setDirection(0); // Set direction to reverse
    stepsToRun = -stepsToRun; // Make steps positive
  }

  for (int step = 0; step < stepsToRun; step++) {
    stepper.step();
    delayMicroseconds(stepPeriodsUs);
  }

  delayMicroseconds(1);
}