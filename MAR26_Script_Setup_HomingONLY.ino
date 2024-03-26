#include <SPI.h>
#include <DRV8434S.h>
#include <ezButton.h>
#include <math.h>
//test
//MATYAS'S NOTES ON THINGS TO VERIFY
// Ensure that limit switch debounce time intialization is correct
// Verify use of motorID index values (does ID array start at index of 0 or 1...)
// Verify use of delayMicroseconds in runMotor function with stepPeriodsUS or with a number (1 in brendan's allhoming code, 2000 in max's 9stepper code)
// Verify use of < or <= in for loops (i.e. if a for loop index starts at 0 and runs until index<1, will it run again for 1 or just stop??)
// Add code that moves the banks to the correct positions after homing is completed (bank1 moves completely in minus limSwitch distance, bank2 movess completely out minus limSwitch distance?)
// Add code that moves the harnesses to the correct positions after homing is completed (halfway until told to go to threading position?)
// Figure out what homing means for the picking motors. Can be done with shuttle since shuttle magnetic switch is effectively a limSwitch
// Verify ability to input motorNLimSwitch vectors into homeHarness function
// ASSUMING SINGLE LIMSWITCH PIN IN BRENDAN'S CODE IS FOR COMBINING OF ALL SIGNALS: discuss whether signals should be combined or separate (I think separate for warp tensioner homing reasons)
// Discuss implementation of two buttons (CONTINUE/START BUTTON and E-STOP BUTTON) and 2 LEDS (errorLED and weavingLED)

// In this code, the 'front' of the machine is the side from which the reed is seen in front of the harness assembly. The fabric is woven on this side.
// In this code, the 'back' of the machine is the side from which the harness is seen in front of the reed assembly. The warp fibers "get shorter" on this side.

// Motor IDs Index Values
// 0-1:   Warp Tensioner Bank 1 (front) Motors (1 = Front Left || 2 = Front Right)
// 2-3:   Warp Tensioner Bank 2 (back) Motors (3 = Back Right || 4 = Back Left)
// 4:     Harness 1
// 5:     Harness 2
// 6:     Left Picking Motor
// 7:     Right Picking Motor
// 8:     Beat Up 
// 9-10:  Remaining Harnesses (10 = Third Harness || 11 = Fourth Harness)

const int numOfSteppers = 11; //number of stepper motors
const uint8_t csPins[numOfSteppers] = {28,29,30,31,32,33,34,35}; //Chip select pins for per motor, see Motor IDs Overview
const uint16_t stepPeriodsUs[numOfSteppers] = {500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500}; //Step periods per motor (ms)
const uint16_t currentLimits[numOfSteppers] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; //Current limits per motor (mA)

//Microstepping mode per motor
const DRV8434SStepMode stepModes[numOfSteppers] = {
      DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, 
      DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, 
      DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2,
      DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2};

DRV8434S steppers[numOfSteppers]; //Array to hold DRV9434S objects per motor


// Limit Switch Setup
// LimSwitches labelled in reference to motor

// Warp Tensioner LimSwitch Pins
ezButton bank1MaxLimSwitch[2] = {8, 9};   //{Motor 0 MaxLimSwitch, Motor 1 MaxLimSwitch}
ezButton bank1MinLimSwitch[2] = {55, 57};   //{Motor 0 MinLimSwitch, Motor 1 MinLimSwitch}
ezButton bank2MaxLimSwitch[2] = {58, 60};   //{Motor 2 MaxLimSwitch, Motor 3 MaxLimSwitch}
ezButton bank2MinLimSwitch[2] = {59, 61};   //{Motor 2 MinLimSwitch, Motor 3 MinLimSwitch}

// Harness 1 & 2 LimSwitch Pins
ezButton motor4LimSwitch[2] = {62, 63};     //{Motor 4 MaxLimSwitch, Motor 4 MinLimSwitch}
ezButton motor5LimSwitch[2] = {64, 65};     //{Motor 5 MaxLimSwitch, Motor 4 MinLimSwitch}

// Beat Up LimSwitch Pin
ezButton motor8LimSwitch(66);

// Harness 3 & 4 LimSwitch Pins
// ezButton motor9LimSwitch[2] = {A, B};
// ezButton motor10LimSwitch[2] = {C, D};

// Control variables per motor type
bool bank1Active = false;  //front bank (motors 0 & 1)
bool bank2Active = false;  //back bank (motors 2 & 3)
bool h1Active = false;     //h1 (motor 4)
bool h2Active = false;    //h2 (motor 5)
bool pickActive = false;   //picking (motors 6 & 7)
bool beatupActive = false; //beat up (motor 8)
bool h3Active = false;     //h3 (motor 9)
bool h4Active = false;     //h4 (motor 10)

//Machine status variables
bool weavingActive = false;
bool error = false;

//Pick counter variables
int totalPicks = 0;    //Change this to chose fabric length  
int currentPick = 1;   //Do not change, always 1

//
const int ERROR_LED_PIN = 13;   // Example pin number for the error LED
const int WEAVING_LED_PIN = 11; // Example pin number for the weaving LED



//SYSTEM POSITIONING VARIABLES/VECTORS
//Variable distance step count vector
int h1StepsVect = 0;     //h1 shed height vector
int h2StepsVect = 0;     //h2 shed height vector
int h3StepsVect = 0;     //h3 shed height vector
int h4StepsVect = 0;     //h4 shed height vector
int bank2TensioningLocationVect = 0;  //bank 2 warp tensioning distance vector

//Fixed distance step counts
int pickDistance = 0;   //steps for picking distance
int beatupDistance = 0; //steps for beat up rotation
int scoochDistance = 0; //steps for tensioner to travel 1.6 fiber diameters

//Number of increments to divide shedding motion into
int numberOfIncrements = 50;


void setup() {
  SPI.begin();
 
 // Initialization of SPI/driver functions. Detailed descriptions here -> https://pololu.github.io/drv8434s-arduino/class_d_r_v8434_s.html#ac9909297f589d0a3660535e22ec2e11f
  for (int i = 0; i < numOfSteppers; i++) {
    pinMode(csPins[i], OUTPUT);                             // Set CS pins as outputs
    digitalWrite(csPins[i], HIGH);                          // Deselect the driver
    
    delay(1);                                               // Delay to allow driver some time to power up

    steppers[i].setChipSelectPin(csPins[i]);                // Configures object to use specified CS pin
    steppers[i].resetSettings();                            // Reset driver settings to default values
    steppers[i].clearFaults();                              // Clear fault conditions latched in drivers
    steppers[i].setCurrentMilliamps(currentLimits[i]);      // Set SCALED current limits (scaling is a function of reference current, 2000 mA by default)
    steppers[i].setStepMode(stepModes[i]);                  // Initialize step mode per motor
    steppers[i].enableSPIDirection();                       // Enables SPI direction control by overriding need for DIR pin use
    steppers[i].enableSPIStep();                            // Enables SPI step control by overriding need for STEP pin use
    steppers[i].enableDriver();                             // Enables driver
  }


// Set Warp Tensioner LimSwitch Debounce Time
for (int i = 0; i < 2; i++) {
  bank1MaxLimSwitch[i].setDebounceTime(50);
}



// Set Beat Up LimSwitch Debounce Time


// Harness 3 & 4 LimSwitch Pins
//  motor10LimSwitch.setDebounceTime[2] = {50, 50};
//  motor11LimSwitch.setDebounceTime[2] = {50, 50};

  Serial.begin(9600);
  Serial.println("Setup complete, motors ready.");

  //pinMode(errorLED, OUTPUT); // Error LED
  //pinMode(weavingLED, OUTPUT); // Weaving LED
}




void loop() {
  //HOMING INSTRUCTIONS
  //Homing cannot start until a confirmation button is pressed

Backhomemin();


  // // Main control loop for weaving
  // if (currentlyWeaving && !error) {
  //   digitalWrite(WEAVING_LED_PIN, HIGH); // Turn on weaving LED
  //   weaving();
  // } else if (error) {
  //   // Handle error, maybe reset or halt the system
  //   digitalWrite(ERROR_LED_PIN, HIGH); // Turn on the error LED
  // } else {
  //   digitalWrite(WEAVING_LED_PIN, LOW); // Turn off weaving LED
  // }

  // if (picking) {
  //   digitalWrite(PICKING_LED_PIN, HIGH); // Turn on picking LED
  // } else {
  //   digitalWrite(PICKING_LED_PIN, LOW); // Turn off picking LED
  // }
}





// Definition of stepper movement functions (Front Bank / Back Bank / H1 / H2 / LeftPick / RightPick / Beat Up / H3 / H4)
// Direction control is built into the runMotor function, so steps should be positive (+) and negative (-) to indicate direction

// Bank1 Movement Function
void moveBank1 (int steps) {
  runMotor (0, steps);
  runMotor (1, steps);
  delayMicroseconds (1);
}

// Bank2 Movement Function
void moveBank2 (int steps) {
  runMotor (2, steps);
  runMotor (3, steps);
  delayMicroseconds (1);
}

// H1 Movement Function
void moveH1 (int steps) {
  runMotor (4, steps);
  delayMicroseconds (1);
}

// H2 Movement Function
void moveH2 (int steps) {
  runMotor (5, steps);
  delayMicroseconds (1);
}

// Left Picking Motor Movement Function
void moveLeftPick (int steps){
  runMotor (6, steps);
  delayMicroseconds (1);
}

// Right Picking Motor Movement Function
void moveRightPick (int steps){
  runMotor (7, steps);
  delayMicroseconds (1);
}

// Beat Up Motor Movement Function
void moveBeatUp (int steps){
  runMotor (8, steps);
  delayMicroseconds (1);
}

// H3 Movement Function
void moveH3 (int steps) {
  runMotor (9, steps);
  delayMicroseconds (1);
}

// H4 Movement Function
void moveH4 (int steps) {
  runMotor (10, steps);
  delayMicroseconds (1);
}

void Backhomemin(){ //Homemin function
  long positions[2] = {0,0}; //Create a blank 2 x 1 Matrix

  int max; //Initiate max variable
  int min; //Initiate Min variable

    Serial.println("Starting homemin()");
    int state; // Declare state variable

    do {
        state = digitalRead(8); // Read the state of the limit switch pin
      
        if (state == LOW) {
            // Limit switch is not pressed, move the motors continuously
            positions[0] = positions[0] + 1; // Add one step to motor 0
            positions[1] = positions[1] + 1; // Add one step to motor 1
            moveBank1(1);
        }
    } while (state == LOW);

    // Limit switch is pressed, stop the motors
    min = positions[0]; // Define the min position as the number of steps it took to get there
    Serial.print(min); // Testing check
    Serial.println("min done");
    delay(2000);
}

void Fronthomemin(){ //Homemin function
  long positions[2] = {0,0}; //Create a blank 2 x 1 Matrix

  int max; //Initiate max variable
  int min; //Initiate Min variable

    Serial.println("Starting homemin()");
    int state; // Declare state variable

    do {
        state = digitalRead(9); // Read the state of the limit switch pin
      
        if (state == LOW) {
            // Limit switch is not pressed, move the motors continuously
            positions[0] = positions[0] + 1; // Add one step to motor 0
            positions[1] = positions[1] + 1; // Add one step to motor 1
            moveBank2(1);
        }
    } while (state == LOW);

    // Limit switch is pressed, stop the motors
    min = positions[0]; // Define the min position as the number of steps it took to get there
    Serial.print(min); // Testing check
    Serial.println("min done");
    delay(2000);
}

void H1homemin(){ //Homemin function
  long positions[2] = {0,0}; //Create a blank 2 x 1 Matrix

  int max; //Initiate max variable
  int min; //Initiate Min variable

    Serial.println("Starting homemin()");
    int state; // Declare state variable

    do {
        state = digitalRead(10); // Read the state of the limit switch pin
      
        if (state == LOW) {
            // Limit switch is not pressed, move the motors continuously
            positions[0] = positions[0] + 1; // Add one step to motor 0
            positions[1] = positions[1] + 1; // Add one step to motor 1
            moveH1(1);
        }
    } while (state == LOW);

    // Limit switch is pressed, stop the motors
    min = positions[0]; // Define the min position as the number of steps it took to get there
    Serial.print(min); // Testing check
    Serial.println("min done");
    delay(2000);
}

void H2homemin(){ //Homemin function
  long positions[2] = {0,0}; //Create a blank 2 x 1 Matrix

  int max; //Initiate max variable
  int min; //Initiate Min variable

    Serial.println("Starting homemin()");
    int state; // Declare state variable

    do {
        state = digitalRead(11); // Read the state of the limit switch pin
      
        if (state == LOW) {
            // Limit switch is not pressed, move the motors continuously
            positions[0] = positions[0] + 1; // Add one step to motor 0
            positions[1] = positions[1] + 1; // Add one step to motor 1
            moveH2(1);
        }
    } while (state == LOW);

    // Limit switch is pressed, stop the motors
    min = positions[0]; // Define the min position as the number of steps it took to get there
    Serial.print(min); // Testing check
    Serial.println("min done");
    delay(2000);
}

void Reedhomemin(){ //Homemin function
  long positions[2] = {0,0}; //Create a blank 2 x 1 Matrix

  int max; //Initiate max variable
  int min; //Initiate Min variable

    Serial.println("Starting homemin()");
    int state; // Declare state variable

    do {
        state = digitalRead(12); // Read the state of the limit switch pin
      
        if (state == LOW) {
            // Limit switch is not pressed, move the motors continuously
            positions[0] = positions[0] + 1; // Add one step to motor 0
            positions[1] = positions[1] + 1; // Add one step to motor 1
            moveBeatUp(1);
        }
    } while (state == LOW);

    // Limit switch is pressed, stop the motors
    min = positions[0]; // Define the min position as the number of steps it took to get there
    Serial.print(min); // Testing check
    Serial.println("min done");
    delay(2000);
}



// Definition of general min/max homing functions



// Definition of stepper homing functions (bank1 / bank2 / H1 / H2 / LeftPick / RightPick / Beat Up / H3 / H4)
// Homing function for Bank 1 (Front Bank)



// FIGURE OUT HOMING FOR PICKING MECHANISM
// Idea:  Once the loom is entirely set up with a loaded shuttle/bobbin on the track, this is the last step before the weaving process can begin
//        Use the shuttle position detector component, push rack out all the way until shuttle detector signal is high, then retract by whatever known distance, repeat with other rack
void pickingHoming() {
  Serial.println("Homing for the picking mechanism is a pain. Nuts.");
}

// Master Homing Function
// void homingFunction (){
//   // Home warp tensioner banks concurrently
//   homeBank1();
//   homeBank2();
//   Serial.println("Homing of warp tensioner banks completed. Please press go button to proceed to harness homing.")
  
//   // Require button press to allow next system to begin homing
//   delay(5000)      //using delay until button exists

//   // Home harnesses concurrently
//   homeHarness(4, motor4LimSwitch);
//   homeHarness(5, motor5LimSwitch);
//   // homeHarness(9, motor9LimSwitch);
//   // homeHarness(10, motor10LimSwitch)
//   Serial.println("Homing of harnesses completed. Please press go button to proceed to reed AKA beat up mechanism homing.")

//   //Require button presss to allow next system to begin homing
//   delay(5000)      //using delay until button exists
 
//   homeBeatUp();
//   Serial.println("Homing of beat up completed")
  
//   delay(3000)      //using delay until button exists
// }






void runMotor(int stepperID, int stepsToRun) {
  if (stepperID < 0 || stepperID >= numOfSteppers) {
    Serial.println("Invalid motor ID.");
    return;
  }
  
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
    steppers[stepperID].step();
    delayMicroseconds(stepPeriodsUs[stepperID]);
  }

  delayMicroseconds(1);
}