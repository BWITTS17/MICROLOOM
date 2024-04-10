#include <SPI.h>
#include <DRV8434S.h>
#include <ezButton.h>
#include <math.h>

//NIMA'S NOTES
//added picking, beat up & scooch distances
//set up full homing and weaving sequences in void loop()
  //took out weaving calling itself recursively, just rely on pick counter
  //condensed homing functions significantly, need to test the whole procedure. not sure what the "return"s will do in homeHarnesses
//NEED TO DOCUMENT DIRECTION MAPPING for harnesses, rack/pinion, and reed and check the assigned values appropriately
  //set mapping for scooch function, should double check
//shedding function needs to recognize which pick it is and invert direction appropriately --> easiest to use odd/even
  //accomplished w/ multiplier that changes with modulus of pick #
//scooch function in MAR26 script does not look like it will move all 4 motors synchronously; will do front first then back
  //function in this script addresses that by mimicking form used in homing
//compressed first pick and weaving into one function by expanding conditionals in shedding fn
//used mod to assign state of pickLeft

// In this code, the 'front' of the machine is the side from which the reed is seen in front of the harness assembly. The fabric is woven on this side.
// In this code, the 'back' of the machine is the side from which the harness is seen in front of the reed assembly. The warp fibers "get shorter" on this side.

// Motor IDs Index Values
// 7-8:   Warp Tensioner Bank 1 (front) Motors (7 = Front Left || 8 = Front Right)
// 2-3:   Warp Tensioner Bank 2 (back) Motors (2 = Back Right || 3 = Back Left)
// 0:     Harness 1
// 1:     Harness 2
// 4:     Left Picking Motor
// 5:     Right Picking Motor
// 6:     Beat Up 
// 9-10:  Remaining Harnesses (10 = Third Harness || 11 = Fourth Harness)

//Limit Switch Pins:
//Digital Pin 8 - Front Warp
//Digital Pin 9 - Back Warp
//Digital Pin 10 - H1
//Digital Pin 11 - H2
//Digital Pin 12 - Reed

//Motor Directions
//Front Bank: (-, 0) = Towards Harnesses; (+, 1) = Away from Harnesses
//Back Bank: (-, 0) = Towards Harnesses; (+, 1) = Away from Harnesses
//Harnesses: (-, 0) = Up, (+, 1) = Down

const int numOfSteppers = 9; //number of stepper motors
const uint8_t csPins[numOfSteppers] = {28,29,30,31,32,33,34,35,36}; //Chip select pins for per motor, see Motor IDs Overview
const uint16_t stepPeriodsUs[numOfSteppers] = {1000, 1000, 1000, 1000, 1500, 1500, 15000, 1000, 1000}; //Step periods per motor (ms)
const uint16_t currentLimits[numOfSteppers] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; //Current limits per motor (mA)

//Microstepping mode per motor
const DRV8434SStepMode stepModes[numOfSteppers] = {
      DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, 
      DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, 
      DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2,
      };

DRV8434S steppers[numOfSteppers]; //Array to hold DRV9434S objects per motor


// Limit Switch Setup
// LimSwitches labelled in reference to motor
ezButton limitSwitchFront(8);
ezButton limitSwitchBack(9);
ezButton limitSwitchH1(10);
ezButton limitSwitchH2(11);
ezButton limitSwitchReed(12);

// Reed Switch Setup for Magnetic Detection
const int leftPickMagnetSwitch = 53; // Pin connected to reed switch
const int rightPickMagnetSwitch = 17; // Pin connected to reed switch


// Global variables for debouncing
unsigned long lastDebounceTimeLeft = 0; // Last time the left switch was toggled
unsigned long lastDebounceTimeRight = 0; // Last time the right switch was toggled
unsigned long debounceDelay = 50; // the debounce time in milliseconds

// Variables to hold the debounced state
int leftSwitchState = LOW;
int rightSwitchState = LOW;

// Control variables per motor type
bool shedOpen = false; //boolean that dictates the direction in which harnesses move; shedOpen = true means harnesses are "extended", false means they are on center
bool pickLeft = false;     //Dictates which pick motor is activated during picking, true = left, false = right

//Machine status variables
bool weavingActive = false;
bool error = false;
bool homed = false;
bool warpStrung = false;

//Pick counter variables
int totalPicks = 50;    //Change this to choose fabric length  
int currentPick = 1;   //Do not change, always 1

//
const int ERROR_LED_PIN = 13;   // Example pin number for the error LED
const int WEAVING_LED_PIN = 11; // Example pin number for the weaving LED



// SYSTEM POSITIONING VARIABLES/Array
// Variable distance step count array

// h1 shed height array
int h1FirstPick[] = {1300, 550, 440};   //steps from neutral axis to H1 first pick height
int h1StepsArray[] = {1447, 599, 480};

// h2 shed height array
int h2FirstPick[] = {1243, 512, 415};     //steps from neutral axis to H2 first pick height
int h2StepsArray[][3] = {
  {1518, 625, 498},
  {1511, 621, 496},
  {1503, 618, 493},
  {1494, 615, 490},
  {1484, 610, 486},
  {1472, 604, 482},
  {1458, 599, 477},
  {1442, 591, 472}
};

// bank 2 warp tensioning distance array
int strainRegulationFirstPick[] = {103, 103, 100};      //steps from warp tensioner start position to first pick position
int strainRegulationSteps[][3] = {
  {170, 170, 165},
  {174, 173, 168},
  {177, 177, 172},
  {181, 181, 176},
  {186, 185, 181},
  {191, 191, 186},
  {197, 197, 193},
  {203, 204, 200}
};

// Fixed distance step counts
int pickDistance = 787;     // Steps for picking distance
int beatupDistance = 35;   // Steps for beat up rotation
int scoochDistance = 13;   // Steps for tensioner to travel 0.25mm
int harnessHome = 2650; //2365 //Steps from harness switch to neutral plane



void setup() {
  Serial.begin(9600);
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

  //Reed Switch Initialization
  Serial.println("Setup complete, motors ready.");

  //pinMode(errorLED, OUTPUT); // Error LED
  //pinMode(weavingLED, OUTPUT); // Weaving LED
}


void loop() {
  
  //HOMING INSTRUCTIONS
  //Homing cannot start until a confirmation button is pressed
  
  
  if (!homed) { //will run on power cycle
    
    //check if "home" button (H1) pressed
    limitSwitchH1.loop();
    int state = limitSwitchH1.getState();
    
    if (state == HIGH) {
      homeMachine(); //sets homed to true at the end
      return;
    }

    //check if "start weave" button (H2) pressed
    waitForStart(); //sets homed to true at the end
    
  }


  // Main control loop for weaving
  if (weavingActive && !error) { //waitForStart() sets weavingActive = true, when pick limit reached weaving() sets weavingActive() false
      weaving();
      delay(1500);
    }
 
}


// Definition of stepper movement functions (Front Bank / Back Bank / H1 / H2 / LeftPick / RightPick / Beat Up / H3 / H4)
// Direction control is built into the runMotor function, so steps should be positive (+) and negative (-) to indicate direction

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

  delayMicroseconds(1); //is this doing anything?
}

//Front Bank Movement Function
void moveFrontBank (int stepsToRun) {

  if (stepsToRun > 0){ //away from harnesses
    steppers[7].setDirection(1);
    steppers[8].setDirection(1);
  } else{ //towards harnesses
    steppers[7].setDirection(0);
    steppers[8].setDirection(0);
    stepsToRun = - stepsToRun;
  }

  for (int step = 0; step < stepsToRun; step++){
    steppers[7].step();
    steppers[8].step();
    delayMicroseconds (stepPeriodsUs[7]);
  }
}

//Back Bank Movement Function
void moveBackBank (int stepsToRun) {

  if (stepsToRun > 0){ //away from harnesses
    steppers[2].setDirection(1);
    steppers[3].setDirection(1);
  } else{ //towards harnesses
    steppers[2].setDirection(0);
    steppers[3].setDirection(0);
    stepsToRun = - stepsToRun;
  }

  for (int step = 0; step < stepsToRun; step++){
    steppers[2].step();
    steppers[3].step();
    delayMicroseconds (stepPeriodsUs[3]);
  }
}

// H1 Movement Function
void moveH1 (int steps) {
  runMotor(0, steps);
  delayMicroseconds (stepPeriodsUs[0]);
}

// H2 Movement Function
void moveH2 (int steps) {
  runMotor (1, steps);
  delayMicroseconds (stepPeriodsUs[1]);
}

// Left Picking Motor Movement Function
void moveLeftPick (int steps){
  runMotor (4, steps);
  delayMicroseconds (stepPeriodsUs[4]);
}

// Right Picking Motor Movement Function
void moveRightPick (int steps){
  runMotor (5, steps);
  delayMicroseconds (stepPeriodsUs[5]);
}

// Beat Up Motor Movement Function
void moveBeatUp (int steps){
  runMotor (6, steps);
  delayMicroseconds (stepPeriodsUs[6]);
}

//HOMING FUNCTIONS
void homeReed() {
  
  steppers[6].setDirection(0); //set direction appropriately to move CW towards vertical

  int state;
  for (int step = 0; step < 1000; step++) { //change step number if reed doesn't make it to limit switch
    steppers[6].step();
    delayMicroseconds(stepPeriodsUs[6]);

    limitSwitchReed.loop();
    state = limitSwitchReed.getState();

    if (state == HIGH) {
      break;
      return;
    }
  }
}

void homeFrontBank() {
  steppers[7].setDirection(0); //towards harnesses
  steppers[8].setDirection(0);  

  int state;
  for (int step = 0; step < 10000; step++) { //change this if too small
    steppers[7].step();
    steppers[8].step();
    delayMicroseconds(stepPeriodsUs[0]);

   limitSwitchFront.loop();
   state = limitSwitchFront.getState();

    if (state == HIGH) {
      break;
      return;
    }
  }
}


void homeBackBank(){
  steppers[2].setDirection(1); //away from harnesses
  steppers[3].setDirection(1);  

  int state;
  for (int step = 0; step < 10000; step++) { //change this if too small
    steppers[2].step();
    steppers[3].step();
    delayMicroseconds(stepPeriodsUs[2]);

   limitSwitchBack.loop();
   state = limitSwitchBack.getState();

    if (state == HIGH) {
      break;
      return;
    }
  }
}

void homeHarnesses() {
  steppers[0].setDirection(1); //H1 down
  steppers[1].setDirection(1); //H2 down

  int state;

  //Bring H1 to switch, then lift to neutral plane
  for (int step = 0; step < 10000; step++) {
    steppers[0].step();
    delayMicroseconds(stepPeriodsUs[0]);
    
    limitSwitchH1.loop();
    state = limitSwitchH1.getState();
    if (state == HIGH) {
      break;
      
    }
  }

  moveH1(-harnessHome);
  delay(1000);

  //Bring H2 to switch, then lift to neutral plane
  for (int step = 0; step < 10000; step++) {
    steppers[1].step();
    delayMicroseconds(stepPeriodsUs[0]);
    
    limitSwitchH2.loop();
    state = limitSwitchH2.getState();
    if (state == HIGH) {
      break;
     
    }
  }

  moveH2(-harnessHome);
  
}

void frontOutTheWay() {
  homeFrontBank(); //drive bank to limit switch
  moveFrontBank(9000); //drive back towards the motor to get it out of the way
}

void homeMachine() {
      frontOutTheWay();
      delay(1000);
      homeReed();
      delay(1000);
      homeFrontBank();
      delay(1000);
      homeBackBank();
      delay(1000);
      homeHarnesses();
      homed = true;
}


//USER CONFIRMATION FUNCTION
void waitForStart() {
      limitSwitchH2.loop();
      int state = limitSwitchH2.getState();

      if (state == HIGH) {
        homed = true;
        weavingActive = true;
      }
}

//WEAVING FUNCTIONS
void shedding()  { 
  int indexNumber = roundf(currentPick/75);  //There are 8 values in arrays, 600 picks/75 = 8 --> number of picks is divided by 75 then rounded to nearest integer to get index value within array 

  //use a dummy +/-1 multiplier to invert directions as pick number changes
  int dirMultiplier = 1;
  if (currentPick % 2 == 1) { //odd picks
    dirMultiplier = 1;
  } else { //even picks
    dirMultiplier = -1;
  }

  //first pick
  if (currentPick == 1) {
    if (shedOpen == false) {
    //First Increment 
    moveH1(dirMultiplier * -h1FirstPick[0]); //h1 goes up on odd picks, down on even picks
    moveH2(dirMultiplier * h2FirstPick[0]); //h2 goes down on odd picks, up on even picks
    moveBackBank(-strainRegulationFirstPick[0]); // (-) = towards harnesses
    Serial.println("first done");
    //Second increment
    moveH1(dirMultiplier * -h1FirstPick[1]);
    moveH2(dirMultiplier * h2FirstPick[1]);
    moveBackBank(-strainRegulationFirstPick[1]);
    Serial.println("second done");
    //Third increment
    moveH1(dirMultiplier * -h1FirstPick[2]);
    moveH2(dirMultiplier * h2FirstPick[2]);
    moveBackBank(-strainRegulationFirstPick[2]);
    Serial.println("third done");

    shedOpen = true;
  } else if (shedOpen == true) {
    //Third increment
    moveH1(dirMultiplier * h1FirstPick[2]);
    moveH2(dirMultiplier * -h2FirstPick[2]);
    moveBackBank(strainRegulationFirstPick[2]); //(+) = away from harnesses
    //Second increment
    moveH1(dirMultiplier * h1FirstPick[1]);
    moveH2(dirMultiplier * -h2FirstPick[1]);
    moveBackBank(strainRegulationFirstPick[1]);
    //First Increment 
    moveH1(dirMultiplier * h1FirstPick[0]);
    moveH2(dirMultiplier * -h2FirstPick[0]);
    moveBackBank(strainRegulationFirstPick[0]);

    shedOpen = false;
  }
    
    //remaining picks, get indexed through main h2 and warp arrays
  } else if (currentPick > 1) {
    if (shedOpen == false) {
    //First Increment 
    moveH1(dirMultiplier * -h1StepsArray[0]); //h1 goes up on odd picks, down on even picks
    moveH2(dirMultiplier * h2StepsArray[indexNumber][0]); //h2 goes down on odd picks, up on even picks
    moveBackBank(-strainRegulationSteps[indexNumber][0]); // (-) = towards harnesses
    //Second increment
    moveH1(dirMultiplier * -h1StepsArray[1]);
    moveH2(dirMultiplier * h2StepsArray[indexNumber][1]);
    moveBackBank(-strainRegulationSteps[indexNumber][1]);
    //Third increment
    moveH1(dirMultiplier * -h1StepsArray[2]);
    moveH2(dirMultiplier * h2StepsArray[indexNumber][2]);
    moveBackBank(-strainRegulationSteps[indexNumber][2]);

    shedOpen = true;
  } else if (shedOpen == true) {
    //Third increment
    moveH1(dirMultiplier * h1StepsArray[2]);
    moveH2(dirMultiplier * -h2StepsArray[indexNumber][2]);
    moveBackBank(strainRegulationSteps[indexNumber][2]); //(+) = away from harnesses
    //Second increment
    moveH1(dirMultiplier * h1StepsArray[1]);
    moveH2(dirMultiplier * -h2StepsArray[indexNumber][1]);
    moveBackBank(strainRegulationSteps[indexNumber][1]);
    //First Increment 
    moveH1(dirMultiplier * h1StepsArray[0]);
    moveH2(dirMultiplier * -h2StepsArray[indexNumber][0]);
    moveBackBank(strainRegulationSteps[indexNumber][0]);

    shedOpen = false;
  }
  }

}

void picking(){ //Need to verify these signs
  if (currentPick % 2 == 1) {
    pickLeft = false; //odd picks use right rack
  } else {
    pickLeft = true; //even picks use left rack
  }
  
  if (pickLeft == true){
    moveLeftPick(-pickDistance);
    delay(1000); //short pause
    moveLeftPick(pickDistance);
  } else if (pickLeft == false){
    moveRightPick(pickDistance);
    delay(1000);
    moveRightPick(-pickDistance);
  }
}

void beatUp(){
  moveBeatUp(beatupDistance);
  delay(3000); //pause to let vibrations die down
  moveBeatUp(-beatupDistance);
}

void scooch(){
  //set bank directions
  steppers[7].setDirection(1); //front away from harnesses
  steppers[8].setDirection(1); //front away from harnesses
  steppers[2].setDirection(0); //back towards harnesses
  steppers[3].setDirection(0); //back towards harnesses
  
  for (int step = 0; step < scoochDistance; step++) {
    steppers[2].step();
    steppers[3].step();
    steppers[7].step();
    steppers[8].step();
    delayMicroseconds(stepPeriodsUs[2]);
  }
}

void weaving() {
  //open shed
  shedding();
  delay(500);

  //pick
  picking();
  delay(500);
  
  //close shed
  shedding();
  delay(500);
  
  //beat up
  beatUp();
  delay(500);
  
  //scooch
  //scooch();
  delay(500);

  //increment pick counter
  Serial.print(currentPick);
  Serial.print(" out of ");
  Serial.print(totalPicks);
  Serial.println(" completed.");

  currentPick++;

  if (currentPick > totalPicks) {
    weavingActive = false;
    Serial.println("Fabric is done.");
  }
}
