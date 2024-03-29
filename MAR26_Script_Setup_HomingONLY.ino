#include <SPI.h>
#include <DRV8434S.h>
#include <ezButton.h>
#include <math.h>

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

//Motor Directions (1 is (+), -1 is (-))
//Front Bank: (-) = Towards Harnesses; (+) = Away from Harnesses
//Back Bank: (-) = Towards Errors; (+) = Away from Harnesses
//H1: (+) = Down, (-) is up
//H2: (+) = down, (-) is up

const int numOfSteppers = 11; //number of stepper motors
const uint8_t csPins[numOfSteppers] = {28,29,30,31,32,33,34,35,36}; //Chip select pins for per motor, see Motor IDs Overview
const uint16_t stepPeriodsUs[numOfSteppers] = {1000, 1000, 1000, 1000, 500, 500, 15000, 1000, 1000}; //Step periods per motor (ms)
const uint16_t currentLimits[numOfSteppers] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; //Current limits per motor (mA)

//Microstepping mode per motor
const DRV8434SStepMode stepModes[numOfSteppers] = {
      DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, 
      DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, 
      DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2,
      DRV8434SStepMode::MicroStep2, DRV8434SStepMode::MicroStep2};

DRV8434S steppers[numOfSteppers]; //Array to hold DRV9434S objects per motor

ezButton limitSwitchFront(8);
ezButton limitSwitchBack(9);
ezButton limitSwitchH1(10);
ezButton limitSwitchH2(11);
ezButton limitSwitchReed(12);

// Limit Switch Setup
// LimSwitches labelled in reference to motor


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
int totalPicks = 10;    //Change this to chose fabric length  
int currentPick = 1;   //Do not change, always 1

//
const int ERROR_LED_PIN = 13;   // Example pin number for the error LED
const int WEAVING_LED_PIN = 11; // Example pin number for the weaving LED



// SYSTEM POSITIONING VARIABLES/Array
// Variable distance step count array
// h1 shed height array
float h1StepsArray[] = {679.00, 281.00, 210.00};

// h2 shed height array
float h2StepsArray[][3] = {
  {701.00, 289.00, 216.00},
  {700.00, 289.00, 216.00},
  {698.00, 288.00, 215.00},
  {697.00, 287.00, 215.00},
  {695.00, 286.00, 214.00},
  {693.00, 285.00, 214.00},
  {690.00, 285.00, 212.00},
  {688.00, 283.00, 212.00}
};

// h3 shed height array
float h3StepsVect[] = {0}; // Not sure if this should be a float array

// h4 shed height array
float h4StepsVect[] = {0}; // Not sure if this should be a float array

// bank 2 warp tensioning distance array
float bank2TensioningLocationArray[][3] = {
  {81.00, 81.00, 78.00},
  {83.00, 82.00, 80.00},
  {84.00, 84.00, 82.00},
  {86.00, 86.00, 83.00},
  {88.00, 88.00, 86.00},
  {90.00, 90.00, 88.00},
  {93.00, 93.00, 91.00},
  {96.00, 96.00, 94.00}
};

// Fixed distance step counts
int pickDistance = 0;     // Steps for picking distance
int beatupDistance = 0;   // Steps for beat up rotation
int scoochDistance = 0;   // Steps for tensioner to travel 1.6 fiber diameters


//Number of increments to divide shedding motion into
int numberOfIncrements = 50;


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

  Serial.begin(9600);
  Serial.println("Setup complete, motors ready.");



  limitSwitchFront.setDebounceTime(50);
  limitSwitchBack.setDebounceTime(50);
  limitSwitchH1.setDebounceTime(50);
  limitSwitchH2.setDebounceTime(50);
  limitSwitchReed.setDebounceTime(50);

  //pinMode(errorLED, OUTPUT); // Error LED
  //pinMode(weavingLED, OUTPUT); // Weaving LED
}




void loop() {
  //HOMING INSTRUCTIONS
  //Homing cannot start until a confirmation button is pressed


moveH2(-3955);




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
void moveFrontBank (int stepsToRun) {

  if (stepsToRun > 0){
    steppers[7].setDirection(1);
    steppers[8].setDirection(1);
  } if(stepsToRun < 0){
    steppers[7].setDirection(0);
    steppers[8].setDirection(0);
    stepsToRun = - stepsToRun;
  }

  for (int step = 0; step < stepsToRun; step++){
    steppers[7].step();
    steppers[8].step();
    delayMicroseconds (stepPeriodsUs[0]);
  }
}

// Bank2 Movement Function
void moveBackBank (int stepsToRun) {

  if (stepsToRun > 0){
    steppers[2].setDirection(1);
    steppers[3].setDirection(1);
  } else{
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
  runMotor (0, steps);
  delayMicroseconds (1);
}

// H2 Movement Function
void moveH2 (int steps) {
  runMotor (1, steps);
  delayMicroseconds (1);
}

// Left Picking Motor Movement Function
void moveLeftPick (int steps){
  runMotor (4, steps);
  delayMicroseconds (1);
}

// Right Picking Motor Movement Function
void moveRightPick (int steps){
  runMotor (5, steps);
  delayMicroseconds (1);
}

// Beat Up Motor Movement Function
void moveBeatUp (int steps){
  runMotor (6, steps);
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

void Fronthomemin(int stepsToRun) { // Homemin function
  if (stepsToRun > 0){
    steppers[7].setDirection(1);
    steppers[8].setDirection(1);  
  }if (stepsToRun <0){
    steppers[7].setDirection(0);
    steppers[8].setDirection(0);
    stepsToRun = - stepsToRun;
  }

  int state;
  for (int step = 0; step < stepsToRun; step++) {
    steppers[7].step();
    steppers[8].step();
    delayMicroseconds(stepPeriodsUs[0]);

   limitSwitchFront.loop();
   int state = limitSwitchFront.getState();

    if (state == HIGH) {
      break;
      return;
    }
  }
}


void Backhomemin(int stepsToRun) // Homemin function
  if (stepsToRun > 0){
    steppers[2].setDirection(1);
    steppers[3].setDirection(1);  
  }if (stepsToRun <0){
    steppers[2].setDirection(0);
    steppers[3].setDirection(0);
    stepsToRun = - stepsToRun;
  }

  int state;
  for (int step = 0; step < stepsToRun; step++) {
    steppers[2].step();
    steppers[3].step();
    delayMicroseconds(stepPeriodsUs[2]);

   limitSwitchBack.loop();
   int state = limitSwitchBack.getState();

    if (state == HIGH) {
      break;
      return;
    }
  }
}

void H1homemin(int stepsToRun) { // Homemin function
  if (stepsToRun > 0){
    steppers[0].setDirection(1);
  }if (stepsToRun <0){
    steppers[0].setDirection(0);
    stepsToRun = - stepsToRun;
  }

  int state;
  for (int step = 0; step < stepsToRun; step++) {
    steppers[0].step();
    delayMicroseconds(stepPeriodsUs[0]);

    limitSwitchH1.loop();
    int state = limitSwitchH1.getState();

    if (state == HIGH) {
      break;
      return;
    }
  }
}

void H2homemin(int stepsToRun){ // Homemin function
  if (stepsToRun > 0){
    steppers[1].setDirection(1);
  }if (stepsToRun <0){
    steppers[1].setDirection(0);
    stepsToRun = - stepsToRun;
  }

  int state;
  for (int step = 0; step < stepsToRun; step++) {
    steppers[1].step();
    delayMicroseconds(stepPeriodsUs[1]);

    limitSwitchH2.loop();
    int state = limitSwitchH2.getState();

    if (state == HIGH) {
      break;
      return;
    }
  }
}

void Reedhomemin(int stepsToRun) { // Homemin function
  if (stepsToRun > 0){
    steppers[6].setDirection(1);
  }if (stepsToRun <0){
    steppers[6].setDirection(0);
    stepsToRun = - stepsToRun;
  }

  int state;
  for (int step = 0; step < stepsToRun; step++) {
    steppers[6].step();
    delayMicroseconds(stepPeriodsUs[6]);

    limitSwitchReed.loop();
    int state = limitSwitchReed.getState();

    if (state == HIGH) {
      break;
      return;
    }
  }
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



void weaving (numberOfPicks, totalNumberOfPicks, h1StepsArray, h2StepsArray, bank2TensioningLocationArray) {

  indexNumber = roundf(numberOfPicks/75);  //There are 8 values in arrays, 600 picks/75 = 8 --> number of picks is divided by 75 then rounded to nearest integer to get index value within array 

  for (int j = 0; j < 3; j++) {
    h2SingleLine[1][j] = h2[indexNumber][j]);
  }
  
  warpStepsNum = bank2TensioningLocationArray(indexNumber);

  sheddingThreeStepsBoring(h1StepsArray, h2SingleLine, warpStepsNum);
  Serial.println("picking occurs here doofus");
  sheddingThreeStepsBoring(-h1StepsArray, -h2SingleLine, -warpStepsNum);
  moveBeatUp(25);
  moveBeatUp(-25);

  numberOfPicks++;

  if (numberOfPicks < totalNumberOfPicks) {
    weaving(numberOfPicks, totalNumberOfPicks, h1StepsArray, h2StepsArray, bank2TensioningLocationArray);
  } else {
    Serial.println((STRING)"Weaving is finished, "+numberOfPicks+" out of "+totalNumberOfPicks+" completed.")
  }
}

void firstPick () {
  //First pick distances IN STEPS
  h1FirstPick = {475.00, 197.00, 147.00};   //steps from neutral axis to H1 first pick height
  h2FirstPick = {484.00, 200.00, 150.00};     //steps from neutral axis to H2 first pick height

  warpFirstPick = {32.00, 32.00, 31.00};      //steps from warp tensioner start position to first pick position

  sheddingThreeStepsBoring(h1FirstPick, h2FirstPick, warpFirstPick, numberOfIncrements);
  Serial.println("picking happens here, if someone codes it **vomit**"); //picking code goes here
  sheddingThreeStepsBoring(-h1FirstPick, -h2FirstPick, -warpFirstPick, numberOfIncrements);
  Serial.println("first pick is done woohoo! balls.")
  moveBeatUp(25);
}

void sheddingThreeStepsBoring(h1Steps, h2Steps, warpSteps)  {  
  if h1Steps > 0 {
    //First Increment 
    moveH1(h1Steps(1));
    moveH2(h2Steps(1));
    moveBank2(warpSteps(1));
    //Second increment
    moveH1(h1Steps(2));
    moveH2(h2Steps(2));
    moveBank2(warpSteps(2));
    //Third increment
    moveH1(h1Steps(3));
    moveH2(h2Steps(3));
    moveBank2(warpSteps(3));
=  } else if (h1Steps < 0) {
    //Third increment
    moveH1(-h1Steps(3));
    moveH2(-h2Steps(3));
    moveBank2(-warpSteps(3));
    //Second increment
    moveH1(-h1Steps(2));
    moveH2(-h2Steps(2));
    moveBank2(-warpSteps(2));
    //First Increment 
    moveH1(-h1Steps(1));
    moveH2(-h2Steps(1));
    moveBank2(-warpSteps(1));
  } else {
    error = true;
  }
}




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
