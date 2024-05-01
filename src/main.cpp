/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       nickr                                                     */
/*    Created:      11/26/2023, 10:21:21 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <string>
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

#define CONTROLLER_DEADBAND 5.0 // Joystick deadband for arcade drive

// TODO: Arm needs to be checked if it has stopped moving mid throw
bool bDisableArm = false; // Stop arm from moving if catch is deployed
int armMotorSpeed = 0;
#define ARM_MAX_RAISE_SPEED 80
#define ARM_MAX_LOWER_SPEED -100
#define ARM_MAX_RAISE_TORQUE 100.0 // percent. We need full torque while raising. TODO: Check if arm has stopped moving
#define ARM_MAX_LOWER_TORQUE 50.0 // percent. Do not need full torque while lowering. TODO: Check if arm has stopped moving
#define ARM_MIN_ANGLE 0.0 // Fully retracted or backstop
#define ARM_MAX_ANGLE -1142.0 // Fully extended or frontstop
#define ARM_MIN_DEADBAND_ANGLE -10.0 // No motor power once close to backstop
#define ARM_MAX_DEADBAND_ANGLE -1050.0 // No motor pwoer once close to frontstop

bool bDisableIntake = false; // No conditions yet
int intakeMotorSpeed = 0;
#define ARM_MAX_COLLECT_SPEED 100
#define ARM_MAX_EJECT_SPEED -100

// Used to detect is intake motor has stopped rotating, if it is we set to hold
bool intakeMotorStart = false;
int intakeMotorStartCount = 0;
#define INTAKE_STALL_SPEED 25
#define INTAKE_STALL_COUNT 80 // Crude timer to see how long intake is not spinning, approx 2sec

// TODO: Catch chatters when motor set to hold
// TODO: Can check initialization by rotating backwards?
#define USECATCH false;
bool bDisableCatch = !USECATCH; // set once we hit X on the controller, can finesse later
#define CATCH_MAX_SPEED 25
#define CATCH_MAX_ANGLE 90.0

#define USELIMIT false

#define DRIVESLOW 50.0
#define DRIVEFAST 100.0
#define TURNSLOW 25.0
#define TURNFAST 100.0
float maxdrive = DRIVEFAST;
float maxturn = TURNFAST;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

struct tSlingLog {
    int32_t time;
    float pos;
    float torque;
};

#define MAXLOG 20000
tSlingLog LOGARRAY[MAXLOG];
int32_t logId = 0;

void slingChanged(float pos, float torque)
{
    if (logId < MAXLOG) {
        LOGARRAY[logId].time = vex::timer::system();
        LOGARRAY[logId].pos = pos;
        LOGARRAY[logId].torque = torque;
        logId++;
    } else if (logId == MAXLOG) {
        printf("log buffer overflow\n");
        logId++;
    }
}

void DumpLog()
{
    if (!Brain.SDcard.isInserted()) {
        printf("No SDCARD\n");
    }
    if (logId >= MAXLOG) logId = MAXLOG;
    std::string str = "";
    char tempStr[100];
    str += "time, deg, torque\n";
    for (int i = 0; i < logId; i++) {
        sprintf(&(tempStr[0]), "%ld, %f, %f\n", LOGARRAY[i].time, LOGARRAY[i].pos, LOGARRAY[i].torque);
        str += tempStr;
        // str += LOGARRAY[i].time + LOGARRAY[i].val + "\n";
    }

    const char *cStr = str.c_str();
    int len = strlen(cStr);

    if (Brain.SDcard.isInserted()) {
        int saved = Brain.SDcard.savefile("data.csv", (uint8_t *) cStr, len);

        printf("%d of %d bytes written to file\n", saved, len);
        if (Brain.SDcard.exists("data.csv")) {
            printf("File size: %ld\n", Brain.SDcard.size("data.csv"));        }
    } else {
        printf("%s", cStr);
    }
    // printf("%s", str.c_str());
    // Brain.SDcard.savefile
}


// Main loop runrate, approx 25ms
int loopCount = 0;

bool armRetracted()
{
  printf("%d %lf\n", ArmLimit.value(), ArmMotor.position(vex::degrees));
  if (ArmLimit.value() == 1 || ArmMotor.position(vex::degrees) > ARM_MIN_DEADBAND_ANGLE) return true;
  return false;
}

// Raise Arm
// This is positive direction
void whenControllerL1Pressed() {

  if (bDisableArm) return;

  // The basic algorithm here is to check if the arm is already moving, if it is we stop it
  // Else we move it in the request direction
  // However, we need to check if we are close to an endstop (front or back) if that's the case
  // make sure we don't drive it towards the closest endstop

  if (!ArmMotor.isSpinning()) armMotorSpeed = 0; // Arm may have been stopped due to back/frontstop deadband
  if (armMotorSpeed != 0) armMotorSpeed = 0; // If we are spinning already, stop
  else armMotorSpeed = ARM_MAX_RAISE_SPEED;

  // NOTE!!! Gives position from program start, not absolute to motor
  double pos = ArmMotor.position(vex::degrees);
  // Check if arm is going to move towards an endstop, if not see if we move or hold position
  // If arm was already moving, we stop it
  // We want to be in coast if we within the deadband (10deg) to either endstop
  ArmMotor.setMaxTorque(ARM_MAX_RAISE_TORQUE, vex::percent);
  if (armMotorSpeed != 0 && pos < ARM_MIN_DEADBAND_ANGLE) {
    printf("start spin raise\n");
    ArmMotor.setBrake(hold);
    ArmMotor.setVelocity(armMotorSpeed, vex::percent);
    ArmMotor.spinToPosition(0.0, vex::degrees, false);
  } else if (armMotorSpeed == 0 && (pos < ARM_MIN_DEADBAND_ANGLE || pos > ARM_MAX_DEADBAND_ANGLE)) {
    printf("stop spin\n");
    ArmMotor.stop(hold);
  }

}

// Arm Lower
// This is negative direction
void whenControllerL2Pressed() {

  if (bDisableArm) return;

  // The basic algorithm here is to check if the arm is already moving, if it is we stop it
  // Else we move it in the request direction
  // However, we need to check if we are close to an endstop (front or back) if that's the case
  // make sure we don't drive it towards the closest endstop  

  if (!ArmMotor.isSpinning()) armMotorSpeed = 0; // Arm may have been stopped due to back/frontstop deadband
  if (armMotorSpeed != 0) armMotorSpeed = 0;  // If we are spinning already, stop
  else armMotorSpeed = ARM_MAX_LOWER_SPEED;

  // printf("Set spin to %d\n", armMotorSpeed);
  // NOTE!!! Gives position from program start, not absolute to motor
  double pos = ArmMotor.position(vex::degrees);
  // Check if arm is going to move towards an endstop, if not see if we move or hold position
  // If arm was already moving, we stop it
  // We want to be in coast if we within the deadband (10deg) to either endstop
  ArmMotor.setMaxTorque(ARM_MAX_LOWER_TORQUE, vex::percent);
  if (armMotorSpeed != 0 && pos > ARM_MAX_DEADBAND_ANGLE) {
    printf("start spin lower\n");
    ArmMotor.setBrake(hold);
    ArmMotor.setVelocity(-armMotorSpeed, vex::percent);
    ArmMotor.spinToPosition(ARM_MAX_ANGLE, vex::degrees, false);
  } else if (armMotorSpeed == 0 && (pos < ARM_MIN_DEADBAND_ANGLE || pos > ARM_MAX_DEADBAND_ANGLE)) {
    printf("stop spin\n");
    ArmMotor.stop(hold);
  }

}

// Intake Collect
// This is positive direction
void whenControllerR2Pressed() {

  if (bDisableIntake) return;
  
  // Same as arm ...
  // If intake is already moving, we stop
  // Else we spin in requested direction
  // For collect direction we use hold when the motor is not spinning

  bool wasEject = (intakeMotorSpeed == ARM_MAX_EJECT_SPEED) ? true : false;

  if (intakeMotorSpeed != 0) intakeMotorSpeed = 0;
  else intakeMotorSpeed = ARM_MAX_COLLECT_SPEED;

  if (intakeMotorSpeed != 0) {
    intakeMotorStart = true;
    intakeMotorStartCount = 0;
    IntakeMotor.setVelocity(intakeMotorSpeed, vex::percent);
    IntakeMotor.spin(forward);
  } else {
    intakeMotorStart = false;
    intakeMotorStartCount = 0;
    IntakeMotor.setVelocity(intakeMotorSpeed, vex::percent);
    if (wasEject) IntakeMotor.stop(coast);
    else IntakeMotor.stop(hold);
  }

}

// Intake Eject
// This is negative direction
void whenControllerR1Pressed() {

  if (bDisableIntake) return;

  // Same as arm ...
  // If intake is already moving, we stop
  // Else we spin in requested direction
  // For eject direction we use hold when the motor is not spinning

  bool wasCollect = (intakeMotorSpeed == ARM_MAX_COLLECT_SPEED) ? true : false;
  
  if (intakeMotorSpeed != 0) intakeMotorSpeed = 0;
  else intakeMotorSpeed = ARM_MAX_EJECT_SPEED;

  if (intakeMotorSpeed != 0) {
    intakeMotorStart = true;
    intakeMotorStartCount = 0;
    IntakeMotor.setVelocity(intakeMotorSpeed, vex::percent);
    IntakeMotor.spin(forward);
  } else {
    intakeMotorStart = false;
    intakeMotorStartCount = 0;
    IntakeMotor.setVelocity(intakeMotorSpeed, vex::percent);
    if (wasCollect) IntakeMotor.stop(hold);
    else IntakeMotor.stop(coast);
  }
}

// All Stop
// Simulates losing power at the end of the match
void whenControllerXPressed() {
  intakeMotorSpeed = 0;
  armMotorSpeed = 0;
  IntakeMotor.stop(coast);
  ArmMotor.stop(coast);
  CatchMotor.stop(coast);
  bDisableArm = true;
  bDisableCatch = true;
}

// Toggle the drive sensitivity
void whenControllerYPressed() {
  if (maxdrive == DRIVEFAST) {
    maxdrive = DRIVESLOW;
    maxturn = TURNSLOW;
  } else {
    maxdrive = DRIVEFAST;
    maxturn = TURNFAST;
  }
}

// Deploy Catch
// NOTE!!! - Must disable arm once this is done
// Assumes there that catch is roughly horizonal at the start of the program
void whenControllerUpPressed() {

  if (bDisableCatch) return;
  
  if (armRetracted()) {
    Brain.Screen.clearScreen(color::red);
    Brain.Screen.clearLine(3);
    Brain.Screen.setCursor(3 , 0);
    Brain.Screen.print("CATCH CAN NOT BE DEPLOYED");
    Brain.Screen.newLine();
    return;
  }

  bDisableArm = true;


  bDisableCatch = true;
  CatchMotor.setVelocity(CATCH_MAX_SPEED, vex::percent);
  CatchMotor.spinToPosition(CATCH_MAX_ANGLE, vex::degrees, false);

}

bool bCataOn = false;

void whenControllerAPressed()
{
  if (!bCataOn) {
    bCataOn = true;
    CataMotor.setBrake(coast);
    CataMotor.setStopping(coast);
    CataMotor.setVelocity(10, percent);
    CataMotor.spin(forward);
  } else {
    bCataOn = false;
    CataMotor.stop();
  }
}

bool bFlyWheelOn = false;

void whenControllerDownPressed()
{
  if (!bFlyWheelOn) {
    bFlyWheelOn = true;
    FlywheelMotor1.setBrake(coast);
    FlywheelMotor1.setVelocity(100, percent);
    FlywheelMotor1.spin(reverse);
    FlywheelMotor2.setBrake(coast);
    FlywheelMotor2.setVelocity(100, percent);
    FlywheelMotor2.spin(reverse);
  } else  {
    bFlyWheelOn = false;
    FlywheelMotor1.stop();
    FlywheelMotor2.stop();    
  }
}

enum eSlingState {STARTUP, INITIALIZED};
eSlingState slingState = STARTUP;
float maxSlingTorque = 0.0;
bool slingTension = false;
bool slingKill = false;
bool bSlingSample = false;

void whenControllerLeftPressed()
{
  SlingMotor.stop(hold);

  uint32_t startTime = timer::system();
  bool continuous = false;

  while (Controller1.ButtonLeft.pressing() && !continuous) {
    this_thread::sleep_for(100);
    if ((timer::system() - startTime) >= 1000) continuous = true;
  }

  if (slingState == STARTUP) {

  }

  SlingMotor.setBrake(hold);
  if (continuous)
    SlingMotor.setVelocity(80, percent);
  else
    SlingMotor.setVelocity(50, percent);
    
  float startposition = SlingMotor.position(degrees);
  float gearratio = 5.0;
  // slingTension = (SlingMotor.torque(Nm) > 0.3) ? true : false;
  slingTension = false;
  slingKill = false;

  float position  = startposition;
  if (continuous) {
    bSlingSample = true;
    if (slingState == INITIALIZED) position += (360.0) * gearratio;
    else position += (360.0 + 5.0) * gearratio;
    SlingMotor.spinToPosition(position, degrees, false);
    while(!SlingMotor.isDone() && !slingKill) {
      if (slingState == STARTUP) {
        if (!slingTension && ((SlingMotor.position(degrees) - startposition)  > 360.0))
          slingTension = (SlingMotor.torque(Nm) > 0.2) ? true : false;
      }
      this_thread::sleep_for(25);
    }
    bSlingSample = false;
  } else {
    position += 360.0 * 1.5;
    SlingMotor.spinToPosition(position, degrees, true);
    if (slingState == STARTUP) {
      slingState = INITIALIZED;
      SlingMotor.setPosition(0.0, degrees);
    }
  }

  printf("sling: tension %d, kill %d, startpos %f, position %f\n", slingTension, slingKill, startposition, position);

}

void whenControllerRightPressed()
{
  uint32_t startTime = timer::system();
  bool continuous = false;

  while (Controller1.ButtonRight.pressing() && !continuous) {
    this_thread::sleep_for(100);
    if ((timer::system() - startTime) >= 1000) continuous = true;
  }

  slingTension = false;
  slingKill = false;

  if (continuous) {
    SlingMotor.stop(coast);
    SlingMotor.setBrake(coast);
    DumpLog();
  } else {
    SlingMotor.stop(hold);
    SlingMotor.setBrake(hold);
    SlingMotor.setVelocity(25, percent);
    float position = SlingMotor.position(degrees);
    position -= 90.0;
    SlingMotor.spinToPosition(position, degrees, true);
  }
}

int SlingSampler()
{
  while (true) {
    float currentTorque = SlingMotor.torque(Nm);
    float currentPosition = SlingMotor.position(degrees);
    if (bSlingSample) slingChanged(currentPosition, currentTorque);
    if (slingTension && (currentTorque < 0.1)) slingKill = true;
    if (slingKill) SlingMotor.stop();
    if (currentTorque > maxSlingTorque) maxSlingTorque = currentTorque;

    this_thread::sleep_for(5);
  }
  return 0;
}

void usercontrol(void) {

  // Deadband stops the motors when Axis values are close to zero.
  int deadband = CONTROLLER_DEADBAND;
  int intakeHaltCount = 0;
  double armMaxTorque = 0.0;

  printf("START-- Arm motor pos = %f deg, torque = %f Nm, current = %f A\n", ArmMotor.position(vex::degrees), ArmMotor.torque(vex::Nm), ArmMotor.current(vex::amp));

  // Controller1.Screen.clearScreen();
  // Main loop, run at around 25ms
  // printf("lm %lf, rm %lf\n", LeftDrive.position(vex::degrees), RightDrive.position(vex::degrees));

  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    /**/ // Start drive train

    // DriveTrain - simple arcade drive using left joystick
    // Get the velocity percentage of the left motor. (Axis3 + Axis4)
    // Get the velocity percentage of the right motor. (Axis3 - Axis4)

    float drivescale = maxdrive / (100 - deadband);
    float turnscale = maxturn / (100 - deadband);
    float updown = Controller1.Axis3.position();
    float leftright = Controller1.Axis4.position();

    if (updown > 0) {
        if (updown < deadband) updown = 0.0;
        else updown = (updown - deadband) * drivescale;
    }
    else {
        if (updown > -deadband) updown = 0.0;
        else updown = (updown + deadband) * drivescale;
    }

    if (leftright > 0) {
        if (leftright < deadband) leftright = 0.0;
        else leftright = (leftright - deadband) * turnscale;
    } else {
        if (leftright > -deadband) leftright = 0;
        else leftright = (leftright + deadband) * turnscale;
    }

    DriveTrain.arcade(updown, leftright);

    // Set the speed of the left motor. If the value is less than the deadband,
    // set it to zero.
    // Set the speed to zero.
    // Set the speed to leftMotorSpeed
 
    // Set the speed of the right motor. If the value is less than the deadband,
    // set it to zero.
    // Set the speed to zero
    // Set the speed to rightMotorSpeed

    // Spin both motors in the forward direction.
    // LeftDrive.setVelocity(updown + leftright, percentUnits::pct);
    // RightDrive.setVelocity(updown - leftright, percentUnits::pct);
    // DriveTrain.
    //LeftDrive.spin(forward);
    //RightDrive.spin(forward);

    /**/ // End DriveTrain

    // Check to see if intake has stopped spinning for roughly 2sec
    // If it has, then set to hold so we are not fighting against a presumable game piece
    if (intakeMotorSpeed != 0 && intakeMotorStartCount > INTAKE_STALL_COUNT) {
      int currentSpeed = IntakeMotor.velocity(vex::percent);
      if (abs(currentSpeed) < INTAKE_STALL_SPEED) {
        IntakeMotor.setVelocity(0, vex::percent);
        IntakeMotor.stop(hold);
        intakeMotorSpeed = 0;

        intakeHaltCount++;
        Brain.Screen.clearLine(2);
        Brain.Screen.setCursor(2 , 0);
        Brain.Screen.print("stop intake activated %d", intakeHaltCount);
        Brain.Screen.newLine();
     }
    }
    else if (intakeMotorSpeed != 0) {
      intakeMotorStartCount++;
    }

    // Check if arm is spinning and closing in on and endstop, if so let it coast the rest of the way
    if (armMotorSpeed < 0 && ArmMotor.position(vex::degrees) < ARM_MAX_DEADBAND_ANGLE) {
      printf("stop spin\n");
      armMotorSpeed = 0;
      ArmMotor.stop(coast);
    } else if (armMotorSpeed > 0 && ArmMotor.position(vex::degrees) > ARM_MIN_DEADBAND_ANGLE) {
      printf("stop spin\n");
      armMotorSpeed = 0;
      ArmMotor.stop(coast);
    }

/**/

    if ((loopCount % 80) == 0) {
      Brain.Screen.clearLine(4);
      Brain.Screen.setCursor(4 , 1);
      Brain.Screen.print("max sling torque %lfNm", maxSlingTorque);
      Brain.Screen.newLine();
 //       printf("Arm motor pos = %f deg, torque = %f Nm, current = %f A\n", ArmMotor.position(vex::degrees), ArmMotor.torque(vex::Nm), ArmMotor.current(vex::amp));
 //       printf("lm %lf, rm %lf\n", LeftDrive.position(vex::degrees), RightDrive.position(vex::degrees));
      }

    double armTorque = ArmMotor.torque(vex::Nm);
    if (fabs(armTorque) > fabs(armMaxTorque)) {
      armMaxTorque = armTorque;
      Brain.Screen.clearLine(3);
      Brain.Screen.setCursor(3 , 1);
      Brain.Screen.print("max arm torque %lfNm", armMaxTorque);
      Brain.Screen.newLine();
    }

    wait(25, msec);
    loopCount++;
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

    // Install callbacks for buttons
  Controller1.ButtonL1.pressed(whenControllerL1Pressed);
  Controller1.ButtonL2.pressed(whenControllerL2Pressed);

  Controller1.ButtonR1.pressed(whenControllerR1Pressed);
  Controller1.ButtonR2.pressed(whenControllerR2Pressed);

  Controller1.ButtonUp.pressed(whenControllerUpPressed);
  Controller1.ButtonX.pressed(whenControllerXPressed);

  Controller1.ButtonY.pressed(whenControllerYPressed);
  Controller1.ButtonDown.pressed(whenControllerDownPressed);

  Controller1.ButtonLeft.pressed(whenControllerLeftPressed);
  Controller1.ButtonRight.pressed(whenControllerRightPressed);

    Controller1.ButtonA.pressed(whenControllerAPressed);
  
  printf("hello vex world\n");
  Brain.Screen.print("## BOWSER ##");
  Brain.Screen.newLine();

  Brain.Screen.print("Intake Velocity: %.2f", IntakeMotor.velocity(percent));
  Brain.Screen.newLine();

  int armLimitActivated = ArmLimit.value();
  if (USELIMIT && (armLimitActivated == 0)) {
    printf("arm limit = %d\n", armLimitActivated);
    Brain.Screen.clearScreen(color::red);
    Brain.Screen.clearLine(2);
    Brain.Screen.setCursor(2 , 0);
    Brain.Screen.print("ARM IS NOT IN CORRECT POSITION");
    Brain.Screen.newLine();

    bDisableArm = true;
    bDisableCatch = true;
  }

  int catchLimitActivated = CatchLimit.value();
  if (USELIMIT && (catchLimitActivated == 0)) {
    printf("catch limit = %d\n", catchLimitActivated);
    if (armLimitActivated != 0) Brain.Screen.clearScreen(color::red);
    Brain.Screen.clearLine(3);
    Brain.Screen.setCursor(3 , 0);
    Brain.Screen.print("CATCH IS NOT IN CORRECT POSITION");
    Brain.Screen.newLine();

    bDisableArm = true;
    bDisableCatch = true;
  }

  ArmMotor.setStopping(coast);
  IntakeMotor.setStopping(coast);
  CatchMotor.setStopping(hold);

  vex::thread t(SlingSampler);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
