#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// VEXcode device constructors
// NOTE: PUTT-PUTT PORT 19 is DEAD - do not use
controller Controller1 = controller(primary);
motor LeftFrontMotor = motor(PORT1, ratio18_1, false);
motor LeftRearMotor = motor(PORT3, ratio18_1, false);
motor_group LeftDrive = motor_group(LeftFrontMotor, LeftRearMotor);
motor RightFrontMotor = motor(PORT2, ratio18_1, true);
motor RightRearMotor = motor(PORT4, ratio18_1, true);
motor_group RightDrive = motor_group(RightFrontMotor, RightRearMotor);
drivetrain DriveTrain = drivetrain(LeftDrive, RightDrive, 319.19, 400.0, 342.9, mm, 1.0);
motor IntakeMotor = motor(PORT19, ratio6_1, false);
motor ArmMotor = motor(PORT18, ratio36_1, false);
motor CatchMotor = motor(PORT15, ratio18_1, true);
limit ArmLimit = limit(Brain.ThreeWirePort.A);
limit CatchLimit = limit(Brain.ThreeWirePort.B);
motor SlingMotor = motor(PORT10, ratio18_1, true);
motor CataMotor = motor(PORT12, ratio18_1, false);
motor FlywheelMotor1 = motor(PORT16, ratio6_1, true);
motor FlywheelMotor2 = motor(PORT17, ratio6_1, false);

// https://www.vexforum.com/t/vexcode-motor-groups-and-drivetrain-example/69161

// VEXcode generated functions

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // nothing to initialize
  wait(100, msec);
}