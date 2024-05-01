using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LeftFrontMotor;
extern motor RightFrontMotor;
extern motor_group LeftDrive;
extern motor_group RightDrive;
extern drivetrain DriveTrain;
extern motor IntakeMotor;
extern motor ArmMotor;
extern motor CatchMotor;
extern limit ArmLimit;
extern limit CatchLimit;
extern motor SlingMotor;
extern motor CataMotor;
extern motor FlywheelMotor1;
extern motor FlywheelMotor2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);