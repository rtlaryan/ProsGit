#include "main.h"

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  //Motors and Sensors
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  pros::Motor frontRight(19, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor backRight(20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor frontLeft(12, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor backLeft(11, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor rightLift(5, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor leftLift(6, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

  pros::ADIEncoder leftEncoder (1,2);
  pros::ADIEncoder rightEncoder (3,4);
  pros::ADIEncoder backEncoder (5,6);

  //Initialize Variables
  float Sl = 0; //distance from tracking center to left tracking wheel
  float Sr = 0; //distance from tracking center to right tracking wheel
  float Ss = 0; //distance from tracking center to back tracking wheel
  float Do = 0; //previous global position vector
  float To = 0; //prevous global orientation
  float Tr = 0; //global orientation at last reset

}
