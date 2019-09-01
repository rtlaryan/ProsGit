#include "main.h"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	//Motors and Sensor Setup
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor frontRight(19, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor backRight(20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor frontLeft(12, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor backLeft(11, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor rightLift(5, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor leftLift(6, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	//Initialize Variables
		//Chassis Variables
		int x1 = 0;
		int x2 = 0;
		int y1 = 0;
		int threshold = 15;

		//Lift Variables
		int liftKp = 0;
		int liftKi = 0;
		int liftKd = 0;

		int liftPower = 0;
		int liftError = 0;

		int liftIntegral = 0;
		int liftIntegralLimit = 0;

		int liftDerivative = 0;
		int liftPreviousError = 0;

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

		////CHASSIS
		//Chassis Algorithem
		frontRight = y1 - x2 - x1;
		backRight =  y1 - x2 + x1;
		frontLeft = y1 + x2 + x1;
		backLeft =  y1 + x2 - x1;

		//DeadZone Algoritem
		if(abs(master.get_analog(ANALOG_LEFT_Y)) > threshold) y1 = master.get_analog(ANALOG_LEFT_Y);
		else y1 = 0;

		if(abs(master.get_analog(ANALOG_LEFT_X)) > threshold)	x1 = master.get_analog(ANALOG_LEFT_X);
		else x1 = 0;

		if(abs(master.get_analog(ANALOG_RIGHT_X)) > threshold) x2 = ANALOG_RIGHT_X;
		else x2 = 0;
		////LIFT CODE
		//Lift P calculation
		int leftLiftPosition = leftLift.get_position();
		int rightLiftPosition = rightLift.get_position();
		liftError = rightLiftPosition-leftLiftPosition;

		//Lift I calculation
		liftIntegral += liftError;
		if(liftError == 0) liftIntegral = 0;
		if(abs(liftError)>liftIntegralLimit) liftIntegral =0;

		//Lift D calculation
		liftDerivative = liftError - liftPreviousError;
		liftPreviousError = liftError;

		//Lift Controller input
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			liftPower = 100;
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			liftPower = -100;
		}
		else{
			liftPower = 0;
		}

		//Final Lift Output
		leftLift = (liftError*liftKp) + (liftIntegral*liftKi) + (liftDerivative*liftKd);
		rightLift = liftPower;


		pros::delay(20);
	}
}
