#include "main.h"
#include<iostream>
#include<list>
using namespace std;
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
		double x1 = 0;
		double x2 = 0;
		double y1 = 0;
		int threshold = 15;

		double chassisMultiplier = 0;
  	int chassisOutput = 1;
		int chassisConstant = 50;
		//Lift Variables
		double liftKp = 0;
		double liftKi = 0;
		double liftKd = 0;

		double liftPower = 0;
		double liftError = 0;

		double liftIntegral = 0;
		double liftIntegralLimit = 0;

		double liftDerivative = 0;
		double liftPreviousError = 0;

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

		////CHASSIS
		//Chassis Algorithem
/*	frontRight = y1 - x1 - x2;
		backRight =  y1 - x1 + x2;
		frontLeft = y1 + x1 + x2;
		backLeft =  y1 + x1 - x2;
*/

		//DeadZone Algoritem
		if(abs(master.get_analog(ANALOG_LEFT_Y)) > threshold) y1 = master.get_analog(ANALOG_LEFT_Y);
		else y1 = 0;

		if(abs(master.get_analog(ANALOG_LEFT_X)) > threshold)	x1 = master.get_analog(ANALOG_LEFT_X);
		else x1 = 0;

		if(abs(master.get_analog(ANALOG_RIGHT_X)) > threshold) x2 = ANALOG_RIGHT_X;
		else x2 = 0;

		//Chassis Debugging
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) chassisMultiplier = 2;
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) chassisMultiplier = 0.5;
		else chassisMultiplier = 1;

		chassisOutput = chassisConstant * chassisMultiplier;

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
			frontRight = chassisOutput;
			frontLeft = chassisOutput;
			backLeft = chassisOutput;
			backRight = chassisOutput;
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
			frontRight = -1*chassisOutput;
			frontLeft = -1*chassisOutput;
			backLeft = -1*chassisOutput;
			backRight = -1*chassisOutput;
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
			frontRight = chassisOutput;
			frontLeft = -1*chassisOutput;
			backLeft = chassisOutput;
			backRight = -1*chassisOutput;
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
			frontRight = -1*chassisOutput;
			frontLeft = chassisOutput;
			backLeft = -1*chassisOutput;
			backRight = chassisOutput;
		}


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
