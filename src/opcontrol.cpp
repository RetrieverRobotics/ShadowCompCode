/**
 * \file opcontrol.cpp
 *
 * Contains user implemented opcontrol. User must use the
 * parameters to the opcontrol function when referencing
 * the master V5 controller or partner V5 controller.
 */

#include "api.h"
#include "umbc.h"

#include <cstdint>

using namespace pros;
using namespace umbc;
using namespace std;

#define MOTOR_RED_GEAR_MULTIPLIER    100
#define MOTOR_GREEN_GEAR_MULTIPLIER  200
#define MOTOR_BLUE_GEAR_MULTIPLIER   600
#define MOTOR_REVERSE                true

//Drive Motors
#define FRONT_LEFT_MOTOR 2
#define BACK_LEFT_MOTOR 1
#define FRONT_RIGHT_MOTOR 14
#define BACK_RIGHT_MOTOR 13

//Shooter Motors
#define BOTTOM_SHOOTER_MOTOR 15
#define MIDDLE_SHOOTER_MOTOR 16
#define TOP_SHOOTER_MOTOR 17

//Roller Motor
#define ROLLER_MOTOR 7

//Intake Motor
#define LEFT_INTAKE_ARM_MOTOR 9
#define RIGHT_INTAKE_ARM_MOTOR 10

enum SET_SPEEDS{ZERO = 0, QUARTER = 127/4, HALF = 127/2, THREE_QUARTERS = (int)(0.75 * 127), MAX = 127}; //25%, 50%, 75%, 100%
enum ARM_DIRECTIONS{CLOSE = 0, OPEN = 1}; 


void umbc::Robot::opcontrol() {

    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    //initialize left drive
    pros:: Motor drive_left_front_motor = pros::Motor(FRONT_LEFT_MOTOR);
    pros:: Motor drive_left_back_motor = pros::Motor(BACK_LEFT_MOTOR);
    pros::MotorGroup drive_left = pros::MotorGroup(vector<pros::Motor>{drive_left_front_motor, drive_left_back_motor});
    drive_left.set_brake_modes(E_MOTOR_BRAKE_COAST);
    drive_left.set_gearing(E_MOTOR_GEAR_BLUE);

    //initialize right drive
    pros:: Motor drive_right_front_motor = pros::Motor(FRONT_RIGHT_MOTOR);
    pros:: Motor drive_right_back_motor = pros::Motor(BACK_RIGHT_MOTOR);
    pros::MotorGroup drive_right = pros::MotorGroup(vector<pros::Motor>{drive_right_front_motor, drive_right_back_motor});
    drive_right.set_brake_modes(E_MOTOR_BRAKE_COAST);
    drive_right.set_gearing(E_MOTOR_GEAR_BLUE);

    //initialize shooters
    pros:: Motor shooter_bottom_motor = pros::Motor(TOP_SHOOTER_MOTOR);
    pros:: Motor shooter_middle_motor = pros::Motor(MIDDLE_SHOOTER_MOTOR);
    pros:: Motor shooter_top_motor = pros::Motor(TOP_SHOOTER_MOTOR);
    pros::MotorGroup shooters = pros::MotorGroup(vector<pros::Motor>{shooter_bottom_motor, shooter_middle_motor, shooter_top_motor});
    shooters.set_brake_modes(E_MOTOR_BRAKE_COAST);
    shooters.set_gearing(E_MOTOR_GEAR_BLUE);

    //initialize lift arms
    pros::Motor left_intake_arm = pros::Motor(LEFT_INTAKE_ARM_MOTOR);
    pros::Motor right_intake_arm = pros::Motor(RIGHT_INTAKE_ARM_MOTOR);
    pros::MotorGroup intake_arms = pros::MotorGroup(vector<pros::Motor>{left_intake_arm, right_intake_arm});
    intake_arms.set_brake_modes(E_MOTOR_BRAKE_COAST);
    intake_arms.set_gearing(E_MOTOR_GEAR_RED);

    //initialize roller
    pros::Motor roller = pros::Motor(ROLLER_MOTOR);
    pros::MotorGroup rollers = pros::MotorGroup(vector<pros::Motor>{roller});
    rollers.set_brake_modes(E_MOTOR_BRAKE_COAST);
    rollers.set_gearing(E_MOTOR_GEAR_BLUE);

    while(1) {
/*
*                **Controls**
* -------------------------------------------
* | Left analog X -                          |
* | Left analog Y - Left wheel drive         |
* | Right analog X -                         |
* | Right analog Y - Right wheel drive       |
* | Face Button A - Move indexer forward     |
* | Face Button B - Move intake up           |
* | Face Button X - Move intake down         |
* | Face Button Y - Move indexer backward    |
* | Face Button Up -                         |
* | Face Button Down - Turn indexer off      |
* | Face Button Left -                       |
* | Face Button Right -                      |
* | Shoulder Button R1 -                     |
* | Shoulder Button R2 - Toggle shooter      |
* | Shoulder Button L1 - Adjust shooter spd  |
* | Shoulder Button L2 -                     |
* -------------------------------------------
* 
*/
       // set velocity for drive (arcade controls)
        int32_t tank_left_y = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int32_t tank_right_y = controller_master->get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

        int32_t drive_left_velocity = (int32_t)(((double)(tank_left_y) / (double)E_CONTROLLER_ANALOG_MAX)
                                        * MOTOR_BLUE_GEAR_MULTIPLIER);

        int32_t drive_right_velocity = (int32_t)(((double)(tank_right_y) / (double)E_CONTROLLER_ANALOG_MAX)
                                        * MOTOR_BLUE_GEAR_MULTIPLIER);                                

        drive_left.move_velocity(drive_left_velocity);
        drive_right.move_velocity(drive_right_velocity);

        //Initialize with the intake and shooter toggled off
		// Note: Intake at i=0, Shooter at i=1
        static bool toggle[2] {{false}}; 
		static int speeds[2] {{SET_SPEEDS(ZERO)}}; 

        //Toggles speed between values in SET_SPEEDS enum
		if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			speeds[1] = speeds[1] == 0 ? 31 : (speeds[1] + 32) % 159;
		}

        // Shooter 
        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { //If R2 gets pressed
			//pros::lcd::print(5, "New button press: R2 %d", !toggle[0]);
            //even tho I assigned motor groups, I have to do them manually to reverse specific motors >:(
			toggle[0] = !toggle[0];
			if (toggle[0]) {
				shooter_top_motor.move(SET_SPEEDS(MAX));
				shooter_middle_motor.move(-SET_SPEEDS(MAX));
				shooter_bottom_motor.move(-SET_SPEEDS(MAX));
			} else {
				shooter_top_motor.move(SET_SPEEDS(ZERO));
				shooter_middle_motor.move(-SET_SPEEDS(ZERO));
				shooter_bottom_motor.move(-SET_SPEEDS(ZERO));
			}
		}

        //Toggle for Intake Roller
        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            pros::lcd::print(5, "New button press: L2 %d", !toggle[1]);
            toggle[1] = !toggle[1];

			if (toggle[1]) {
				roller = speeds[1]; 
			}
        }



        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}
