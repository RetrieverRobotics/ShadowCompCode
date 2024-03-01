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
#include <cmath>

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

int cycle_positions[2][8] = {0}; //Need to be set during initialization
void cycle_intake_pos(bool dir, pros::Motor &base, pros::Motor &joint) {
    static short int current_cycle_pos = 0;
    const int ERROR_MARGIN = 30;

    //Move the motor until close to target
    auto go_to = [](pros::Motor m, int pos, bool direction, int speed=SET_SPEEDS(HALF)) {
        while (std::abs(m.get_position() - pos) > ERROR_MARGIN) {
            m = direction ? speed : speed * -1;
        }
    };

    current_cycle_pos = dir ? current_cycle_pos + 1 : current_cycle_pos - 1;

    if (current_cycle_pos < 0)
        current_cycle_pos = 7;
    else if (current_cycle_pos == 8)
        current_cycle_pos = 0;
    
    go_to(base, cycle_positions[0][current_cycle_pos], dir);
    go_to(base, cycle_positions[1][current_cycle_pos], dir);
}

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
* | Face Button Up - Toggle tank/arcade      |
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
        //Handle arcade/tank toggle
        static bool is_arcade = 1;
        is_arcade = controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN) ? !is_arcade : is_arcade;

        //Set velocity for drive (arcade controls)
        int32_t tank_left_y = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int32_t tank_right_y = controller_master->get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);

        //Arcade vs Tank
        if (is_arcade) {
            int32_t drive_left_velocity = (int32_t)(((double)(tank_left_y) / (double)E_CONTROLLER_ANALOG_MAX)
                                            * MOTOR_BLUE_GEAR_MULTIPLIER);

            int32_t drive_right_velocity = (int32_t)(((double)(tank_right_y) / (double)E_CONTROLLER_ANALOG_MAX)
                                            * MOTOR_BLUE_GEAR_MULTIPLIER);                                

            drive_left.move_velocity(drive_left_velocity);
            drive_right.move_velocity(drive_right_velocity);
        } else {
            drive_left.move_velocity(tank_left_y);
            drive_right.move_velocity(tank_right_y);
        }

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
				shooter_top_motor.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER);
				shooter_middle_motor.move_velocity(-MOTOR_BLUE_GEAR_MULTIPLIER);
				shooter_bottom_motor.move_velocity(-MOTOR_BLUE_GEAR_MULTIPLIER);
			} else {
				shooter_top_motor.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER);
				shooter_middle_motor.move_velocity(-MOTOR_BLUE_GEAR_MULTIPLIER);
				shooter_bottom_motor.move_velocity(-MOTOR_BLUE_GEAR_MULTIPLIER);
			}
		}

        //Toggle for Intake Roller
        if (controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            pros::lcd::print(5, "New button press: L2 %d", !toggle[1]);
            toggle[1] = !toggle[1];

			if (toggle[1]) {
				roller.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER);
			} else {
                roller.move_velocity(ZERO);
            }
        }

        //Intake Arms
        int dir = 0;
		if (controller_master->get_digital(pros::E_CONTROLLER_DIGITAL_X)){ //Going Down
			//pros::lcd::print(5, "New button press: L2 %d", !toggle[1]);
			left_intake_arm.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER);
			right_intake_arm.move_velocity(-MOTOR_BLUE_GEAR_MULTIPLIER);
			dir = 1;
		}
		if (controller_master->get_digital(pros::E_CONTROLLER_DIGITAL_B)){ //Going Up
			//pros::lcd::print(5, "New button press: L2 %d", !toggle[1]);
			left_intake_arm.move(-MOTOR_BLUE_GEAR_MULTIPLIER);
			right_intake_arm.move(MOTOR_BLUE_GEAR_MULTIPLIER);
			dir -1;
		}

/*
// Intake edge case control
		double intake1_pos = intake1_arm.get_position();
		double intake2_pos = intake2_arm.get_position();
		if (!dir && (std::abs(intake1_pos - intake1_min) < motor_pos_error || std::abs(intake1_pos - intake1_max) < motor_pos_error)) {
			intake1_arm.move(0);
		}
		if (!dir && (std::abs(intake2_pos - intake2_min) < motor_pos_error || std::abs(intake2_pos - intake2_max) < motor_pos_error)) {
			intake2_arm.move(0);
		}

		pros::lcd::print(3, "Intake Arm 1 Range: %f to %f", intake1_min, intake1_max);
		pros::lcd::print(4, "Intake Arm 2 Range: %f to %f", intake2_min, intake2_max);
		pros::lcd::print(5, "Intake Arm 1 Pos: %f", intake1_arm.get_position());
		pros::lcd::print(6, "Intake Arm 2 Pos: %f", intake2_arm.get_position());



*/


        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}
