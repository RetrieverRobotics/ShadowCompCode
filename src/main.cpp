#include "main.h"
#include <cmath>

enum SET_SPEEDS{ZERO = 0, QUARTER = 127/4, HALF = 127/2, THREE_QUARTERS = (int)(0.75 * 127), MAX = 127}; //25%, 50%, 75%, 100%
enum ARM_DIRECTIONS{CLOSE = 0, OPEN = 1};
pros::Motor intake1_arm(9); 
pros::Motor intake2_arm(10);
pros::Motor indexer; //Not added to bot yet
double motor_pos_error = 50;
double intake1_min, intake1_max, intake2_min, intake2_max;

/*
* Determine if motor position is in range
*/
bool in_range(double pos, double target) {
	return std::abs(pos - target) < motor_pos_error;
}

/*
* Eases the arm motors towards their target positions
*/
void ease_arm_movement(bool direction) {

	//True is towards max, false is towards min (Open/Close)
	if (direction) {
		unsigned long int start_time = pros::c::millis();
		intake1_arm.move(SET_SPEEDS(QUARTER));
		intake2_arm.move(-SET_SPEEDS(QUARTER));

		//Move until position reached or timout
		while (!in_range(intake1_arm.get_position(), intake1_max) && !in_range(intake2_arm.get_position(), intake2_max) && pros::c::millis() - start_time < 2000) {
			double move_speed;
			double avg_offset = (std::abs(intake1_arm.get_position()) + std::abs(intake2_arm.get_position())) / 2;

			//TO-DO: Rewrite this code so that it's lowering
			// at a consistent speed opposite to the force of
			// graity. This is currently a low-priority.
			if (pros::c::millis() - start_time > 500) {
				move_speed = -SET_SPEEDS(QUARTER);
			}

			intake1_arm.move(-move_speed);
			intake2_arm.move(move_speed);
		}
	} else {
		unsigned long int start_time = pros::c::millis();
		intake1_arm.move(-SET_SPEEDS(MAX));
		intake2_arm.move(SET_SPEEDS(MAX));

		//Move until position reached or timout
		while (!in_range(intake1_arm.get_position(), intake1_min) && !in_range(intake2_arm.get_position(), intake2_min) && pros::c::millis() - start_time < 2500) {
			double move_speed;
			double avg_offset = (std::abs(intake1_arm.get_position()) + std::abs(intake2_arm.get_position())) / 2;

			//These values need to be tweaked more to allow for easing
			// Once done, this loop should be implemented for a decline,
			// which combined will be used for an intake toggle function.
			if (avg_offset > 200) {
				move_speed = SET_SPEEDS(HALF) * 1.25;
			} else if (avg_offset > 50) {
				move_speed = SET_SPEEDS(HALF) * 0.70;
			} else if (avg_offset > 25) {
				move_speed = SET_SPEEDS(HALF) * 0.35; //I know I could do SET_SPEEDS(QUARTER) but this is a stylistic choice
			} else {
				move_speed = SET_SPEEDS(HALF) * 0.25;
			}

			intake1_arm.move(move_speed);
			intake2_arm.move(-move_speed);
		}
	}

	//Stop moving to prevent motor burnout
	intake1_arm.move(SET_SPEEDS(ZERO));
	intake2_arm.move(SET_SPEEDS(ZERO));
}

/*
* Calibrates the min and max positions for the arms
*/
void calibrate_arms() {
	//Record start position
	intake1_min = intake1_arm.get_position();
	intake2_min = intake2_arm.get_position();

	//Open (lower) the arms
	intake1_arm.move(-SET_SPEEDS(QUARTER));
	intake2_arm.move(SET_SPEEDS(QUARTER));
	
	pros::delay(1500);
	
	//Record end position
	intake1_max = intake1_arm.get_position();
	intake2_max = intake2_arm.get_position();
	
	//Print min and max values 
	auto values1 = "Min/max: " + std::to_string(intake1_min) + " " + std::to_string(intake1_max);
	auto values2 = std::to_string(intake2_min) + " " + std::to_string(intake2_max);
	pros::lcd::set_text(1, values1);
	pros::lcd::set_text(2, values2);
	
	//Stop trying to open (to avoid burnout)
	intake1_arm.move(SET_SPEEDS(ZERO));
	intake2_arm.move(SET_SPEEDS(ZERO));

	pros::delay(1500);

	//Return to closed (up) position
	ease_arm_movement(ARM_DIRECTIONS(CLOSE));
}

/*
* Moves indexer (not currently implemented on bot)
*/
void move_indexer(int dir) {
	//Move forwards (towards shooter)
	if (dir == 1) {
		indexer.move(SET_SPEEDS(MAX));
	}

	//Move backwards (towards intake)
	else if (dir == -1) {
		indexer.move(-SET_SPEEDS(MAX));
	}

	//Stop moving indexer
	else {
		indexer.move(SET_SPEEDS(ZERO));
	}
}

/**
 * A callback function for LLEMU's center button. ttwrmwf.ke
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {

	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Chaos Control!");
	pros::lcd::register_btn1_cb(on_center_button);

	calibrate_arms();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


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
/*Temp cords for 
new_gyro_p_turn(47.4,100.0);
improved_pid_move(86.3,47.4,100.0);
new_gyro_p_turn(32.3,100.0);
improved_pid_move(114.2,32.3,100.0);
new_gyro_p_turn(92.4,100.0);
improved_pid_move(122.0,92.4,100.0);
new_gyro_p_turn(84.3,100.0);
improved_pid_move(25.5,84.3,100.0);
new_gyro_p_turn(-84.2,100.0);
improved_pid_move(176.2,-84.2,100.0); 
new_gyro_p_turn(-158.8,100.0);
improved_pid_move(182.5,-158.8,100.0);
new_gyro_p_turn(94.4,100.0);
improved_pid_move(33.1,94.4,100.0);
new_gyro_p_turn(180.0,100.0);
improved_pid_move(61.0,180.0,100.0);
*/
	/*
	const u_int32_t start_time = pros::c::millis();
	const int TILES = 3;
	const int MILISECONDPERTILE = 1000; //milisecond/tile when set at HALF speed
	const int RUN_TIME = TILES * MILISECONDPERTILE;
	pros::Motor front_left_mtr(2);
	pros::Motor back_left_mtr(1);
	pros::Motor front_right_mtr(14);
	pros::Motor back_right_mtr(13);
	pros::Motor climber1(15);
	pros::Motor climber2(16);
	pros::Motor intake(5); //temp num for shooter & intake, change when programmed
	pros::Motor shooter(6);
	pros::ADIGyro gyro(9);
	front_right_mtr.set_reversed(true);
	back_right_mtr.set_reversed(true); 
	
	auto move_all_motors = [front_left_mtr, front_right_mtr, back_left_mtr, back_right_mtr]
	(int speed) {
		front_left_mtr.move(speed);
		front_right_mtr.move(speed);
		back_left_mtr.move(speed);
		back_right_mtr.move(speed);
	};

	//use THREE_QUARTERS speed
	auto turn_ninety_degrees_right = [front_left_mtr, back_left_mtr]
	(int speed) {
		front_left_mtr.move(speed);
		back_left_mtr.move(speed);
	};

	//use HALF speed maybe?
	auto turn_fourtyfive_degrees_right = [front_left_mtr, back_left_mtr]
	(int speed) {
		front_left_mtr.move(speed);
		back_left_mtr.move(speed);
	};

	//use THREE_QUARTERS speed
	auto turn_ninety_degrees_left = [front_right_mtr, back_right_mtr]
	(int speed) {
		front_right_mtr.move(speed);
		back_right_mtr.move(speed);
	};

	while (pros::c::millis() - start_time < RUN_TIME)
	{
		//Move forward for one second
		move_all_motors(SET_SPEEDS(HALF));
		//Align/angle with the bar
		while(pros::c::millis() - start_time > 1000 && pros::c::millis() - start_time < 2000)
		{
			move_all_motors(SET_SPEEDS(ZERO));
			turn_ninety_degrees_right(SET_SPEEDS(THREE_QUARTERS));
			//turn_fourtyfive_degrees_right(SET_SPEEDS(HALF));
		}
		//Reverse to the bar
		while(pros::c::millis() - start_time > 2000 && pros::c::millis() - start_time < RUN_TIME)
		{
			move_all_motors(SET_SPEEDS(ZERO));
			front_right_mtr.set_reversed(false);
			back_right_mtr.set_reversed(false);
			front_left_mtr.set_reversed(true);
			back_left_mtr.set_reversed(true);
			move_all_motors(SET_SPEEDS(HALF));   
		}
		//pros::screen::print(pros::E_TEXT_MEDIUM, 2, "%d", front_left_mtr.get_actual_velocity());
		//pros::screen::print(pros::E_TEXT_MEDIUM, 3, "%d", back_left_mtr.get_actual_velocity());
		//pros::screen::print(pros::E_TEXT_MEDIUM, 4, "%d", front_right_mtr.get_actual_velocity());
		//pros::screen::print(pros::E_TEXT_MEDIUM, 5, "%d", back_right_mtr.get_actual_velocity());
	}


	front_left_mtr.move(SET_SPEEDS(ZERO));
	front_right_mtr.move(SET_SPEEDS(ZERO));
	back_left_mtr.move(SET_SPEEDS(ZERO));
	back_right_mtr.move(SET_SPEEDS(ZERO));
	*/
}

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
	//Initialize values
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor front_left_mtr(2);
	pros::Motor back_left_mtr(1);
	pros::Motor front_right_mtr(14);
	pros::Motor back_right_mtr(13);
	pros::Motor shooter1(15);
	pros::Motor shooter2(16);
	pros::Motor shooter3(17);
	pros::Motor roller(7); 
	pros::Motor intake1_arm(9); 
	pros::Motor intake2_arm(10);
	front_right_mtr.set_reversed(true);
	back_right_mtr.set_reversed(true);

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
	while (true) {
		//Twin stick movement	 
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		//Initialize with the intake and shooter toggled off
		// Note: Intake at i=0, Shooter at i=1
		static bool toggle[2] {{false}}; 
		static int speeds[2] {{SET_SPEEDS(ZERO)}}; 

		//Toggles speed between values in SET_SPEEDS enum
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			speeds[1] = speeds[1] == 0 ? 31 : (speeds[1] + 32) % 159;
		}

		//Shooter Controls
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { //If R2 gets pressed
			//pros::lcd::print(5, "New button press: R2 %d", !toggle[0]);
			toggle[0] = !toggle[0];
			if (toggle[0]) {
				shooter1.move(SET_SPEEDS(MAX));
				shooter2.move(-SET_SPEEDS(MAX));
				shooter3.move(-SET_SPEEDS(MAX));
			} else {
				shooter1.move(SET_SPEEDS(ZERO));
				shooter2.move(SET_SPEEDS(ZERO));
				shooter3.move(SET_SPEEDS(ZERO));
			}
		}

		// Intake Roller
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			pros::lcd::print(5, "New button press: L2 %d", !toggle[1]);
			toggle[1] = !toggle[1];

			if (toggle[1]) {
				roller = speeds[1]; 
			}
		}

		// Intake Arms
		static int dir = 0; //1=Open, -1=Close
		
		//Going Down
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){ 
			//pros::lcd::print(5, "New button press: L2 %d", !toggle[1]);
			intake1_arm.move(SET_SPEEDS(MAX));
			intake2_arm.move(-SET_SPEEDS(MAX));
			dir = 1;
		}

		//Going Up
		else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){ 
			//pros::lcd::print(5, "New button press: L2 %d", !toggle[1]);
			intake1_arm.move(-SET_SPEEDS(MAX));
			intake2_arm.move(SET_SPEEDS(MAX));
			dir = -1;
		}

		double intake1_pos = intake1_arm.get_position();
		double intake2_pos = intake2_arm.get_position();

		//Auto shutoff for the arms once they're in range
		if (dir == 1 && in_range(intake1_pos, intake1_max) && in_range(intake2_pos, intake2_max)) {
			intake1_arm.move(SET_SPEEDS(ZERO));
			intake2_arm.move(SET_SPEEDS(ZERO));
		} else if (dir == -1 && in_range(intake1_pos, intake1_min) && in_range(intake2_pos, intake2_min)) {
			intake1_arm.move(SET_SPEEDS(ZERO));
			intake2_arm.move(SET_SPEEDS(ZERO));
		}

		pros::lcd::print(5, "Intake Arm 1 Pos: %f", intake1_arm.get_position());
		pros::lcd::print(6, "Intake Arm 2 Pos: %f", intake2_arm.get_position());

		//Set Drive Train motor speeds
		front_left_mtr = left;
		back_left_mtr = left;
		front_right_mtr = right;
		back_right_mtr = right;

		pros::delay(20);
	}
}