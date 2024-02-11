#include "main.h"
#include <cmath>

enum SET_SPEEDS{ZERO = 0, QUARTER = 127/4, HALF = 127/2, THREE_QUARTERS = (int)(0.75 * 127), MAX = 127}; //25%, 50%, 75%, 100%
pros::Motor intake1_arm(9); //temp port number, subject to change
pros::Motor intake2_arm(10); //temp port number, subject to change
double motor_pos_error = 0.25;

/*
* Calibrates the min and max positions for the arms
*/
double intake1_min, intake1_max, intake2_min, intake2_max;
void calibrate_arms() {
	intake1_arm.move(SET_SPEEDS(HALF));
	intake2_arm.move(-SET_SPEEDS(HALF));
	double start_time = pros::c::millis();
	while (pros::c::millis() - start_time > 3000) {}
	intake1_min = intake1_arm.get_position();
	intake2_min = intake2_arm.get_position();

	intake1_arm.move(-SET_SPEEDS(MAX));
	intake2_arm.move(SET_SPEEDS(MAX));
	start_time = pros::c::millis();
	while (pros::c::millis() - start_time > 3000) {}
	intake1_max = intake1_arm.get_position();
	intake2_max = intake2_arm.get_position();
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
		pros::lcd::set_text(2, "I was pressed!");
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor front_left_mtr(2);
	pros::Motor back_left_mtr(1);
	pros::Motor front_right_mtr(14);
	pros::Motor back_right_mtr(13);
	pros::Motor shooter1(15);
	pros::Motor shooter2(16);
	pros::Motor shooter3(17);
	pros::Motor roller(7); //temp port number, subject to change
	pros::Motor intake1_arm(9); //temp port number, subject to change
	pros::Motor intake2_arm(10); //temp port number, subject to change

	front_right_mtr.set_reversed(true);
	back_right_mtr.set_reversed(true);
	
	enum SET_SPEEDS{ZERO = 0, QUARTER = 127/4, HALF = 127/2, THREE_QUARTERS = (int)(0.75 * 127), MAX = 127}; //25%, 50%, 75%, 100%

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
						 
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		static bool toggle[2] {{false}}; //Intake at 0, Shooter at 1
		static int speeds[2] {{SET_SPEEDS(ZERO)}}; 

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			//toggles speed between values in SET_SPEEDS enum
			speeds[1] = speeds[1] == 0 ? 31 : (speeds[1] + 32) % 159;
		}

		//Shooter Controls
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { //If R1 gets pressed
			//pros::lcd::print(5, "New button press: R2 %d", !toggle[0]);
			toggle[0] = !toggle[0];

			if (toggle[0]) {
				shooter1 = speeds[0];
				shooter2 = speeds[0];
				shooter3 = speeds[0];
			}
		}

		// Intake Roller
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) { //If R1 gets pressed
			pros::lcd::print(5, "New button press: L2 %d", !toggle[1]);
			toggle[1] = !toggle[1];

			if (toggle[1]) {
				roller = speeds[1]; 
			}
		}

		// Intake Arms
		int dir = 0;
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){ //Going Down
			//pros::lcd::print(5, "New button press: L2 %d", !toggle[1]);
			intake1_arm.move(SET_SPEEDS(HALF));
			intake2_arm.move(-SET_SPEEDS(HALF));
			dir = 1;
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){ //Going Up
			//pros::lcd::print(5, "New button press: L2 %d", !toggle[1]);
			intake1_arm.move(-SET_SPEEDS(MAX));
			intake2_arm.move(SET_SPEEDS(MAX));
			dir -1;
		}

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

		//For Drive Train
		front_left_mtr = left;
		back_left_mtr = left;
		front_right_mtr = right;
		back_right_mtr = right;

		
		pros::lcd::print(0, "Right: %d", right);
		pros::lcd::print(1, "Left: %d", left);
		pros::delay(20);
	}
}