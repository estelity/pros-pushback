#include "main.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include <cmath>

int autonSelected = 0;
// 0=left
// 1=right
// 2=skills

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({12, 13, 14});    // Creates a motor group with ports 12, 13, 14
pros::MotorGroup right_mg({17, 18, 19});  // Creates a motor group with ports 17, 18, 19

pros::Motor bottom_intake(5);
pros::Motor top_intake(4);

pros::ADIDigitalOut solenoidA('A');
pros::ADIDigitalOut solenoidB('B');

bool loaderDown = false;
bool loaderDown2 = false;

pros::IMU imu(1);
pros::Rotation odom(2);

double x = 0.0;      // field X position in inches
double y = 0.0;      // field Y position in inches
double theta = 0.0;  // heading in radians

const double WHEEL_DIAMETER = 2.00;  // inches
const double WHEEL_CIRC = WHEEL_DIAMETER * M_PI;

double degToInches(double deg) {
    return (deg / 360.0) * WHEEL_CIRC;
}

void odomTask() {
    double lastOdomDeg = odom.get_position(); // initial reading

    while(true) {
        // 1. Get odom wheel movement
        double currDeg = odom.get_position();
        double deltaDeg = currDeg - lastOdomDeg;
        lastOdomDeg = currDeg;

        double deltaDist = degToInches(deltaDeg);

        // 2. Get heading from IMU
        theta = imu.get_rotation() * M_PI / 180.0; // convert deg → rad

        // 3. Project onto field coordinates
        double dx = deltaDist * cos(theta);
        double dy = deltaDist * sin(theta);

        // 4. Update global position
        x += dx;
        y += dy;

        pros::delay(10); // 100 Hz update
    }
}

void displayOdom() {
    pros::lcd::print(6, "X: %.2f", x);
    pros::lcd::print(7, "Y: %.2f", y);
    pros::lcd::print(8, "Theta: %.2f deg", theta * 180.0 / M_PI);
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	// static bool pressed = false;
	// pressed = !pressed;
	// if (pressed) {
	// 	pros::lcd::set_text(2, "I was pressed!");
	// } else {
	// 	pros::lcd::clear_line(2);
	// }

	autonSelected = (autonSelected + 1) % 3;

    switch (autonSelected) {
        case 0:
            pros::lcd::set_text(2, "Auton: LEFT");
            break;
        case 1:
            pros::lcd::set_text(2, "Auton: RIGHT");
            break;
        case 2:
            pros::lcd::set_text(2, "Auton: SKILLS");
            break;
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// pros::lcd::initialize();
	// pros::lcd::set_text(1, "Hello PROS User!");

	// pros::lcd::register_btn1_cb(on_center_button);

	pros::lcd::initialize();
    pros::lcd::set_text(1, "Auton Selector");
    pros::lcd::set_text(2, "Auton: LEFT");

    pros::lcd::register_btn1_cb(on_center_button);

	pros::lcd::register_btn0_cb(autonomous);

    // IMU needs calibration
    imu.reset();
    pros::delay(2000); // wait for calibration

    odom.reset_position();

    // Start odometry task
    pros::Task odom_background(odomTask);
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
 * drive functions start here
 */
void driveForward(){}

void turnLeft(){}

void turnRight(){}

void driveBackward(){}

void intakeIn(){}

void intakeOut(){}

void intakeStop(){}

void topUnload(){}

void topStop(){}

void driveStop(){}

/**
 * auton selections
 */
void autonSkills(){
	pros::lcd::set_text(4, "skills");
}

void autonLeft(){
	pros::lcd::set_text(4, "left");
}

void autonRight(){
	pros::lcd::set_text(4, "right");
}
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
	switch (autonSelected) {
        case 0:
            autonLeft();
            break;
        case 1:
            autonRight();
            break;
        case 2:
            autonSkills();
            break;
    }
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

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		pros::lcd::set_text(4, "driver");

		// Arcade control scheme
		int forward = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = - (master.get_analog(ANALOG_RIGHT_X) * 0.75);  // Gets the turn left/right from right joystick
		right_mg.move((turn + forward));                      // Sets left motor voltage
		left_mg.move((turn - forward));                     // Sets right motor voltage

		// intakes and penumatics
		int direction = 1;
		int direction2 = 1;
		int top_speed = 0;
		int bottom_speed = 0;

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			direction = 1;
			direction2 = -1;
			bottom_speed = 127;
			top_speed = 127;
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			direction = -1;
			direction2 = 1;
			top_speed = 127;
			bottom_speed = 127;
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			direction2 = 1;
			bottom_speed = 127;
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			direction2 = -1;
			bottom_speed = 127;
		}

		bottom_intake.move(direction2*(bottom_speed));
		top_intake.move(direction*(top_speed));

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			loaderDown = !loaderDown;
			solenoidA.set_value(loaderDown);
			while (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
				pros::delay(10);
			}
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			loaderDown2 = !loaderDown2;
			solenoidB.set_value(loaderDown2);
			while (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
				pros::delay(10);
			}
		}

		pros::delay(20);                               // Run for 20 ms then update
	}
}