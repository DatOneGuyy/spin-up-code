#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;

enum Direction {right, left};
Direction r = right;
Direction l = left;
double flywheel_speed = 0;
double target_speed = 0;

double controller_map(double i) {
		double sign = i == 0 ? 0 : fabs(i) / i;
    double input = fabs(i * 100);
    double output = 0;

    if (input < 5) {
        output = 0;
    } else if (input < 100.0 / 1.3) {
        output = input * 1.3;
    } else {
        output = 100;
    }

    return output / 100 * sign;
}
double ptv(double percent) {
	return percent * 120;
}
double avg(double a, double b) {
  return (a + b) / 2;
}
double c(double min, double max, double value) {
  if (value < min) {
    return min;
  } else if (value > max) {
    return max;
  } else {
    return value;
  }
}

double slew(double rate, int count, double target, double base) {
	if (fabs(base + rate * count) < fabs(target)) {
		return base + rate * count;
	} else {
		return target;
	}
}

void forward(double distance, double p = 0.15, double g = 1.1, double slew_rate = 0.3, double threshold = 5, double timeout = 3000) {
	MotorGroup left({-3, -11, -12});
	MotorGroup right({10, 18, 19});

	left.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
	right.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
	left.setBrakeMode(AbstractMotor::brakeMode::hold);
	right.setBrakeMode(AbstractMotor::brakeMode::hold);
	left.tarePosition();
	right.tarePosition();

	IMU inertial(16, IMUAxes::z);
	inertial.reset();

	double error = distance;
	double power = 0;
	double kp = p;
	double kg = g;
	double angle = 0;

	int slew_count = 0;
	int step = 5;

	while (fabs(error) > threshold && step * slew_count < timeout) {
		error = distance - avg(left.getPosition(), right.getPosition());
		angle = inertial.getRemapped();
		power = kp * error;
		power = c(-20, 100, power);
		power = slew(slew_rate, slew_count, power, 35);

		if (error > 200) {
			left.moveVoltage(120 * (power - angle * kg));
			right.moveVoltage(120 * (power + angle * kg));
		} else {
			left.moveVoltage(120 * (power - angle * kg * fabs(error) / 200));
			right.moveVoltage(120 * (power + angle * kg * fabs(error) / 200));
		}
		slew_count++;

		if (120 * (power - angle * kg) < 5) {
			break;
		}

		pros::delay(step);
	}

	left.moveVelocity(0);
	right.moveVelocity(0);
}
void backward(double distance, double p = 0.15, double g = 1.1, double slew_rate = 0.3, double threshold = 5, double timeout = 3000) {
	MotorGroup left({-3, -11, -12});
	MotorGroup right({10, 18, 19});

	left.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
	right.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
	left.setBrakeMode(AbstractMotor::brakeMode::hold);
	right.setBrakeMode(AbstractMotor::brakeMode::hold);
	left.tarePosition();
	right.tarePosition();

	IMU inertial(16, IMUAxes::z);
	inertial.reset();

	double error = -distance;
	double power = 0;
	double kp = p;
	double kg = g;
	double angle = 0;

	int slew_count = 0;
	int step = 5;

	while (fabs(error) > threshold && step * slew_count < timeout) {
		error = -distance - avg(left.getPosition(), right.getPosition());
		angle = inertial.getRemapped();
		power = kp * error;
		power = c(-100, 20, power);
		power = slew(-slew_rate, slew_count, power, -35);

		if (error < -200) {
			left.moveVoltage(120 * (power - angle * kg));
			right.moveVoltage(120 * (power + angle * kg));
		} else {
			left.moveVoltage(120 * (power - angle * kg * fabs(error) / 200));
			right.moveVoltage(120 * (power + angle * kg * fabs(error) / 200));
		}
		slew_count++;

		pros::delay(step);
	}

	left.moveVelocity(0);
	right.moveVelocity(0);
}
void drive(double distance, double p = 0.15, double g = 1.1, double slew_rate = 0.3, double threshold = 5, double timeout = 3000) {
	if (distance > 0) {
		forward(distance, p, g, slew_rate, threshold, timeout);
	} else {
		backward(-distance, p, g, slew_rate, threshold, timeout);
	}
}
void pressure(double time, double speed = 70) {
	MotorGroup left({-3, -11, -12});
	MotorGroup right({10, 18, 19});
	Motor intake(-9);

	left.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
	right.setEncoderUnits(AbstractMotor::encoderUnits::degrees);

	left.moveVoltage(-speed * 120);
	right.moveVoltage(-speed * 120);

	for (int i = 0; i < time / 10; i++) {
		intake.moveVoltage(-12000);
		pros::delay(10);
	}

	intake.moveVoltage(0);
	left.moveVoltage(0);
	right.moveVoltage(0);
}
void turn_right(double angle, double slew_rate = 0.6, double threshold = 2, double timeout = 3000) {
	MotorGroup left({-3, -11, -12});
	MotorGroup right({10, 18, 19});

	left.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
	right.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
	left.setBrakeMode(AbstractMotor::brakeMode::hold);
	right.setBrakeMode(AbstractMotor::brakeMode::hold);
	left.tarePosition();
	right.tarePosition();

	IMU inertial(16, IMUAxes::z);
	inertial.reset();

	double error = angle;
	double power = 0;
	double kp = fmax(fmin(angle / 57, 1.5), 0.6);
	double kd = 0;
	double past_error = 0;

	int slew_count = 0;
	int step = 2;

	while (error > threshold && slew_count * step < timeout) {
		error = angle - abs(inertial.getRemapped(-180, 180));
		power = kp * error;
		power = c(-100, 100, power);
		power = slew(slew_rate, slew_count, power, 35);
		power = fmax(power + kd * (error - past_error), 35);

		left.moveVoltage(120 * power);
		right.moveVoltage(-120 * power);

		pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "error: %f, power: %f", error, power);
		pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2, "angle: %f", fabs(inertial.getRemapped(-180, 180) < -90 ? inertial.getRemapped(-180, 180) + 360 : inertial.getRemapped(-180, 180)));
		slew_count++;
		past_error = error;

		pros::delay(step);
	}
	left.moveVoltage(0);
	right.moveVoltage(0);
}
void turn_left(double angle, double slew_rate = 0.6, double threshold = 2, double timeout = 3000) {
	MotorGroup left({-3, -11, -12});
	MotorGroup right({10, 18, 19});

	left.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
	right.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
	left.setBrakeMode(AbstractMotor::brakeMode::hold);
	right.setBrakeMode(AbstractMotor::brakeMode::hold);
	left.tarePosition();
	right.tarePosition();

	IMU inertial(16, IMUAxes::z);
	inertial.reset();

	double error = angle;
	double power = 0;
	double kp = fmax(fmin(angle / 57, 1.5), 0.6);
	double past_error = 0;
	double kd = 0;

	int slew_count = 0;
	int step = 2;

	while (error > threshold && slew_count * step < timeout) {
		error = angle - abs(inertial.getRemapped(-180, 180));
		power = kp * error;
		power = c(-100, 100, power);
		power = slew(slew_rate, slew_count, power, 35);
		power = power + kd * (error - past_error);

		left.moveVoltage(-120 * power);
		right.moveVoltage(120 * power);

		slew_count++;
		past_error = error;

		pros::delay(step);
	}

	left.moveVoltage(0);
	right.moveVoltage(0);
}
void turn(double angle, Direction direction, double p = 1.0, double d = 0.25, double slew_rate = 0.4, double threshold = 2, double timeout = 3000) {
	if (direction == r) {
		turn_right(angle, slew_rate, threshold, timeout);
	} else {
		turn_left(angle, slew_rate, threshold, timeout);
	}} //turn right = true
void toggle_intake(bool reverse = false) {
	Motor intake(-9);
	bool intake_active = false;

	if (intake.getActualVelocity() > 3) {
		intake_active = true;
	} else {
		intake_active = false;
	}

	if (intake_active) {
		intake_active = false;
		intake.moveVoltage(0);
	} else {
		intake_active = true;
		intake.moveVoltage(reverse ? -12000 : 12000);
	}
}
void start_intake() {
	Motor intake(-9);
	intake.moveVoltage(12000);
}
void stop_intake() {
	Motor intake(-9);
	intake.moveVoltage(0);
}
void reverse_intake() {
	Motor intake(-9);
	intake.moveVoltage(-12000);
}
void shoot(int count) {
	pros::ADIPort indexer('A', pros::E_ADI_DIGITAL_OUT);
	double shot_1, shot_2, shot_3;
	for (int i = 0; i < count; i++) {
		while (abs(flywheel_speed - target_speed) > 0.7) {
			pros::delay(10);
			pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "flywheel error: %f", abs(flywheel_speed - 80));
		}
		indexer.set_value(true);
		pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5 + i, "shot: %f", abs(flywheel_speed));

		pros::delay(100);
		indexer.set_value(false);
		pros::delay(400);
	}
}

void intake_task(void*) {
	ControllerButton R1(ControllerDigital::R1);
	ControllerButton R2(ControllerDigital::R2);
	Motor intake(-9);
	bool active = false;

	while (true) {
		if (R1.changedToPressed()) {
			active = !active;
		}

		if (!R2.isPressed()) {
			intake.moveVoltage(12000 * (active ? 1 : 0));
		} else if (R2.isPressed()) {
			intake.moveVoltage(-12000);
		} else {
			intake.moveVoltage(0);
		}
	}
}
void indexer_task(void*) {
	ControllerButton L1(ControllerDigital::L1);
	pros::ADIPort indexer('A', pros::E_ADI_DIGITAL_OUT);
	ControllerButton L2(ControllerDigital::L2);
	Motor flywheel(13);

	double rate = 3;

	bool flywheel_idle = true;
	double idle = 40;
	double active = 90;

	indexer.set_value(false);

	while (true) {
		if (flywheel_idle) {
			flywheel.moveVoltage(ptv(idle));
		} else {
			flywheel.moveVoltage(ptv(active));
		}

		if (L2.changedToPressed()) {
			flywheel_idle = !flywheel_idle;
		}
		if (L1.changedToPressed() && !flywheel_idle) {
			for (int i = 0; i < 3; i++) {
				indexer.set_value(true);
				flywheel.moveVoltage(ptv(100));
				pros::delay(1000 / rate * 0.3);
				indexer.set_value(false);
				pros::delay(1000 / rate * 0.7);
			}
			pros::delay(500);
			flywheel_idle = true;
			flywheel.moveVoltage(ptv(idle));
			pros::delay(500);
		}
		pros::delay(10);
	}
}
void flywheel_task(void*) {
	ControllerButton R2(ControllerDigital::R2);
	Motor flywheel(13);

	flywheel.setGearing(AbstractMotor::gearset::red);

	double target = target_speed;
	double speed = 0;
	double kp = 4;
	double error = target;
	double voltage = target * 120;

	while (true) {
		error = target - flywheel.getActualVelocity();
		pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 0, "error: %f, speed: %f, voltage: %f", error, flywheel.getActualVelocity(), voltage);
		voltage += kp * error;
		voltage = c(0, 12000, voltage);
		flywheel.moveVoltage(voltage);
		flywheel_speed = flywheel.getActualVelocity();
		pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "speed: %f, voltage: %f", flywheel.getActualVelocity(), voltage);
		pros::delay(10);
	}
}
void catapults_task(void*) {
	ControllerButton LEFT(ControllerDigital::left);
	pros::ADIPort catapults('B', pros::E_ADI_DIGITAL_OUT);
	catapults.set_value(false);

	bool pneumatics_extended = true;

	while (true) {
		if (LEFT.changedToPressed()) {
			catapults.set_value(pneumatics_extended);
			pneumatics_extended = !pneumatics_extended;
		}
		pros::delay(20);
	}
}
void drive_task(void*) {
	Controller controller;

	IMU inertial(16, IMUAxes::z);

	double left, right;

	std::shared_ptr<ChassisController> drive =
		ChassisControllerBuilder()
			.withMotors({-3, -11, -12}, {10, 18, 19})
			.withDimensions({AbstractMotor::gearset::blue, (36.0 / 84.0)}, {{4.125_in, 16_in}, imev5BlueTPR})
			.build();

	drive->getModel()->setBrakeMode(AbstractMotor::brakeMode::brake);

	while (true) {
		left = controller.getAnalog(ControllerAnalog::leftY);
		right = controller.getAnalog(ControllerAnalog::rightY);

		drive->getModel()->tank(controller_map(left), controller_map(right));

		pros::delay(3);
	}
}

void right_auton() {
	target_speed = 77;
	pros::Task flywheel_auton(flywheel_task);

	toggle_intake();
	drive(770);
	turn(17, r);
	pros::delay(600);
	shoot(3);
	toggle_intake();
	turn(68, l);
	target_speed = 40;
	drive(-950);
	turn(40, r);
	pressure(350, 50);
}
void left_auton() {
	target_speed = 79;
	pros::Task flywheel_auton(flywheel_task);

	pressure(200, 30);
	drive(200);
	turn(30, r);
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 8, "finished turn");
	drive(600);
	turn(90, l);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {}

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
	Direction auton = l;
	if (auton == r) {
		right_auton();
	} else {
		left_auton();
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
	IMU inertial(16, IMUAxes::z);
	inertial.reset();

	pros::Task run_indexer(indexer_task);
	pros::Task run_intake(intake_task);
	pros::Task run_drive(drive_task);
	pros::Task run_catapults(catapults_task);

	while (true) {
		pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 9, "angle: %f", inertial.getRemapped(-180, 180));
		pros::delay(15);
	}
}
