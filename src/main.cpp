#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;

enum Direction {right, left};
Direction r = right;
Direction l = left;

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
	return percent * 12000;
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
	double kp = fmin(angle / 57, 1.5);
	double kd = 0.2;
	double past_error = 0;

	int slew_count = 0;
	int step = 5;

	while (error > threshold && slew_count * step < timeout) {
		error = fabs(angle) - fabs(inertial.getRemapped(-180, 180));
		power = kp * error;
		power = c(-100, 100, power);
		power = slew(slew_rate, slew_count, power, 35);
		power = power + kd * (error - past_error);

		left.moveVoltage(120 * power);
		right.moveVoltage(-120 * power);

		pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "error: %f, power: %f", error, power);
		pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2, "angle: %f", fabs(inertial.getRemapped(-180, 180) < -90 ? inertial.getRemapped(-180, 180) + 360 : inertial.getRemapped(-180, 180)));
		slew_count++;
		past_error = error;

		pros::delay(step);
	}
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
	double kp = fmin(angle / 57, 1.5);
	double past_error = 0;
	double kd = 0.5;

	int slew_count = 0;
	int step = 5;

	while (fabs(error) > threshold && slew_count * step < timeout) {
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
}
void turn(double angle, Direction direction, double p = 1.0, double d = 0.25, double slew_rate = 0.4, double threshold = 1, double timeout = 3000) {
	if (direction == r) {
		turn_right(angle, slew_rate, threshold, timeout);
	} else {
		turn_left(angle, slew_rate, threshold, timeout);
	}} //turn right = true

int disc_count() {
	return 0;
}

void intake_task(void*) {
	ControllerButton R1(ControllerDigital::R1);
	ControllerButton X(ControllerDigital::X);
	Motor intake(-9);
	bool active = false;

	while (true) {
		if (R1.changedToPressed()) {
			active = !active;
		}

		if (!X.isPressed()) {
			intake.moveVoltage(12000 * (active ? 1 : -1));
		} else {
			intake.moveVoltage(-12000);
		}
	}
}
void flywheel_task(void*) {
	ControllerButton R2(ControllerDigital::R2);
	Motor flywheel(13);

	double idle = 0.15;
	double ready = 0.5;

	int slew_counter = 0;

	flywheel.moveVoltage(ptv(idle));
	while (true) {
		if (disc_count() > 0) {
			flywheel.moveVoltage(ready);
		}
		pros::delay(10);
	}
}

void indexer_task(void*) {
	ControllerButton L1(ControllerDigital::L1);
	pros::ADIPort indexer('A', pros::E_ADI_DIGITAL_OUT);
	double rate = 3;

	indexer.set_value(false);

	while (true) {
		if (L1.isPressed()) {
			for (int i = 0; i < 3; i++) {
				indexer.set_value(true);
				pros::delay(1000 / rate * 0.3);
				indexer.set_value(false);
				pros::delay(1000 / rate * 0.7);
			}
		}
		pros::delay(10);
	}
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
	turn(180, r);
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
	Controller controller;
	ControllerButton R1(ControllerDigital::R1);
	ControllerButton R2(ControllerDigital::R2);
	ControllerButton L1(ControllerDigital::L1);
	Motor intake(-9);
	Motor flywheel(13);
	pros::ADIPort indexer('A', pros::E_ADI_DIGITAL_OUT);
	double left = 0;
	double right = 0;

	IMU inertial(16, IMUAxes::z);
	inertial.reset();

	bool flywheel_active = false;
	bool pneumatics_active = false;

	pros::Task run_indexer(indexer_task);

	//6 motor drive, 257 rpm, 4" omniss
	std::shared_ptr<ChassisController> drive =
		ChassisControllerBuilder()
			.withMotors({-3, -11, -12}, {10, 18, 19})
			.withDimensions({AbstractMotor::gearset::blue, (36.0 / 84.0)}, {{4.125_in, 16_in}, imev5BlueTPR})
			.build();

	drive->getModel()->setBrakeMode(AbstractMotor::brakeMode::brake);

	while (true) {
		if (R2.changedToPressed()) {
			flywheel_active = !flywheel_active;
		}

		if (flywheel_active) {
			flywheel.moveVoltage(12000);
		} else {
			flywheel.moveVoltage(0);
		}

		left = controller.getAnalog(ControllerAnalog::leftY);
		right = controller.getAnalog(ControllerAnalog::rightY);

		drive->getModel()->tank(controller_map(left), controller_map(right));
		pros::delay(10);

		intake.moveVelocity(R1.isPressed() ? 200 : 0);
	}
}
