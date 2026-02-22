#include "main.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>


// motor groups
pros::MotorGroup leftMotors({-10, -2, -3}, pros::MotorGearset::blue); // left motor group
pros::MotorGroup rightMotors({18, 17, 13}, pros::MotorGearset::blue); // right motor group

pros::Imu imu(12);

// tracking wheels
pros::Rotation verticalEnc(-1);

lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0.8);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              11.5,
                              lemlib::Omniwheel::NEW_325,
                              480,
                              8 
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, 
                            nullptr,
                            nullptr, 
                            nullptr, 
                            &imu
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

pros::Motor lever(-14, pros::v5::MotorGears::red);
pros::Motor intake(-9, pros::v5::MotorGears::blue);

//pneumatics
pros::adi::Pneumatics matchloader('H', false);
pros::adi::Pneumatics storage('F', false);
pros::adi::Pneumatics wing('E', true);
pros::adi::Pneumatics park('G', false);

//rotation sensor
pros::Rotation rotationsensor(11);

void setIntake(int intakePower){
        intake.move(intakePower);
}

bool isMatchloaderExtended = false;
bool isStorageExtended = false;
bool isWingExtended = true;
bool isParkExtended = false;
bool isHalf = false;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

const int numStates = 2;
int states[numStates] = {0, 180};
int currState = 0;
int target = 0;
bool leverPID = false;

void nextState() {
    currState += 1;
    if (currState == 2) {
        currState = 0;
    }
    if (isStorageExtended) {
        if (currState == 1) {
            if (isHalf) {
                target = states[currState] - 115;
            }
            else {
                target = states[currState] - 50;

            }
        }
        else {
            target = states[currState];
        }
        
    }
    else {
        target = states[currState];
    }
}

void leverControl() {
  if (leverPID) {
    if (isStorageExtended) {
    double kp = 2.9;
    lever.move(kp * (target - (rotationsensor.get_position()/100.0)));
    }
    else {
    double kp = 0.7;
    lever.move((kp * (target - (rotationsensor.get_position()/100.0))));
    }
    }
}

//distance sensor
pros::Distance leftdistance(19);
pros::Distance rightdistance(16);
pros::Distance frontdistance(20);

double leftvalue;
double rightvalue;
double frontvalue;

void updateDistanceValues() {
        leftvalue = 72 - ((leftdistance.get_distance()/25.4) + 5.25);
        rightvalue = 72 - ((rightdistance.get_distance()/25.4) + 5.25);
        frontvalue = 72 - ((frontdistance.get_distance()/25.4) + 7);
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    chassis.calibrate(); // calibrate sensors
    pros::lcd::initialize(); // initialize lcd

    pros::Task leverTask([]{
        while (true) {
            leverControl();
            pros::delay(10);
        }
    });

    pros::Task distanceTask([]{
        while (true) {
            updateDistanceValues();
            pros::delay(10);
        }
    });
   
}

/**
 * Runs while the robot is disabled
 */

void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */

void competition_initialize() {}


void soloAWP() {
	chassis.setPose(-1, -46.75, -90);
	setIntake(127);
	storage.extend();
    isStorageExtended = true;
	chassis.moveToPoint(-4, -46.75, 2000, {.minSpeed = 35, .earlyExitRange = 8});
	chassis.moveToPoint(39, -46.75, 2000, {.forwards = false, .maxSpeed = 80});
	chassis.waitUntilDone();
	matchloader.extend();
	chassis.turnToHeading(-175, 500);
	chassis.waitUntilDone();
	chassis.setPose(leftvalue, -55, -180);
	chassis.moveToPoint(48, -67, 1500, {.maxSpeed = 60});
	chassis.waitUntilDone();
	chassis.moveToPoint(50, -35.5, 1500, {.forwards = false, .maxSpeed = 75});
	matchloader.retract();
	wing.retract();
	chassis.waitUntilDone();
	chassis.setPose(49, -29.2, chassis.getPose().theta);
	leverPID = true;
	nextState();
	pros::delay(700);
	setIntake(-127);
	chassis.swingToHeading(-45, DriveSide::RIGHT, 2000, {.minSpeed = 20, .earlyExitRange = 5});
	nextState();
	chassis.moveToPose(24, -18, -90, 2000, {.maxSpeed = 95, .minSpeed = 20, .earlyExitRange = 4});
	setIntake(127);
	wing.extend();
	chassis.moveToPose(-24, -18, -90, 2000, {.maxSpeed = 95, .minSpeed = 20, .earlyExitRange = 4});
	chassis.waitUntil(40);
	matchloader.extend();
	chassis.moveToPose(-9, -8, -135, 1000, {.forwards = false, .maxSpeed = 70});
	isStorageExtended = false;
    matchloader.retract();
    storage.retract();
	wing.retract();
	chassis.waitUntilDone();
	leverPID = true;
	nextState();
	pros::delay(500);
	setIntake(-127);
	nextState();
    isStorageExtended = true;
	storage.extend();
	wing.extend();
    matchloader.extend();
	chassis.moveToPoint(-40.5, -40, 2000, {.minSpeed = 20, .earlyExitRange = 8});
	setIntake(127);
	chassis.waitUntilDone();
	chassis.turnToHeading(-173, 500);
	chassis.waitUntilDone();
	chassis.setPose(-rightvalue, -53, -180);
	chassis.moveToPoint(-49, -68, 1000, {.maxSpeed = 60});
	chassis.waitUntilDone();
	chassis.moveToPoint(-47.5, -31, 500, {.forwards = false, .maxSpeed = 75});
	matchloader.retract();
	wing.retract();
	chassis.waitUntilDone();
	leverPID = true;
	nextState();
	setIntake(-127);
}

void right() {
    chassis.setPose(-1, -46.75, -90);
	setIntake(127);
	storage.extend();
    isStorageExtended = true;
	chassis.moveToPoint(39, -46.75, 2000, {.forwards = false, .maxSpeed = 80});
	chassis.waitUntilDone();
	matchloader.extend();
	chassis.turnToHeading(-175, 500);
	chassis.waitUntilDone();
	chassis.setPose(leftvalue, -55, -180);
	chassis.moveToPoint(48, -67, 1000, {.maxSpeed = 60});
	chassis.waitUntilDone();
	chassis.moveToPoint(50, -35.5, 1500, {.forwards = false, .maxSpeed = 75});
	matchloader.retract();
	wing.retract();
	chassis.waitUntilDone();
	chassis.setPose(49, -29.2, chassis.getPose().theta);
	leverPID = true;
	nextState();
	pros::delay(300);
	setIntake(-127);
    pros::delay(400);
	chassis.swingToHeading(-33, DriveSide::RIGHT, 2000, {.minSpeed = 20, .earlyExitRange = 5});
	nextState();
    // go for middle 2 blocks before score middle
    chassis.moveToPoint(22, -22, 2000, {.minSpeed = 20, .earlyExitRange = 4});
    wing.extend();
    setIntake(127);
    chassis.waitUntil(5);
    matchloader.extend();
    chassis.waitUntilDone();
    chassis.moveToPose(16, -6.5, -45, 2000, {.maxSpeed = 60}); //this is original if straight to middle
    /* contest 2 in middle under goal
    chassis.turnToHeading(60, 1000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(50);
    matchloader.retract();
    chassis.moveToPoint(48, -5, 2000, {.maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(18);
    matchloader.extend();
    chassis.moveToPoint(31, -13, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 5});
    chassis.turnToHeading(-35, 1000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(20);
    matchloader.retract();
    chassis.moveToPoint(22, -9.6, 2000, {.maxSpeed = 60});
    */
    chassis.waitUntil(5);
    matchloader.retract();
    chassis.waitUntilDone();
    chassis.setPose(15.2, -10.7, chassis.getPose().theta);
    park.extend();
    setIntake(-90);
    pros::delay(1000);
    chassis.moveToPoint(35, -25, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.turnToHeading(20, 1000);
    wing.retract();
    park.retract();
    chassis.waitUntilDone();
    chassis.moveToPoint(42, -18, 2000, {.maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 5});
    chassis.turnToHeading(-15, 1000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.moveToPoint(42, -10, 2000, {.maxSpeed = 50});
    chassis.waitUntilDone();
    chassis.turnToHeading(-35, 1500);

}

void left() {
    chassis.setPose(1, -46.75, 90);
    setIntake(127);
    storage.extend();
    isStorageExtended = true;
    chassis.moveToPoint(-40, -46.75, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    matchloader.extend();
    chassis.turnToHeading(175, 500);
    chassis.waitUntilDone();
    chassis.setPose(-rightvalue, -55, 180);
    chassis.moveToPoint(-47.5, -69, 1500, {.maxSpeed = 60});
	chassis.waitUntilDone();
	pros::delay(200); // could save time for future if matchloading is faster
	chassis.moveToPoint(-51, -35.5, 3000, {.forwards = false, .maxSpeed = 75});
	matchloader.retract();
	wing.retract();
	chassis.waitUntilDone();
	chassis.setPose(-49, -29.2, chassis.getPose().theta);
	leverPID = true;
	nextState();
	pros::delay(300);
	setIntake(-127);
    pros::delay(400);
	chassis.swingToHeading(30, DriveSide::LEFT, 2000, {.minSpeed = 20, .earlyExitRange = 5});
	nextState();
    chassis.moveToPoint(-26, -21, 2000, {.minSpeed = 20, .earlyExitRange = 4});
    wing.extend();
    setIntake(127);
    chassis.waitUntil(3);
    matchloader.extend();
    chassis.turnToHeading(-60, 1000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(50);
    matchloader.retract();
    chassis.moveToPoint(-49, -5, 2000, {.maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(18);
    matchloader.extend();
    chassis.moveToPoint(-27.5, -13, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 5});
    storage.retract();
    isStorageExtended = false;
    chassis.turnToHeading(-127, 1000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(20);
    matchloader.retract();
    chassis.moveToPoint(-16, -8, 700, {.forwards = false, .maxSpeed = 60});
    wing.retract();
    chassis.waitUntilDone();
    chassis.setPose(-15.2, -10.7, chassis.getPose().theta);
    nextState();
    pros::delay(1000);
    setIntake(-127);
    nextState();
    chassis.moveToPoint(-38, -26, 2000, {.maxSpeed = 70});
    setIntake(127);
    storage.extend();
    wing.extend();
    isStorageExtended = true;
    chassis.waitUntilDone();
    chassis.turnToHeading(165, 1000);
    wing.retract();
    chassis.waitUntilDone();
    chassis.moveToPoint(-42, -18, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 5});
    chassis.turnToHeading(175, 1000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.moveToPoint(-40, -8, 2000, {.forwards = false, .maxSpeed = 50});
    chassis.waitUntilDone();
}

void leftrush() {
    chassis.setPose(-18, -50.5, 0);
    setIntake(127);
    storage.extend();
    isStorageExtended = true;
    chassis.moveToPoint(-25, -25, 2000, {.maxSpeed = 90, .minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(12);
    matchloader.extend();
    chassis.moveToPoint(-48.5, -45, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntil(15);
    matchloader.retract();
    chassis.waitUntil(40);
    matchloader.extend();
    chassis.waitUntilDone();
    chassis.turnToHeading(-175, 800);
    chassis.waitUntilDone();
    chassis.setPose(-rightvalue, -55, -180);
    chassis.moveToPoint(-48, -74, 1500, {.maxSpeed = 45});
    chassis.waitUntilDone();
    pros::delay(400); // could save time for future if matchloading is faster
    chassis.moveToPoint(-51, -37.5, 1000, {.forwards = false, .maxSpeed = 65});
    matchloader.retract();
    chassis.waitUntilDone();
    wing.retract();
    pros::delay(200);
    chassis.setPose(-49, -29.2, chassis.getPose().theta);
    leverPID = true;
    nextState();
    pros::delay(300);
    setIntake(-127);
    pros::delay(400);
    chassis.moveToPoint(-39, -35, 1000, {.maxSpeed = 75});
    nextState();
    chassis.turnToHeading(175, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-36.5, -11, 10000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
}   

void rightrush() {
    chassis.setPose(18, -50.5, 0);
    setIntake(127);
    storage.extend();
    isStorageExtended = true;
    chassis.moveToPoint(25, -25, 2000, {.maxSpeed = 90, .minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(12);
    matchloader.extend();
    chassis.moveToPoint(48.5, -45, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntil(15);
    matchloader.retract();
    chassis.waitUntil(40);
    matchloader.extend();
    chassis.waitUntilDone();
    chassis.turnToHeading(-175, 800);
    chassis.waitUntilDone();
    chassis.setPose(leftvalue, -55, -180);
    chassis.moveToPoint(48, -74, 1500, {.maxSpeed = 45});
    chassis.waitUntilDone();
    pros::delay(400); // could save time for future if matchloading is faster
    chassis.moveToPoint(51, -37.5, 1000, {.forwards = false, .maxSpeed = 65});
    matchloader.retract();
    chassis.waitUntilDone();
    wing.retract();
    pros::delay(200);
    chassis.setPose(49, -29.2, chassis.getPose().theta);
    leverPID = true;
    nextState();
    pros::delay(500);
    setIntake(-127);
    pros::delay(400);
    chassis.moveToPoint(59, -35, 1000, {.maxSpeed = 75});
    nextState();
    chassis.turnToHeading(175, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(61.5, -11, 10000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
}

void skills() {
    chassis.setPose(-1, -46.75, -90);
    setIntake(127);
    storage.extend();
    isStorageExtended = true;
    chassis.moveToPoint(40, -46.75, 2000, {.forwards = false, .maxSpeed = 75});
    chassis.waitUntilDone();
    matchloader.extend();
    chassis.turnToHeading(-175, 500);
    chassis.waitUntilDone();
    chassis.setPose(leftvalue, -55, -180);
    // matchload
    chassis.moveToPoint(48, -68, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(500); // could save time for future if matchloading is faster
    chassis.moveToPoint(48, -67.5, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(48, -67.5, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(48, -55, 3000, {.forwards = false, .maxSpeed = 75});
    chassis.waitUntilDone();
    setIntake(0);
    chassis.turnToHeading(-150, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(64, -45, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 8});
    chassis.moveToPose(66, 30, 180, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 8});
    chassis.turnToHeading(-100, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(54, 30, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(-10, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(54, 10, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    wing.retract();
    chassis.setPose(rightvalue, 35.5, chassis.getPose().theta);
    leverPID = true;
    setIntake(127);
    pros::delay(500);
    nextState();
    pros::delay(1000);
    setIntake(-127);
    nextState();
    pros::delay(200);
    setIntake(127);
    nextState();
    pros::delay(1000);
    setIntake(-127);
    nextState();
    chassis.moveToPoint(49, 68, 1500, {.maxSpeed = 40});
    setIntake(127);
    chassis.waitUntil(5);
    wing.extend();
    chassis.waitUntilDone();
    pros::delay(500); // could save time for future if matchloading is faster
    chassis.moveToPoint(49, 69, 1500, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(49, 69, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(49, 33.5, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    wing.retract();
    pros::delay(500);
    leverPID = true;
    nextState();
    matchloader.retract();
    pros::delay(1000);
    setIntake(-127);
    nextState();
    pros::delay(200);
    setIntake(127);
    nextState();
    pros::delay(1000);
    setIntake(-127);
    nextState();
    chassis.setPose(49, 29.2, chassis.getPose().theta);
    setIntake(0);
    chassis.moveToPoint(49, 40, 2000);
    matchloader.extend();
    chassis.waitUntil(5);
    wing.extend();
    /*
    // clear blue park zone
    matchloader.retract();
    chassis.waitUntilDone();
    chassis.turnToHeading(-40, 1000);
    chassis.waitUntilDone();
    chassis.moveToPose(22, 63, -90, 2000);
    chassis.waitUntilDone();
    matchloader.extend();
    pros::delay(700);
    chassis.moveToPose(-10, 63, -90, 4000, {.maxSpeed = 100});
    chassis.waitUntilDone();
    chassis.setPose(-frontvalue, rightvalue, -90);
    chassis.turnToHeading(-135, 1000);
    chassis.waitUntilDone();
    */
    chassis.moveToPoint(-46, 45, 3000, {.maxSpeed = 85});
    chassis.waitUntilDone();
    chassis.turnToHeading(-5, 500);
    chassis.waitUntilDone();
    setIntake(127);
    chassis.setPose(-leftvalue, 55, 0);
    chassis.moveToPoint(-48, 75, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(500); // could save time for future if matchloading is faster
    chassis.moveToPoint(-48, 75, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(-48, 75, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(-49, 55, 2000, {.forwards = false, .maxSpeed = 75});
    chassis.waitUntilDone();
    setIntake(0);
    chassis.turnToHeading(25, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(-64, 45, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 8});
    chassis.moveToPose(-66, -32.5, 0, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 8});
    chassis.turnToHeading(75, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-54, -30, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(175, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-54, -10, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    wing.retract();
    setIntake(127);
    pros::delay(500);
    chassis.setPose(-49, -35.5, chassis.getPose().theta);
    leverPID = true;
    nextState();
    pros::delay(1000);
    setIntake(-127);
    nextState();
    pros::delay(200);
    setIntake(127);
    nextState();
    pros::delay(1000);
    setIntake(-127);
    nextState();
    chassis.moveToPoint(-48.5, -69, 1000, {.maxSpeed = 40});
    setIntake(127);
    chassis.waitUntil(5);
    wing.extend();
    chassis.waitUntilDone();
    pros::delay(500); // could save time for future if matchloading is faster
    chassis.moveToPoint(-48.5, -69, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(-48.5, -69, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPoint(-49, -35.5, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    matchloader.retract();
    wing.retract();
    pros::delay(500);
    leverPID = true;
    nextState();
    pros::delay(1000);
    setIntake(-127);
    nextState();
    pros::delay(200);
    setIntake(127);
    nextState();
    pros::delay(1000);
    setIntake(-127);
    nextState();
    chassis.setPose(-49, -29.2, chassis.getPose().theta);
    // clear red park zone and park
    /* diagnoal then curve
    chassis.moveToPoint(-35, -55, 2000, {.maxSpeed = 60});
    setIntake(0);
    chassis.waitUntil(5);
    wing.extend();
    chassis.waitUntilDone();
    chassis.moveToPose(-24, -68, 90, 2000);
    chassis.waitUntilDone();
    */
    // curve but with smaller lead
    chassis.moveToPose(-26, -68, 90, 2000, {.lead = 0.3});
    chassis.waitUntilDone();
    setIntake(127);
    matchloader.extend();
    pros::delay(700);
    chassis.moveToPoint(-5, -68, 5000);
    chassis.waitUntilDone();
    matchloader.retract();

}

void autonomous() {
    soloAWP();
}

const double driveConst = 2.0;

double computeY(double x, double t) {
    double exp_neg_t = std::exp(-t / 10.0);
    double exp_term = std::exp((std::abs(x) - 127.0) / 10.0); // absolute value using std::abs
    double y = (exp_neg_t + exp_term * (1.0 - exp_neg_t)) * x;
    return y;
}

void opcontrol() {
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    pros::Controller master(pros::E_CONTROLLER_MASTER);

    // loop forever
    while (true) {
        // drive
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        leftMotors.move(computeY(leftY, driveConst));
        rightMotors.move(computeY(rightY, driveConst));



        setIntake((controller.get_digital(DIGITAL_R1) - controller.get_digital(DIGITAL_R2)) * 127);

        if (master.get_digital_new_press(DIGITAL_L2)) {
            leverPID = true;
            nextState();
            }

        if(master.get_digital_new_press(DIGITAL_L1)) {
            if(isStorageExtended) {
                storage.retract();
                isStorageExtended = false;
            }
            else {
                storage.extend();
                isStorageExtended = true;
            }
        }

         if(master.get_digital_new_press(DIGITAL_RIGHT)) {
            if(isMatchloaderExtended) {
                matchloader.retract();
                isMatchloaderExtended = false;
            }
            else {
                matchloader.extend();
                isMatchloaderExtended = true;
            }
        }

         if(master.get_digital_new_press(DIGITAL_Y)) {
            if(isWingExtended) {
                wing.retract();
                isWingExtended = false;
            }
            else {
                wing.extend();
                isWingExtended = true;
            }
        }

        if(master.get_digital_new_press(DIGITAL_A)) {
            if(isParkExtended) {
                park.retract();
                isParkExtended = false;
            }
            else {
                park.extend();
                isParkExtended = true;
            }
        }

        if(master.get_digital_new_press(DIGITAL_LEFT)) {
            if(isHalf) {
                isHalf = false;
            }
            else {
                isHalf = true;
            }
        }
            
        // delay to save resources
        pros::delay(25);
    }
}