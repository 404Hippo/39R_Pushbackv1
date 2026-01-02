#include "main.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/rtos.hpp"


// motor groups
pros::MotorGroup leftMotors({-8, -9, -10}, pros::MotorGearset::blue); // left motor group
pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::blue); // right motor group

pros::Imu imu(7);

// tracking wheels
pros::Rotation verticalEnc(-6);

lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0.8);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 11.5 inch track width
                              lemlib::Omniwheel::NEW_325,
                              480, // drivetrain rpm is 450
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
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel 1, set to nullptr as we don't have one
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have one
                            &imu // inertial sensor
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

pros::Motor lever(-11, pros::v5::MotorGears::red);
pros::Motor intake(-19, pros::v5::MotorGears::blue);

//pneumatics
pros::adi::Pneumatics matchloader('F', false);
pros::adi::Pneumatics storage('H', false);
pros::adi::Pneumatics wing('G', true);
pros::adi::Pneumatics park('E', false);

//rotation sensor
pros::Rotation rotationsensor(20);

void setIntake(int intakePower){
        intake.move(intakePower);
}

bool isMatchloaderExtended = false;
bool isStorageExtended = false;
bool isWingExtended = true;
bool isParkExtended = false;

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
            target = states[currState] - 50;
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
pros::Distance leftdistance(5);
pros::Distance rightdistance(4);
pros::Distance frontdistance(17);

double leftvalue;
double rightvalue;
double frontvalue;


void updateDistanceValues() {
    while (true) {
        leftvalue = 72 - ((leftdistance.get_distance()/25.4) + 5.25);
        rightvalue = 72 - ((rightdistance.get_distance()/25.4) + 5.25);
        frontvalue = 72 - ((frontdistance.get_distance()/25.4) + 13);
    }
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
	chassis.moveToPoint(40.5, -46.75, 2000, {.forwards = false, .maxSpeed = 80});
	chassis.waitUntilDone();
	matchloader.extend();
	chassis.turnToHeading(-175, 500);
	chassis.waitUntilDone();
	chassis.setPose(leftvalue, -55, -180);
	chassis.moveToPoint(48.5, -66, 1500, {.maxSpeed = 60});
	chassis.waitUntilDone();
	pros::delay(200); // could save time for future if matchloading is faster
	chassis.moveToPoint(49, -35.5, 3000, {.forwards = false, .maxSpeed = 75});
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
	chassis.moveToPose(24, -21, -90, 2000, {.maxSpeed = 95, .minSpeed = 20, .earlyExitRange = 4});
	setIntake(127);
	wing.extend();
	chassis.moveToPose(-24, -21, -90, 2000, {.maxSpeed = 95, .minSpeed = 20, .earlyExitRange = 4});
	chassis.waitUntil(40);
	matchloader.extend();
	chassis.moveToPose(-9.2, -10, -135, 2000, {.forwards = false, .maxSpeed = 70});
	isStorageExtended = false;
    storage.retract();
	wing.retract();
	chassis.waitUntilDone();
	leverPID = true;
	nextState();
	pros::delay(400);
	setIntake(-127);
	nextState();
    isStorageExtended = true;
	storage.extend();
	wing.extend();
	chassis.moveToPoint(-38, -40, 2000, {.minSpeed = 20, .earlyExitRange = 8});
	setIntake(127);
	chassis.waitUntilDone();
	chassis.turnToHeading(-173, 500);
	chassis.waitUntilDone();
	chassis.setPose(-rightvalue, -53, -180);
	chassis.moveToPoint(-48.5, -65.5, 1500, {.maxSpeed = 60});
	chassis.waitUntilDone();
	pros::delay(200); // could save time for future if matchloading is faster
	chassis.moveToPoint(-49, -31, 2000, {.forwards = false, .maxSpeed = 75});
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
	chassis.moveToPoint(40.5, -46.75, 2000, {.forwards = false, .maxSpeed = 80});
	chassis.waitUntilDone();
	matchloader.extend();
	chassis.turnToHeading(-175, 500);
	chassis.waitUntilDone();
	chassis.setPose(leftvalue, -55, -180);
	chassis.moveToPoint(48.5, -66, 1500, {.maxSpeed = 60});
	chassis.waitUntilDone();
	pros::delay(200); // could save time for future if matchloading is faster
	chassis.moveToPoint(49, -35.5, 3000, {.forwards = false, .maxSpeed = 75});
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
    chassis.moveToPoint(24, -24, 2000, {.minSpeed = 20, .earlyExitRange = 4});
    // chassis.moveToPoint(15.2, -10.7, 2000, {.maxSpeed = 60}); //this is original if straight to middle
    wing.extend();
    setIntake(127);
    chassis.waitUntil(5);
    matchloader.extend();
    // chassis.waitUntilDone(); //if used original
    chassis.turnToHeading(60, 1000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(50);
    matchloader.retract();
    chassis.moveToPoint(46, -5, 2000, {.maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(18);
    matchloader.extend();
    chassis.moveToPoint(31, -13, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 5});
    chassis.turnToHeading(-35, 1000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(20);
    matchloader.retract();
    chassis.moveToPoint(22.5, -9.5, 2000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.setPose(15.2, -10.7, chassis.getPose().theta);
    setIntake(-127);
    pros::delay(1000);
    chassis.moveToPoint(38.5, -25, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.turnToHeading(-150, 1000);
    wing.retract();
    chassis.waitUntilDone();
    chassis.moveToPoint(42.5, -18, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 5});
    chassis.turnToHeading(-175, 1000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.moveToPoint(41, -8, 2000, {.forwards = false, .maxSpeed = 50});
    chassis.waitUntilDone();

}

void left() {
    chassis.setPose(1, -46.75, 90);
    setIntake(127);
    storage.extend();
    isStorageExtended = true;
    chassis.moveToPoint(-40.5, -46.75, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    matchloader.extend();
    chassis.turnToHeading(175, 500);
    chassis.waitUntilDone();
    chassis.setPose(-rightvalue, -55, 180);
    chassis.moveToPoint(-48.5, -66, 1500, {.maxSpeed = 60});
	chassis.waitUntilDone();
	pros::delay(200); // could save time for future if matchloading is faster
	chassis.moveToPoint(-49, -35.5, 3000, {.forwards = false, .maxSpeed = 75});
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
    chassis.moveToPoint(-26, -24, 2000, {.minSpeed = 20, .earlyExitRange = 4});
    wing.extend();
    setIntake(127);
    chassis.waitUntil(3);
    matchloader.extend();
    chassis.turnToHeading(-60, 1000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(50);
    matchloader.retract();
    chassis.moveToPoint(-49, -7, 2000, {.maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(18);
    matchloader.extend();
    chassis.moveToPoint(-27.5, -13, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 5});
    storage.retract();
    isStorageExtended = false;
    chassis.turnToHeading(-127, 1000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(20);
    matchloader.retract();
    chassis.moveToPoint(-16, -10, 700, {.forwards = false, .maxSpeed = 60});
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
    chassis.moveToPoint(-42.5, -18, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 20, .earlyExitRange = 5});
    chassis.turnToHeading(175, 1000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.moveToPoint(-40.5, -8, 2000, {.forwards = false, .maxSpeed = 50});
    chassis.waitUntilDone();
}

void skills() {
    chassis.setPose(-1, -46.75, -90);
    setIntake(127);
    storage.extend();
	isStorageExtended = true;
    chassis.moveToPoint(40.5, -46.75, 2000, {.forwards = false, .maxSpeed = 75});
    chassis.waitUntilDone();
    matchloader.extend();
    chassis.turnToHeading(-175, 500);
    chassis.waitUntilDone();
    chassis.setPose(leftvalue, -55, -180);
    chassis.moveToPoint(48.5, -65.5, 1500, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(1000); // could save time for future if matchloading is faster
    chassis.moveToPoint(48.5, -66, 1500, {.maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.moveToPoint(49, -55, 3000, {.forwards = false, .maxSpeed = 75});
    chassis.waitUntilDone();
    chassis.turnToHeading(-150, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(64, -45, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 8});
    chassis.moveToPose(66, 30, 180, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 8});
    chassis.turnToHeading(-90, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(56, 30, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(-10, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(52, 10, 1000, {.forwards = false, .maxSpeed = 70});
    wing.retract();
    chassis.waitUntilDone();
    chassis.setPose(49, 35.5, chassis.getPose().theta);
    leverPID = true;
    nextState();
    pros::delay(1500);
    setIntake(-127);
    nextState();
    chassis.moveToPoint(48.5, 66, 1500, {.maxSpeed = 60});
    setIntake(127);
    wing.extend();
    chassis.waitUntilDone();
    pros::delay(1000); // could save time for future if matchloading is faster
    chassis.moveToPoint(48.5, 66, 1500, {.maxSpeed = 50});
    chassis.waitUntilDone();
    chassis.moveToPoint(49, 35.5, 1000, {.forwards = false, .maxSpeed = 70});
    wing.retract();
    chassis.waitUntilDone();
    matchloader.retract();
    leverPID = true;
    nextState();
    pros::delay(1500);
    setIntake(-127);
    nextState();
    chassis.setPose(49, 29.2, chassis.getPose().theta);
    // clear blue park zone
    chassis.swingToHeading(-80, DriveSide::LEFT, 2000, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.moveToPoint(-45, 45, 3000, {.maxSpeed = 85});
    wing.extend();
    setIntake(127);
    chassis.waitUntilDone();
    chassis.turnToHeading(-5, 500);
    matchloader.extend();
    chassis.waitUntilDone();
    chassis.setPose(-leftvalue, 55, 0);
    chassis.moveToPoint(-48.5, 67.5, 1500, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(1000); // could save time for future if matchloading is faster
    chassis.moveToPoint(-48.5, 67.5, 1500, {.maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.moveToPoint(-49, 55, 3000, {.forwards = false, .maxSpeed = 75});
    chassis.waitUntilDone();
    chassis.turnToHeading(30, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(-62, 45, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 8});
    chassis.moveToPose(-62, -30, 0, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 8});
    chassis.turnToHeading(90, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(-50, -30, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(175, 1000);
    chassis.waitUntilDone();
	chassis.moveToPoint(-52, -10, 1000, {.forwards = false}); //this line freezes
	wing.retract();
	chassis.waitUntilDone();
	chassis.setPose(-49, -35.5, chassis.getPose().theta);
	leverPID = true;
	nextState();
	pros::delay(1300);
	setIntake(-127);
	nextState();
	chassis.moveToPoint(-48.5, -66, 1500, {.maxSpeed = 60});
	setIntake(127);
	wing.retract();
	chassis.waitUntilDone();
	pros::delay(1000); // could save time for future if matchloading is faster
	chassis.moveToPoint(-48.5, -66, 1500, {.maxSpeed = 60});
	chassis.waitUntilDone();
	chassis.moveToPoint(-49, -35.5, 1000, {.forwards = false, .maxSpeed = 70});
	matchloader.retract();
	wing.extend();
	chassis.waitUntilDone();
	leverPID = true;
	nextState();
	pros::delay(1300);
	setIntake(-127);
	nextState();
	chassis.setPose(-49, -29.2, chassis.getPose().theta);
	// clear red park zone and park
}

void autonomous() {
    left();
}

void opcontrol() {
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    pros::Controller master(pros::E_CONTROLLER_MASTER);

    // loop forever
    while (true) {
        // drive
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(leftY, rightY);


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
            
        // delay to save resources
        pros::delay(25);
    }
}