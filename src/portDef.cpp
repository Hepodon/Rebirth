#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/exitcondition.hpp"
#include "main.h"
#include <vector>

using namespace pros;

const std::vector<std::int8_t> rightDTPorts{1, 1, 1};

const std::vector<std::int8_t> leftDTPorts{1, 1, 1};

float leftDT[sizeof(leftDTPorts) / sizeof(leftDTPorts[0])][4] = {
    {1, 0, 0, 0}, {1, 0, 0, 0}, {1, 0, 0, 0}};

float rightDT[sizeof(rightDTPorts) / sizeof(rightDTPorts[0])][4] = {
    {1, 0, 0, 0}, {1, 0, 0, 0}, {1, 0, 0, 0}};

struct otherMotors {
  std::string name;
  float port[4];
};

otherMotors motors[2]{{"left-Middle", {0, 0, 0, 0}},
                      {"right-Middle", {0, 0, 0, 0}}};

MotorGroup aleft(leftDTPorts, MotorGearset::blue, v5::MotorUnits::degrees);
MotorGroup aright(rightDTPorts, MotorGearset::blue, v5::MotorUnits::degrees);

Motor leftMiddle(motors[0].port[0]);
Motor rightMiddle(motors[1].port[0]);

adi::Pneumatics match('a', false);
adi::Pneumatics arm('h', false);
pros::adi::Pneumatics tripstate('b', false);
pros::adi::Pneumatics tripstate2('e', false);

Distance Dleft(0);
Distance Dright(0);
Distance Dfront(0);
Distance DbackR(0);
Distance DbackL(0);

Rotation vertRotation(-16);

v5::Optical colorSensorMatch(0);
v5::Optical colorSensorScore(0);

lemlib::Drivetrain DT(&aleft, &aright, 12.72, lemlib::Omniwheel::NEW_325, 450,
                      8);

IMU inertial1(1);

lemlib::TrackingWheel leftVert(&vertRotation, lemlib::Omniwheel::NEW_2, 0.0, 1);

lemlib::OdomSensors sensors(&leftVert, nullptr, nullptr, nullptr, &inertial1);
// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(5,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       5.5, // derivative gain (kD)
                       3,   // anti windup
                       0.5, // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       1.5, // large error range, in inches
                       200, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(2.45, // proportional gain (kP)
                       0,    // integral gain (kI)
                       10,   // derivative gain (kD)
                       3,    // anti windup
                       1,    // small error range, in degrees
                       100,  // small error range timeout, in milliseconds
                       3,    // large error range, in degrees
                       500,  // large error range timeout, in milliseconds
                       0     // maximum acceleration (slew)
    );

lemlib::ExpoDriveCurve driveCurve(3, 15, 1.015);

lemlib::Chassis chassis(DT, lateral_controller, angular_controller, sensors,
                        &driveCurve, &driveCurve);
Controller userInput(E_CONTROLLER_MASTER);