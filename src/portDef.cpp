#include "portDef.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/exitcondition.hpp"
#include "main.h"
#include <array>
#include <vector>

using namespace pros;

const std::vector<std::int8_t> rightDTPorts{8, 9, -11};

const std::vector<std::int8_t> leftDTPorts{-1, -2, 3};

Motors motors[6]{
    {"DT-leftMini", {3, 0, 0, 0}, 200},  {"DT-rightMini", {11, 0, 0, 0}, 200},
    {"DT-leftFront", {1, 0, 0, 0}, 600}, {"DT-rightFront", {8, 0, 0, 0}, 600},
    {"DT-leftBack", {2, 0, 0, 0}, 600},  {"DT-rightBack", {9, 0, 0, 0}, 600}};

MotorGroup aleft(leftDTPorts, MotorGearset::blue, v5::MotorUnits::degrees);
MotorGroup aright(rightDTPorts, MotorGearset::blue, v5::MotorUnits::degrees);

Rotation vertRotation(0);

Rotation horRotation(0);

v5::Optical colorSensor(0);

lemlib::Drivetrain DT(&aleft, &aright, 12.72, lemlib::Omniwheel::NEW_325, 450,
                      8);

IMU inertial1(0);
IMU inertial2(0);

lemlib::TrackingWheel leftVert(&vertRotation, lemlib::Omniwheel::NEW_2, 0.0, 1);

lemlib::TrackingWheel leftVHor(&horRotation, lemlib::Omniwheel::NEW_2, 0.0, 1);

lemlib::OdomSensors sensors(&leftVert, nullptr, &leftVHor, nullptr, &inertial1);

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