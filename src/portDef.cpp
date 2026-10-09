#include "portDef.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/exitcondition.hpp"
#include "main.h"
#include <array>
#include <vector>

using namespace pros;

const std::vector<std::int8_t> rightDTPorts{-12, -21};

const std::vector<std::int8_t> leftDTPorts{10, 13};

Motors motors[9]{

    {"DT-LFRONT", {13, 0, 0, 0}, 600}, {"DT-LBack", {10, 0, 0, 0}, 600}, {"DT-RFront", {-12, 0, 0, 0}, 600}, {"DT-RBack", {-21, 0, 0, 0}, 600}, {"Intake", {4, 0, 0, 0}, 600}, {"Extake", {-6, 0, 0, 0}, 200}, {"Rotate", {8, 0, 0, 0}, 200}, {"CascadeL", {9, 0, 0, 0}, 200}, {"CascadeR", {-1, 0, 0, 0}, 200}};

MotorGroup aleft(leftDTPorts, MotorGearset::blue, v5::MotorUnits::degrees);
MotorGroup aright(rightDTPorts, MotorGearset::blue, v5::MotorUnits::degrees);

Motor intake(4);
Motor rotate(8);
Motor shitter(6);

MotorGroup cascade({9, -1});

Rotation vertRotation(11);

lemlib::Drivetrain DT(&aleft, &aright, 11, lemlib::Omniwheel::NEW_275, 360,
                      4);

IMU inertial2(20);
IMU inertial1(16);

lemlib::TrackingWheel leftVert(&vertRotation, lemlib::Omniwheel::NEW_2, 0.0, 1);

lemlib::OdomSensors sensors(&leftVert, nullptr, nullptr, nullptr, &inertial1);

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
    angular_controller(3, // proportional gain (kP)
                       0,    // integral gain (kI)
                       3,   // derivative gain (kD)
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