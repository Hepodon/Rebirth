#pragma once
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"

struct Motors {
  std::string name;
  float port[4];
  int RPM;
  bool DT = false;
};

extern Motors motors[9];

extern pros::MotorGroup aleft;
extern pros::MotorGroup aright;

extern pros::Motor intake;
extern pros::Motor rotate;
extern pros::Motor shitter;

extern pros::MotorGroup cascade;


extern pros::Rotation vertRotation;

extern pros::v5::Optical colorSensor;

extern lemlib::Drivetrain DT;

extern pros::IMU inertial1;
extern pros::IMU inertial2;

extern lemlib::TrackingWheel leftVert;

extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateral_controller;

extern lemlib::ControllerSettings angular_controller;

extern lemlib::Chassis chassis;

extern pros::Controller userInput;

extern void avgIMU();