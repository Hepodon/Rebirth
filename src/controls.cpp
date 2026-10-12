#include "controls.hpp"
#include "main.h"
#include "portDef.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

pros::controller_digital_e_t buttons[13] = {
    pros::E_CONTROLLER_DIGITAL_R1,   pros::E_CONTROLLER_DIGITAL_R2,
    pros::E_CONTROLLER_DIGITAL_L1,   pros::E_CONTROLLER_DIGITAL_L2,
    pros::E_CONTROLLER_DIGITAL_UP,   pros::E_CONTROLLER_DIGITAL_DOWN,
    pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT,
    pros::E_CONTROLLER_DIGITAL_A,    pros::E_CONTROLLER_DIGITAL_B,
    pros::E_CONTROLLER_DIGITAL_X,    pros::E_CONTROLLER_DIGITAL_Y,
    pros::E_CONTROLLER_DIGITAL_POWER};

void applyButtons(pros::Controller userInput) {
  for (pros::controller_digital_e_t button : buttons) {
    if (userInput.get_digital_new_press(button)) {
      buttonFunction(button);
    }
  }
}

bool restPos = true;

void erect() {
  rotate.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    rotate.move(90);
  }
  rotate.brake();
}

void deErect() {
  rotate.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    if (!(rotate.get_position() < 0.0)) {
      rotate.move(-90);
    } else {
      rotate.brake();
    }
  }
  rotate.brake();
}

void suck() {
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    intake.move(127);
    shitter.move(-127);
    pros::delay(20);
  }
  intake.brake();
  shitter.brake();
}

void spit() {
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    intake.move(-127);
    shitter.move(127);
    pros::delay(20);
  }
  intake.brake();
  shitter.brake();
}

void CasUp() {
  cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
    cascade.move(127);
  }
  cascade.brake();
}

void CasDown() {
  cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    if (!(cascade.get_position() < 0.0)) {
      cascade.move(-127);
    } else {
      cascade  .brake();
    }
  }
  cascade.brake();
}

void score() {
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    shitter.move(127);
  }
  shitter.brake();
}

void buttonFunction(pros::controller_digital_e_t button) {
  switch (button) {
  case pros::E_CONTROLLER_DIGITAL_R1:
    erect();
    break;
  case pros::E_CONTROLLER_DIGITAL_R2:
    score();
    break;
  case pros::E_CONTROLLER_DIGITAL_L2:
    suck();
    break;
  case pros::E_CONTROLLER_DIGITAL_L1:
    spit();
    break;
  case pros::E_CONTROLLER_DIGITAL_UP:
    CasUp();
    break;
  case pros::E_CONTROLLER_DIGITAL_DOWN:
    CasDown();
    break;
  case pros::E_CONTROLLER_DIGITAL_LEFT:
    break;
  case pros::E_CONTROLLER_DIGITAL_RIGHT:
    break;
  case pros::E_CONTROLLER_DIGITAL_A:
    break;
  case pros::E_CONTROLLER_DIGITAL_B:
    deErect();
    break;
  case pros::E_CONTROLLER_DIGITAL_X:
    break;
  case pros::E_CONTROLLER_DIGITAL_Y:
    break;
  case pros::E_CONTROLLER_DIGITAL_POWER:
    break;
  }
}