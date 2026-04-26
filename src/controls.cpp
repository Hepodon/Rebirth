#include "controls.hpp"
#include "portDef.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"

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

void buttonFunction(pros::controller_digital_e_t button) {
  switch (button) {
  case pros::E_CONTROLLER_DIGITAL_R1:
    break;
  case pros::E_CONTROLLER_DIGITAL_R2:
    break;
  case pros::E_CONTROLLER_DIGITAL_L2:
    break;
  case pros::E_CONTROLLER_DIGITAL_L1:
    break;
  case pros::E_CONTROLLER_DIGITAL_UP:
    break;
  case pros::E_CONTROLLER_DIGITAL_DOWN:
    break;
  case pros::E_CONTROLLER_DIGITAL_LEFT:
    break;
  case pros::E_CONTROLLER_DIGITAL_RIGHT:
    break;
  case pros::E_CONTROLLER_DIGITAL_A:
    break;
  case pros::E_CONTROLLER_DIGITAL_B:
    break;
  case pros::E_CONTROLLER_DIGITAL_X:
    break;
  case pros::E_CONTROLLER_DIGITAL_Y:
    break;
  case pros::E_CONTROLLER_DIGITAL_POWER:
    userInput.rumble("---___---");
    break;
  }
}