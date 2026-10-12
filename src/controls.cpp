#include "controls.hpp"
#include "main.h"
#include "portDef.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

void erect() {
  rotate.move(75);
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    pros::delay(15);
  }
  rotate.brake();
}

void deErect() {
  if (!(rotate.get_position() < 0.0)) {
    rotate.move(-75);
  }
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    if (!(rotate.get_position() < 0.0)) {
      rotate.brake();
    }
    pros::delay(15);
  }
  rotate.brake();
}

void suck() {
  intake.move(127);
  shitter.move(-127);
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    pros::delay(15);
  }
  intake.brake();
  shitter.brake();
}

void spit() {
  intake.move(-127);
  shitter.move(127);
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    pros::delay(15);
  }
  intake.brake();
  shitter.brake();
}

void CasUp() {
  cascade.move(127);
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
    pros::delay(15);
  }
  cascade.brake();
}

void CasDown() {
  if (cascade.get_position() >= -1) {
    cascade.move(-127);
  }
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    if (cascade.get_position() < -0.5) {
      cascade.brake();
    }
    pros::delay(15);
  }
  cascade.brake();
}

void score() {
  shitter.move(127);
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    pros::delay(15);
  }
  shitter.brake();
}

void resetCascadeRotate() {
  cascade.move_absolute(0, 200);
  rotate.move_absolute(0, 200);
}

void buttonFunctionPinControls() {
  while (true) {
    if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
      suck();
    } else if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
      score();
    } else if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      spit();
    }
    pros::delay(15);
  }
}

void buttonFunctionCasControls() {
  cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  cascade.brake();
  while (true) {
    if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      CasUp();
    } else if (userInput.get_digital_new_press(
                   pros::E_CONTROLLER_DIGITAL_DOWN)) {
      CasDown();
    }
    pros::delay(15);
  }
}

void buttonFunctionShitControls() {
  rotate.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rotate.brake();
  while (true) {
    if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      erect();
    } else if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      deErect();
    }
    pros::delay(15);
  }
}