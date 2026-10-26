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

void score() {
  shitter.move(127);
  while (userInput.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    pros::delay(15);
  }
  shitter.brake();
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
      cascade.move(127);
    } else if (userInput.get_digital_new_press(
                   pros::E_CONTROLLER_DIGITAL_DOWN)) {
      cascade.move(-127);
    } else if (userInput.get_digital_new_release(
                   pros::E_CONTROLLER_DIGITAL_UP) &&
               !userInput.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      cascade.brake();
    } else if (userInput.get_digital_new_release(
                   pros::E_CONTROLLER_DIGITAL_DOWN) &&
               !userInput.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      cascade.brake();
    }
    pros::delay(15);
  }
}

enum wristState { DOWN = 0, UP = 1340, TILT = 560, SCORE = 960 };
wristState wrist_State = DOWN;

void buttonFunctionShitControls() {
  float kp = 0.25;
  float ki = 0;
  float kd = 0.01;

  float ki_val = ki;

  int timeout = 0;

  float error;
  float power;
  float current;
  float last_error = 0;
  rotate.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rotate.brake();
  bool settled = true;
  while (true) {
    if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      wrist_State = SCORE;
      settled = false;
    } else if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      settled = false;
      wrist_State = UP;
    } else if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      settled = false;
      wrist_State = TILT;
    }
    while (!settled) {
      if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        wrist_State = SCORE;
      } else if (userInput.get_digital_new_press(
                     pros::E_CONTROLLER_DIGITAL_B)) {
        wrist_State = UP;
      } else if (userInput.get_digital_new_press(
                     pros::E_CONTROLLER_DIGITAL_Y)) {
        wrist_State = TILT;
      }

      current = rotate.get_position();

      error = static_cast<float>(wrist_State) - current;

      power = (error * kp);

      power += (error - last_error) * kd;

      ki_val += error * ki;

      power += ki_val * ki;

      last_error = error;

      if (std::abs(error) < 5.0) {
        timeout += 15;
        if (timeout > 500) {
          ki_val = ki;
          rotate.brake();
          settled = true;
          break;
        }
      } else {
        timeout = 0;
      }

      power = std::clamp(power, -127.0f, 127.0f);

      rotate.move(static_cast<int>(power));

      pros::delay(15);
    }
    pros::delay(15);
  }
}

void resetCascadeRotate() {
  cascade.move_absolute(0, 200);
  wrist_State = DOWN;
  rotate.move_absolute(0, 200);
}