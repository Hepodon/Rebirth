#include "main.h"
#include "controls.hpp"
#include "graphics.hpp"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/misc/lv_event.h"
#include "liblvgl/misc/lv_types.h"
#include "lvgl_Customs.hpp"
#include "mcl.hpp"
#include "portDef.hpp"
#include "pros/motors.h"
#include "pros/screen.hpp"
#include <cmath>
#include <cstdio>

using namespace pros;
using namespace std;

// constexpr size_t PARTICLES = 3000;
// ad::MCL<PARTICLES> mcl;
// ad::Point last_odom_pos{0.0f, 0.0f};
// bool mcl_initialized = false;

// void odom() {

//   auto pose = chassis.getPose();

//   float odom_x = pose.x;
//   float odom_y = pose.y;

//   ad::Point current_odom{odom_x, odom_y};

//   float dx = current_odom.x - last_odom_pos.x;
//   float dy = current_odom.y - last_odom_pos.y;

//   last_odom_pos = current_odom;

//   float std_dev = std::hypot(dx, dy) / 4.0f;
//   mcl.predict(dx, dy, std_dev);

//   std::vector<ad::Reading> readings;

//   float theta = inertial1.get_rotation() * M_PI / 180.0f;
//   ad::Rotation heading = ad::rad(theta);

//   ad::Position robot_pos{current_odom.x, current_odom.y, heading};

//   auto add_sensor = [&](Distance &sensor, ad::Position offset) {
//     auto v = sensor.get();
//     if (!v)
//       return;

//     float d = v;

//     float bound = d < 7.874015f ? 0.590551f : 0.05f * d;
//     constexpr float K = 3.0f;
//     float std_dev = std::max(bound / K, 1e-6f);

//     ad::Position rel = offset.rotate(robot_pos.theta);
//     ad::Point ray_pt =
//         rel.point() + ad::Point{rel.theta.cos(), rel.theta.sin()};

//     readings.emplace_back(d, std_dev, rel.point(), ray_pt);
//   };

//   add_sensor(Dleft, ad::Position{-5.0f, 0.0f, ad::deg(180)});
//   add_sensor(Dright, ad::Position{5.0f, 0.0f, ad::deg(0)});
//   add_sensor(Dfront, ad::Position{0.0f, 5.0f, ad::deg(90)});
//   add_sensor(DbackL, ad::Position{0.0f, -5.0f, ad::deg(-90)});

//   mcl.update(readings);

//   ad::Point est = mcl.estimate();

//   chassis.setPose(est.x, est.y, 0);

//   mcl.resample();
// }

// Task odomTask([] {
//   while (true) {
//     odom();
//     delay(10);
//   }
// });

float mm_in(float mm) { return mm / 25.4; }

void printer() {
  while (true) {
    std::cout << "\rClaw_Height: " << mm_in(dClaw.get())
              << "In, Detected_Dist: " << mm_in(dFront.get())
              << "In, RGB: " << static_cast<int>(colorSensor.get_rgb().red)
              << " R, " << static_cast<int>(colorSensor.get_rgb().blue)
              << " B, " << static_cast<int>(colorSensor.get_rgb().green)
              << " G";
    delay(150);
  }
}

struct RGB {
  double red;
  double green;
  double blue;
};

struct block {
  bool exists;
  double heading;
  int height;
  struct RGB helo;
};

struct stack {
  double heading;
  int stack_height;
  struct block blocks[6];
};

std::vector<stack> stacks;

bool moving = false;

float c_kP = 10;
float c_kD = 0;
int timeout = 500;

void raise_cascade(int t_height_in) {
  moving = true;
  double start = mm_in(dClaw.get());
  double error = t_height_in - start;
  double prev_error = error;
  bool targetZone = false;
  int time = 0;

  while (moving) {
    error = t_height_in - start;
    double p_Term = error * c_kP;
    double d_Term = c_kD * (error - prev_error);
    prev_error = error;

    double power = p_Term + d_Term;

    if (mm_in(dClaw.get()) <= t_height_in + 1 ||
        mm_in(dClaw.get()) >= t_height_in - 1) {
      targetZone = true;
    }

    if (targetZone == true) {
      if (!(mm_in(dClaw.get()) <= t_height_in + 1) ||
          !(mm_in(dClaw.get()) >= t_height_in - 1)) {
        targetZone = false;
        time = 0;
      } else {
        time += 20;
      }
      if (time == timeout) {
        moving = false;
        break;
      }
    }
    cascade.move(power);
    delay(20);
  }
  cascade.brake();
}

void initialize() {
  Task printingTask(printer);
  colorSensor.set_led_pwm(75);
  chassis.calibrate(false);
  lvgl_init();
  screen_init();
  claw.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  cascade.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
}

void disabled() {}

void competition_initialize() { chassis.calibrate(true); }

stack current_Stack;
block current_Block[6];
RGB current_Block_Color[6];

void autonomous() {
  chassis.setPose(0, 0, 0);
  float saved_dist;
  int count = 0;
  double heading;
  bool scanning = true;
  while (scanning) {
    while (mm_in(dFront.get()) < 36) {
      chassis.arcade(0, 30);
      if (inertial1.get_heading() > 350) {
        scanning = false;
        break;
      }
    }
    chassis.arcade(0, 0);
    saved_dist = mm_in(dFront.get());
    heading = inertial1.get_heading();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, saved_dist - 4, 10000);
    chassis.waitUntilDone();

    for (int ip = 1; ip <= 6; ip++) {
      raise_cascade(ip * 6.3);
      while (moving == true) {
        delay(20);
      }
      if (!(mm_in(dFront.get()) > 10)) {
        current_Block_Color[ip] = {colorSensor.get_rgb().red,
                                   colorSensor.get_rgb().green,
                                   colorSensor.get_rgb().blue};

        current_Block[ip] = {true, heading, ip, current_Block_Color[ip]};
        count++;
      } else {
        for (int i = ip; i <= 6; i++) {
          current_Block_Color[i] = {0, 0, 0};
          current_Block[i] = {false, 0, 0, current_Block_Color[i]};
        }
        ip = 7;
      }
    }
    raise_cascade(4);
    while (moving == true) {
      delay(20);
    }

    current_Stack = {heading,
                     count,
                     {current_Block[0], current_Block[1], current_Block[2],
                      current_Block[3], current_Block[4], current_Block[5]}};

    stacks.push_back(current_Stack);
    chassis.moveToPoint(0, 0, 10000, {.forwards = false});
    chassis.waitUntilDone();
    chassis.setPose(0, 0, inertial1.get_heading());
    if (inertial1.get_heading() > 350) {
      scanning = false;
      break;
    }
  }
}
void opcontrol() {
  int leftY;
  int leftX;
  int rightY;
  int rightX;

  chassis.setBrakeMode(E_MOTOR_BRAKE_COAST);

  while (true) {
    leftY = userInput.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    leftX = userInput.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
    rightY = userInput.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
    rightX = userInput.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

    applyButtons(userInput);

    chassis.arcade(leftY, leftX, false, 0.40);

    if (abs(rightY) > 10) {
      cascade.move(rightY);
    } else {
      cascade.brake();
    }
    if (abs(rightX) > 10) {
      claw.move(rightX);
    } else {
      claw.brake();
    }
    delay(10);
  }
}