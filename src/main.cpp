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

// constexpr size_t PARTICLES = 3000;
// ad::MCL<PARTICLES> mcl;
// ad::Point last_odom_pos{0.0f, 0.0f};
// bool mcl_initialized = false;

// void odom()
// {

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

//   auto add_sensor = [&](Distance &sensor, ad::Position offset)
//   {
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

//   // add_sensor(Dleft, ad::Position{-5.0f, 0.0f, ad::deg(180)});
//   // add_sensor(Dright, ad::Position{5.0f, 0.0f, ad::deg(0)});
//   // add_sensor(Dfront, ad::Position{0.0f, 5.0f, ad::deg(90)});
//   // add_sensor(DbackL, ad::Position{0.0f, -5.0f, ad::deg(-90)});

//   mcl.update(readings);

//   ad::Point est = mcl.estimate();

//   chassis.setPose(est.x, est.y, 0);

//   mcl.resample();
// }

// Task odomTask([]
//               {
//   while (true) {
//     odom();
//     delay(10);
//   } });

float mm_in(float mm) { return mm / 25.4; }

void initialize() {
  chassis.calibrate(false);

  lvgl_init();
  screen_init();
}

void disabled() {}

void competition_initialize() { chassis.calibrate(true); }

void autononm(AutonSel AUTON) {
  switch (AUTON) {
  case (Blue_L):
    break;
  case (Blue_R):
    break;
  case (Red_L):
    break;
  case (Red_R):
    break;
  case (Skills):
    break;
  }
}

void autonomous() {
  // while (!aready) {
  //   delay(20);
  // }
  // Red Right
  // autononm(selectedAutonomo);
  // chassis.moveToPoint(0, -10, 650, {.forwards = false, .minSpeed = 85});
  // chassis.waitUntilDone();

  chassis.arcade(-127, 0);
  delay(400);
  chassis.arcade(127, 0);
  delay(800);
  
  chassis.arcade(-127, -80);
  delay(400);
  chassis.arcade(90, 0);
  delay(800);
  chassis.arcade(-60, 0);
  delay(400);
  chassis.arcade(0,0);
  
  chassis.cancelAllMotions();
  
  // chassis.moveToPoint(0, -10, 650, {.forwards = false, .minSpeed = 85});
  // chassis.waitUntilDone();

  

  // shitter.move(-50);
  // rotate.move_absolute(720, 200);
  // chassis.moveToPose(24, -16, -90, 3000,
  //                    {.forwards = false, .horizontalDrift = 4});
  // chassis.waitUntilDone();
  // shitter.move(50);
  // cascade.move_absolute(50, 100);
}

void buttonsTask() {
  while (true) {
    applyButtons(userInput);
    delay(15);
  }
}

void opcontrol() {
  Task ok(buttonsTask);

  chassis.setBrakeMode(E_MOTOR_BRAKE_COAST);

  while (true) {

    chassis.arcade(userInput.get_analog(ANALOG_LEFT_Y),
                   userInput.get_analog(ANALOG_RIGHT_X), false, 0.40);

    delay(10);
  }
}