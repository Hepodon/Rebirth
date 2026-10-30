#include "main.h"
#include "controls.hpp"
#include "graphics.hpp"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/misc/lv_event.h"
#include "liblvgl/misc/lv_types.h"
#include "lvgl_Customs.hpp"
#include "mcl.hpp"
#include "portDef.hpp"
#include "pros/misc.h"
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

// float mm_in(float mm) { return mm / 25.4; }

int dist_spacing = 245;

void initialize() {
  chassis.calibrate(false);

  // lvgl_init();
  // screen_init();
  pros::lcd::initialize(); // initialize brain screen
  // print position to brain screen
  pros::Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // delay to save resources
      pros::delay(20);
    }
  });
}

float get_theta_rel() {

  float theta =
      std::atan2(left_D.get() - right_D.get(), dist_spacing) * (180 / M_PI);

  return theta;
}
float get_dist_rel_in(float theta) {

  float distance =
      ((left_D.get() + right_D.get()) / 2.0) * cos(theta / (180 * M_PI));

  distance /= 25.4;

  return distance;
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

enum awristState { DOWN = 0, UP = 1340, TILT = 560, SCORE = 960 };
awristState awrist_State = DOWN;

void autonomous() {
  // while (!aready) {
  //   delay(20);
  // }
  // Red Right
  // autononm(selectedAutonomo);
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, -5, 650, {.forwards = false, .minSpeed = 85});
  chassis.waitUntil(4);
  chassis.cancelAllMotions();
  chassis.arcade(105, 0);
  pros::delay(650);
  chassis.arcade(-90, 0);
  pros::delay(450);
  chassis.arcade(105, 0);
  pros::delay(650);
  chassis.cancelAllMotions();

  float theta = -get_theta_rel();

  chassis.setPose(chassis.getPose().x, get_dist_rel_in(theta), theta);

  rotate.move_absolute(760, 200);
  chassis.moveToPose(37.5, -23, -30, 1450,
                     {.forwards = false, .horizontalDrift = 4, .lead = 0.5});
  chassis.waitUntilDone();
  delay(150);
  shitter.move_velocity(100);
}

void opcontrol() {
  Task ok1(buttonFunctionPinControls);
  Task ok2(buttonFunctionCasControls);
  Task ok3(buttonFunctionShitControls);

  chassis.setBrakeMode(E_MOTOR_BRAKE_COAST);

  rotate.brake();

  while (true) {
    if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      resetCascadeRotate();
    }

    chassis.arcade(userInput.get_analog(ANALOG_LEFT_Y),
                   userInput.get_analog(ANALOG_RIGHT_X), false, 0.40);

    delay(10);
  }
}