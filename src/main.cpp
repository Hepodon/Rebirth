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

constexpr size_t PARTICLES = 3000;
ad::MCL<PARTICLES> mcl;
ad::Point last_odom_pos{0.0f, 0.0f};
bool mcl_initialized = false;

void odom() {

  auto pose = chassis.getPose();

  float odom_x = pose.x;
  float odom_y = pose.y;

  ad::Point current_odom{odom_x, odom_y};

  float dx = current_odom.x - last_odom_pos.x;
  float dy = current_odom.y - last_odom_pos.y;

  last_odom_pos = current_odom;

  float std_dev = std::hypot(dx, dy) / 4.0f;
  mcl.predict(dx, dy, std_dev);

  std::vector<ad::Reading> readings;

  float theta = inertial1.get_rotation() * M_PI / 180.0f;
  ad::Rotation heading = ad::rad(theta);

  ad::Position robot_pos{current_odom.x, current_odom.y, heading};

  auto add_sensor = [&](Distance &sensor, ad::Position offset) {
    auto v = sensor.get();
    if (!v)
      return;

    float d = v;

    float bound = d < 7.874015f ? 0.590551f : 0.05f * d;
    constexpr float K = 3.0f;
    float std_dev = std::max(bound / K, 1e-6f);

    ad::Position rel = offset.rotate(robot_pos.theta);
    ad::Point ray_pt =
        rel.point() + ad::Point{rel.theta.cos(), rel.theta.sin()};

    readings.emplace_back(d, std_dev, rel.point(), ray_pt);
  };

  // add_sensor(Dleft, ad::Position{-5.0f, 0.0f, ad::deg(180)});
  // add_sensor(Dright, ad::Position{5.0f, 0.0f, ad::deg(0)});
  // add_sensor(Dfront, ad::Position{0.0f, 5.0f, ad::deg(90)});
  // add_sensor(DbackL, ad::Position{0.0f, -5.0f, ad::deg(-90)});

  mcl.update(readings);

  ad::Point est = mcl.estimate();

  chassis.setPose(est.x, est.y, 0);

  mcl.resample();
}

Task odomTask([] {
  while (true) {
    odom();
    delay(10);
  }
});

float mm_in(float mm) { return mm / 25.4; }

void printer() {
  // while (true) {
  //   std::cout << "\rClaw_Height: " << mm_in(dClaw.get())
  //             << "In, Detected_Dist: " << mm_in(dFront.get())
  //             << "In, RGB: " << static_cast<int>(colorSensor.get_rgb().red)
  //             << " R, " << static_cast<int>(colorSensor.get_rgb().blue)
  //             << " B, " << static_cast<int>(colorSensor.get_rgb().green)
  //             << " G";
  //   delay(150);
  // }
}

void initialize() {
  // Task printingTask(printer);
  // colorSensor.set_led_pwm(75);
  chassis.calibrate(false);
  lvgl_init();
  screen_init();
}

void disabled() {}

void competition_initialize() { chassis.calibrate(true); }

void autonomous() { chassis.setPose(0, 0, 0); }
void opcontrol() {

  chassis.setBrakeMode(E_MOTOR_BRAKE_COAST);

  while (true) {
    applyButtons(userInput);

    chassis.arcade(userInput.get_analog(ANALOG_LEFT_Y),
                   userInput.get_analog(ANALOG_RIGHT_X), false, 0.40);

    delay(10);
  }
}