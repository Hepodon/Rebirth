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
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>

using namespace pros;
// =====================================================================
//  CONFIG: the only part you edit when you move or add sensors
// =====================================================================
constexpr float MM_TO_IN = 1.0f / 25.4f;
constexpr float mm(float v) {
  return v * MM_TO_IN;
} // write measurements in mm if you like

constexpr float START_X = 0.0f, START_Y = 0.0f, START_THETA = 0.0f;

// Field coordinates of your odometry's (0,0). MEASURE these.
// Assumes odom axes are parallel to the field axes and heading 0 = field +Y.
constexpr float ORIGIN_X = 0.0f;
constexpr float ORIGIN_Y = FIELD_MIN + 7.0f;

constexpr size_t PARTICLES = 1000;

// Facing, in degrees CCW from the robot's right side
constexpr float FACE_RIGHT = 0, FACE_FRONT = 90, FACE_LEFT = 180,
                FACE_BACK = -90;

struct DistSensor {
  const char *name;
  pros::Distance *dev;
  float x_in, y_in; // sensor FACE position from tracking center (x right, y
                    // forward), inches
  float facing_deg; // direction it points
  bool use_in_mcl;  // false = ignore in MCL (e.g. sees a mechanism), still
                    // usable by wall_reset
};

DistSensor dist_sensors[] = {
    //  name     device    x_in           y_in   facing      mcl
    {"backL", &left_D, -mm(122.5f), -4.0f, FACE_BACK, true},
    {"backR", &right_D, mm(122.5f), -4.0f, FACE_BACK, true},
    // {"front", &front_D,  0.0f,       6.0f,  FACE_FRONT, true},
    // {"side",  &side_D,   5.5f,       0.0f,  FACE_RIGHT, true},
};

// Same order as the table above. Use these names when calling wall_reset().
enum SensorId { BACK_L, BACK_R, /* FRONT, SIDE, */ SENSOR_COUNT };
constexpr int N_SENSORS = sizeof(dist_sensors) / sizeof(dist_sensors[0]);
static_assert(N_SENSORS == SENSOR_COUNT,
              "SensorId enum and dist_sensors[] are out of sync");
// =====================================================================

ad::MCL<PARTICLES> mcl;
ad::Point last_odom{0.0f, 0.0f};
std::atomic<bool> apply_mcl_request{false};
std::atomic<bool> reset_pending{false};
std::atomic<bool> mcl_apply_enabled{
    false}; // correction kill switch (starts OFF)
std::atomic<bool> snapshot_request{false}; // print one labeled line on demand
std::atomic<float> nudge_y_request{0.0f}; // shift odom pose by this many inches
pros::Task *mcl_task = nullptr;

static bool valid_mm(int32_t v) { return v > 0 && v < 2000 && v != PROS_ERR; }

static void build_readings(std::vector<ad::Reading> &out,
                           float lemlib_heading_deg) {
  out.clear();
  // LemLib heading is clockwise from +Y; ad::Rotation is CCW math convention
  ad::Rotation rot = ad::Rotation::deg(-lemlib_heading_deg);

  for (const auto &s : dist_sensors) {
    if (!s.use_in_mcl)
      continue;
    int32_t raw = s.dev->get();
    if (!valid_mm(raw))
      continue;

    float d = raw * MM_TO_IN;
    float bound = d < 7.874015f ? 0.590551f : 0.05f * d;
    float sd = std::max(bound / 2.0f, 0.25f);

    ad::Position rel =
        ad::Position{s.x_in, s.y_in, ad::deg(s.facing_deg)}.rotate(rot);
    ad::Point dir{rel.theta.cos(), rel.theta.sin()};
    out.emplace_back(d, sd, rel.point(), rel.point() + dir);
  }
}

static void mcl_loop() {
  std::vector<ad::Reading> readings;
  readings.reserve(N_SENSORS);

  while (true) {
    auto pose = chassis.getPose();

    if (reset_pending.exchange(false)) {
      last_odom = {pose.x, pose.y};
      mcl.init(pose.x + ORIGIN_X, pose.y + ORIGIN_Y, 1.0f);
    }
    float nudge = nudge_y_request.exchange(0.0f);
    if (nudge != 0.0f) {
      chassis.setPose(pose.x, pose.y + nudge, pose.theta);
      last_odom = {pose.x,
                   pose.y + nudge}; // don't feed the jump into predict()
      pose = chassis.getPose();
    }

    ad::Point cur{pose.x, pose.y};
    ad::Point d = cur - last_odom;
    last_odom = cur;

    mcl.predict(d.x, d.y, 0.03f * d.hypot());
    build_readings(readings, pose.theta);
    mcl.update(readings);

    ad::Point est = mcl.estimate();
    est = {est.x - ORIGIN_X, est.y - ORIGIN_Y};

    bool stationary = d.hypot() < 0.05f;
    if (stationary) {
      static int n = 0;
      bool snap = snapshot_request.exchange(false);
      if (snap || ++n % 4 == 0) {
        float sx, sy;
        mcl.spread_xy(sx, sy);
        printf("%s odom %.2f,%.2f th %.1f | mcl %.2f,%.2f | sx %.2f sy %.2f "
               "neff %.0f | apply %d |",
               snap ? "SNAP" : "log", pose.x, pose.y, pose.theta, est.x, est.y,
               sx, sy, mcl.n_eff(), (int)mcl_apply_enabled.load());
        for (int i = 0; i < N_SENSORS; ++i)
          printf(" %s=%.2f", dist_sensors[i].name,
                 dist_sensors[i].dev->get() * MM_TO_IN);
        printf("\n");
      }
    }
    static int n = 0;
    bool snap = snapshot_request.exchange(false);
    if (snap || ++n % 4 == 0) {
      float sx, sy;
      mcl.spread_xy(sx, sy);
      pros::lcd::print(0, "odom %.1f %.1f th %.0f", pose.x, pose.y, pose.theta);
      pros::lcd::print(1, "mcl  %.1f %.1f", est.x, est.y);
      pros::lcd::print(2, "sx %.2f sy %.2f", sx, sy);
      pros::lcd::print(3, "neff %.0f apply %d", mcl.n_eff(),
                       (int)mcl_apply_enabled.load());
      for (int i = 0; i < N_SENSORS && i < 3; ++i)
        pros::lcd::print(4 + i, "%s %.2f in", dist_sensors[i].name,
                         dist_sensors[i].dev->get() * MM_TO_IN);
      // line 7 is kept free for wall_reset messages
    }
    mcl.resample_if_needed();
    pros::delay(50); // sensors only update ~20 Hz; faster just re-counts the
                     // same reading
  }
}

void initialize() {
  chassis.calibrate(true);
  chassis.setPose(START_X, START_Y, START_THETA);
  mcl.init(START_X + ORIGIN_X, START_Y + ORIGIN_Y, 2.0f);
  last_odom = {START_X, START_Y};

  mcl_task = new pros::Task([] { mcl_loop(); });

  // lvgl_init();
  // screen_init();
  pros::lcd::initialize();
}

static int median_mm(pros::Distance &d, int n = 7) {
  int v[16], k = 0;
  for (int i = 0; i < n && i < 16; ++i) {
    int32_t x = d.get();
    if (valid_mm(x))
      v[k++] = x;
    pros::delay(30);
  }
  if (k < n / 2 + 1)
    return -1;
  std::sort(v, v + k);
  return v[k / 2];
}

// Reset position from a wall using any sensors in the table, e.g.
//   wall_reset({BACK_L, BACK_R});   // back wall, two sensors averaged
//   wall_reset({SIDE});             // side wall, one sensor
// All listed sensors must face the same wall. The robot must be stationary and
// roughly square to that wall (within 10 deg by IMU heading). It sets only the
// axis the wall constrains (y for a front/back wall, x for a side wall).
bool wall_reset(std::initializer_list<int> ids) {
  constexpr float MAX_TILT_DEG = 10.0f;   // IMU heading vs wall normal
  constexpr float MAX_DISAGREE_IN = 1.5f; // sensors must agree with each other

  auto p = chassis.getPose();
  ad::Rotation rot = ad::Rotation::deg(-p.theta);

  int axis = -1;    // 0 = x wall, 1 = y wall
  float sgn = 0.0f; // +1 = max wall, -1 = min wall
  float est[8];
  int n = 0;

  for (int id : ids) {
    if (id < 0 || id >= N_SENSORS || n >= 8)
      return false;
    const DistSensor &s = dist_sensors[id];

    // Ray direction in the field frame
    ad::Rotation face = ad::deg(s.facing_deg);
    ad::Point u = ad::Point{face.cos(), face.sin()}.rotate(rot);

    int a = std::fabs(u.x) > std::fabs(u.y) ? 0 : 1;
    float along = a == 0 ? u.x : u.y;
    float across = a == 0 ? u.y : u.x;
    float sg = along > 0 ? 1.0f : -1.0f;

    float tilt_deg =
        std::atan2(std::fabs(across), std::fabs(along)) * 180.0f / M_PI;
    if (tilt_deg > MAX_TILT_DEG) {
      printf("wall_reset: %s is %.1f deg off the wall normal\n", s.name,
             tilt_deg);
      return false;
    }
    if (n == 0) {
      axis = a;
      sgn = sg;
    } else if (a != axis || sg != sgn) {
      printf("wall_reset: sensors don't face the same wall\n");
      return false;
    }

    int raw = median_mm(*s.dev);
    if (raw < 0) {
      printf("wall_reset: %s has no valid reading\n", s.name);
      return false;
    }

    float wall = sg > 0 ? FIELD_MAX : FIELD_MIN;
    float sensor_pos = wall - sg * (raw * MM_TO_IN) * std::fabs(along);

    // Convert sensor position into robot-center position using the current
    // heading
    ad::Point off = ad::Point{s.x_in, s.y_in}.rotate(rot);
    est[n++] = sensor_pos - (a == 0 ? off.x : off.y);
  }
  if (n == 0)
    return false;

  float lo = est[0], hi = est[0], sum = 0.0f;
  for (int i = 0; i < n; ++i) {
    lo = std::min(lo, est[i]);
    hi = std::max(hi, est[i]);
    sum += est[i];
  }
  if (hi - lo > MAX_DISAGREE_IN) {
    printf("wall_reset: sensors disagree by %.2f in, rejecting\n", hi - lo);
    return false;
  }

  float field_pos = sum / n;
  if (axis == 0) {
    float x = field_pos - ORIGIN_X;
    chassis.setPose(x, p.y, p.theta);
    printf("wall_reset: x=%.2f (spread %.2f)\n", x, hi - lo);
  } else {
    float y = field_pos - ORIGIN_Y;
    chassis.setPose(p.x, y, p.theta);
    printf("wall_reset: y=%.2f (spread %.2f)\n", y, hi - lo);
  }
  reset_pending = true;
  return true;
}
void disabled() {}

void competition_initialize() {}

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

void test_drive_out_and_back() {
  auto p = chassis.getPose();
  chassis.moveToPoint(p.x, p.y + 48, 4000); // 48 in along heading 0
  chassis.waitUntilDone();
  chassis.moveToPoint(p.x, p.y, 4000, {.forwards = false});
  chassis.waitUntilDone();
  pros::delay(1500); // let it settle and the MCL catch up
  snapshot_request = true;
}

static void mcl_correct(int ms = 400) {
  mcl_apply_enabled = true;
  pros::delay(ms);           // ~8 loop iterations at 50 ms
  mcl_apply_enabled = false; // off again before any motion starts
}
void autonomous() {
  mcl_apply_enabled = false; // known state, in case opcontrol left it on

  // Red Right: back into the wall (unchanged)
  chassis.moveToPoint(0, -5, 650, {.forwards = false, .minSpeed = 85});
  chassis.waitUntil(4);
  chassis.cancelAllMotions();
  chassis.arcade(105, 0);
  pros::delay(650);
  chassis.arcade(-90, 0);
  pros::delay(450);
  chassis.arcade(105, 0);
  pros::delay(650);
  chassis.arcade(-60, 0);
  pros::delay(200);
  chassis.arcade(0, 0);
  chassis.cancelAllMotions();
  pros::delay(200);

  // Re-localize on the wall, retry once if it was rejected
  if (!wall_reset({BACK_L, BACK_R})) {
    pros::delay(100);
    wall_reset({BACK_L, BACK_R});
  }
  mcl_correct(400); // fixes x too, once you add a front/side sensor

  shitter.move(-90);
  rotate.move_absolute(720, 200);
  chassis.moveToPose(26, -21, -90, 2500,
                     {.forwards = false, .horizontalDrift = 4, .lead = 0.5});
  chassis.waitUntilDone();
  delay(150);

  // mcl_correct(400);         // add this before any further motion that needs
  // accuracy
  shitter.move(100);
}

void opcontrol() {
  // Task ok1(buttonFunctionPinControls);
  // Task ok2(buttonFunctionCasControls);
  // Task ok3(buttonFunctionShitControls);

  chassis.setBrakeMode(E_MOTOR_BRAKE_COAST);

  rotate.brake();

  while (true) {

    // A: wall reset (robot must be still)
    if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) &&
        std::abs(userInput.get_analog(ANALOG_LEFT_Y)) < 5 &&
        std::abs(userInput.get_analog(ANALOG_RIGHT_X)) < 5) {
      wall_reset({BACK_L, BACK_R});
    }
    // B: toggle MCL correction
    if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      mcl_apply_enabled = !mcl_apply_enabled;
      printf("MCL apply: %d\n", (int)mcl_apply_enabled.load());
    }
    // X: print one labeled snapshot (do this at each tape-measured spot)
    if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      snapshot_request = true;
    }
    // Y: nudge odometry 3 in off (correction test)
    if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      nudge_y_request = 3.0f;
    }
    chassis.arcade(userInput.get_analog(ANALOG_LEFT_Y),
                   userInput.get_analog(ANALOG_RIGHT_X), false, 0.40);

    delay(10);
  }
}