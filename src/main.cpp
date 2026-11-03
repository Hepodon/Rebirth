#include "main.h"
#include "controls.hpp"
#include "graphics.hpp"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/misc/lv_event.h"
#include "liblvgl/misc/lv_types.h"
#include "lvgl_Customs.hpp"
#include "mcl.hpp"
#include "portDef.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/screen.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <vector>

using namespace pros;

constexpr float MM_TO_IN = 1.0f / 25.4f;
constexpr float mm(float v) { return v * MM_TO_IN; }

constexpr float FRONT_TO_CENTER_IN = 10.0f;
constexpr float START_X = 0.0f;
constexpr float START_Y = FIELD_MAX - FRONT_TO_CENTER_IN;
constexpr float START_THETA = 0.0f;

constexpr size_t PARTICLES = 1000;

constexpr float GAIN_STILL = 0.30f;
constexpr float GAIN_MOVING = 0.08f;
constexpr float MAX_STEP_STILL = 1.5f;
constexpr float MAX_STEP_MOVING = 0.4f;
constexpr float AXIS_SPREAD_GATE = 1.0f;
constexpr float CORR_DEADBAND = 0.15f;
constexpr float CORR_MAX_ERR = 8.0f;
constexpr float MAX_TURN_DPS = 90.0f;
constexpr float LAG_S = 0.06f;

constexpr float FACE_RIGHT = 0, FACE_FRONT = 90, FACE_LEFT = 180,
                FACE_BACK = -90;

struct DistSensor {
  const char *name;
  pros::Distance *dev;
  float x_in, y_in;
  float facing_deg;
  bool use_in_mcl;
};

DistSensor dist_sensors[] = {
    //  name      device          x_in            y_in           facing      mcl
    {"frontL", &left_D, -mm(122.5f), mm(130.0f), FACE_FRONT, true},
    {"frontR", &right_D, mm(122.5f), mm(130.0f), FACE_FRONT, true},
    {"left", &side_left_D, -mm(140.0f), -mm(20.0f), FACE_LEFT, true},
    {"right", &side_right_D, mm(140.0f), -mm(20.0f), FACE_RIGHT, true},
};

enum SensorId { FRONT_L, FRONT_R, LEFT, RIGHT, SENSOR_COUNT };
constexpr int N_SENSORS = sizeof(dist_sensors) / sizeof(dist_sensors[0]);
static_assert(N_SENSORS == SENSOR_COUNT,
              "SensorId enum and dist_sensors[] are out of sync");

ad::MCL<PARTICLES> mcl;
ad::Point last_odom{0.0f, 0.0f};
std::atomic<bool> reset_pending{false};
std::atomic<bool> mcl_apply_enabled{true};
std::atomic<bool> snapshot_request{false};
std::atomic<float> nudge_y_request{0.0f};
pros::Task *mcl_task = nullptr;

struct Box {
  float x0, x1, y0, y1;
};
Box reset_box{};

static void request_particle_reset(float x0, float x1, float y0, float y1) {
  reset_box = {x0, x1, y0, y1}; // written before the flag
  reset_pending = true;
}

static bool valid_mm(int32_t v) { return v > 0 && v < 2000 && v != PROS_ERR; }

int global_localize();

static void build_readings(std::vector<ad::Reading> &out,
                           float lemlib_heading_deg, float extra_sd) {
  out.clear();
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
    sd = std::sqrt(sd * sd + extra_sd * extra_sd);

    ad::Position rel =
        ad::Position{s.x_in, s.y_in, ad::deg(s.facing_deg)}.rotate(rot);
    ad::Point dir{rel.theta.cos(), rel.theta.sin()};
    out.emplace_back(d, sd, rel.point(), rel.point() + dir);
  }
}

static void mcl_loop() {
  std::vector<ad::Reading> readings;
  readings.reserve(N_SENSORS);

  float prev_theta = chassis.getPose().theta;
  uint32_t prev_ms = pros::millis();

  while (true) {
    bool did_reset = reset_pending.exchange(false);
    auto pose = chassis.getPose();

    uint32_t now = pros::millis();
    float dt = std::max((now - prev_ms) * 0.001f, 0.01f);
    prev_ms = now;

    if (did_reset) {
      last_odom = {pose.x, pose.y};
      mcl.init_box(reset_box.x0, reset_box.x1, reset_box.y0, reset_box.y1);
    }

    float nudge = nudge_y_request.exchange(0.0f);
    if (nudge != 0.0f) {
      chassis.setPose(pose.x, pose.y + nudge, pose.theta);
      last_odom = {pose.x, pose.y + nudge};
      pose = chassis.getPose();
    }

    ad::Point cur{pose.x, pose.y};
    ad::Point d = cur - last_odom;
    last_odom = cur;

    float speed = d.hypot() / dt;
    float turn_dps =
        std::fabs(std::remainder(pose.theta - prev_theta, 360.0f)) / dt;
    prev_theta = pose.theta;

    mcl.predict(d.x, d.y, 0.03f * d.hypot());

    if (turn_dps < MAX_TURN_DPS) {
      build_readings(readings, pose.theta, LAG_S * speed);
      mcl.update(readings);
    }

    ad::Point est = mcl.estimate();

    if (mcl_apply_enabled) {
      bool moving = speed > 2.0f;
      float gain = moving ? GAIN_MOVING : GAIN_STILL;
      float max_step = moving ? MAX_STEP_MOVING : MAX_STEP_STILL;

      float sx, sy;
      mcl.spread_xy(sx, sy);

      auto step = [&](float err, float spread) -> float {
        float a = std::fabs(err);
        if (spread > AXIS_SPREAD_GATE || a < CORR_DEADBAND || a > CORR_MAX_ERR)
          return 0.0f;
        return std::clamp(gain * err, -max_step, max_step);
      };
      float cx = step(est.x - pose.x, sx);
      float cy = step(est.y - pose.y, sy);

      if (cx != 0.0f || cy != 0.0f) {
        auto p2 = chassis.getPose();
        chassis.setPose(p2.x + cx, p2.y + cy, p2.theta);
        last_odom = {cur.x + cx, cur.y + cy};
      }
    }

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

      pros::lcd::print(0, "odom %.1f %.1f th %.0f", pose.x, pose.y, pose.theta);
      pros::lcd::print(1, "mcl  %.1f %.1f", est.x, est.y);
      pros::lcd::print(2, "sx %.2f sy %.2f", sx, sy);
      pros::lcd::print(3, "neff %.0f apply %d", mcl.n_eff(),
                       (int)mcl_apply_enabled.load());
      pros::lcd::print(4, "%s %.1f  %s %.1f", dist_sensors[0].name,
                       dist_sensors[0].dev->get() * MM_TO_IN,
                       dist_sensors[1].name,
                       dist_sensors[1].dev->get() * MM_TO_IN);
      pros::lcd::print(5, "%s %.1f  %s %.1f", dist_sensors[2].name,
                       dist_sensors[2].dev->get() * MM_TO_IN,
                       dist_sensors[3].name,
                       dist_sensors[3].dev->get() * MM_TO_IN);
    }

    mcl.resample_if_needed();
    pros::delay(50);
  }
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

static void sample_all(int *out, int n = 5) {
  int v[N_SENSORS][8], k[N_SENSORS] = {};
  for (int i = 0; i < n && i < 8; ++i) {
    for (int s = 0; s < N_SENSORS; ++s) {
      int32_t x = dist_sensors[s].dev->get();
      if (valid_mm(x))
        v[s][k[s]++] = x;
    }
    pros::delay(30);
  }
  for (int s = 0; s < N_SENSORS; ++s) {
    if (k[s] < n / 2 + 1) {
      out[s] = -1;
      continue;
    }
    std::sort(v[s], v[s] + k[s]);
    out[s] = v[s][k[s] / 2];
  }
}

int global_localize() {
  constexpr float MAX_TILT_DEG = 10.0f, MIN_DISAGREE_IN = 1.5f;
  auto p = chassis.getPose();
  ad::Rotation rot = ad::Rotation::deg(-p.theta);

  int raw[N_SENSORS];
  sample_all(raw);

  float sumw[2] = {0, 0}, sume[2] = {0, 0};
  float lo[2] = {1e9f, 1e9f}, hi[2] = {-1e9f, -1e9f}, maxd[2] = {0, 0};
  int cnt[2] = {0, 0};

  for (int id = 0; id < N_SENSORS; ++id) {
    if (raw[id] < 0)
      continue;
    const DistSensor &s = dist_sensors[id];

    ad::Rotation face = ad::deg(s.facing_deg);
    ad::Point u = ad::Point{face.cos(), face.sin()}.rotate(rot);
    int a = std::fabs(u.x) > std::fabs(u.y) ? 0 : 1;
    float along = a == 0 ? u.x : u.y;
    float across = a == 0 ? u.y : u.x;
    float sg = along > 0 ? 1.0f : -1.0f;
    if (std::atan2(std::fabs(across), std::fabs(along)) * 180.0f / M_PI >
        MAX_TILT_DEG)
      continue;

    float dist_in = raw[id] * MM_TO_IN;
    float wall = sg > 0 ? FIELD_MAX : FIELD_MIN;
    float sensor_pos = wall - sg * dist_in * std::fabs(along);
    ad::Point off = ad::Point{s.x_in, s.y_in}.rotate(rot);
    float e = sensor_pos - (a == 0 ? off.x : off.y);

    float w = 1.0f / std::max(dist_in * dist_in, 16.0f);
    sumw[a] += w;
    sume[a] += w * e;
    cnt[a]++;
    lo[a] = std::min(lo[a], e);
    hi[a] = std::max(hi[a], e);
    maxd[a] = std::max(maxd[a], dist_in);
  }

  int found = 0;
  float nx = p.x, ny = p.y;
  for (int a = 0; a < 2; ++a) {
    if (cnt[a] == 0)
      continue;
    float tol =
        std::max(MIN_DISAGREE_IN, 0.08f * maxd[a]); // far walls are noisier
    if (hi[a] - lo[a] > tol) {
      printf("localize: %c sensors disagree by %.2f in\n", a ? 'y' : 'x',
             hi[a] - lo[a]);
      continue;
    }
    (a == 0 ? nx : ny) = sume[a] / sumw[a];
    found |= 1 << a;
  }

  chassis.setPose(nx, ny, p.theta);
  float mx = (found & 1) ? std::max(1.5f, 0.05f * maxd[0]) : 4.0f;
  float my = (found & 2) ? std::max(1.5f, 0.05f * maxd[1]) : 4.0f;
  request_particle_reset(nx - mx, nx + mx, ny - my, ny + my);
  printf("localize: found=%d pose %.2f, %.2f\n", found, nx, ny);
  pros::lcd::print(7, "loc %d %.1f %.1f", found, nx, ny);
  return found;
}

bool wall_reset(std::initializer_list<int> ids) {
  constexpr float MAX_TILT_DEG = 10.0f;
  constexpr float MAX_DISAGREE_IN = 1.5f;

  auto p = chassis.getPose();
  ad::Rotation rot = ad::Rotation::deg(-p.theta);

  int axis = -1;
  float sgn = 0.0f;
  float est[8];
  int n = 0;

  for (int id : ids) {
    if (id < 0 || id >= N_SENSORS || n >= 8)
      return false;
    const DistSensor &s = dist_sensors[id];

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
      pros::lcd::print(7, "reject: tilt %.0f", tilt_deg);
      return false;
    }
    if (n == 0) {
      axis = a;
      sgn = sg;
    } else if (a != axis || sg != sgn) {
      printf("wall_reset: sensors don't face the same wall\n");
      pros::lcd::print(7, "reject: diff walls");
      return false;
    }

    int raw = median_mm(*s.dev);
    if (raw < 0) {
      printf("wall_reset: %s has no valid reading\n", s.name);
      pros::lcd::print(7, "reject: no read");
      return false;
    }

    float wall = sg > 0 ? FIELD_MAX : FIELD_MIN;
    float sensor_pos = wall - sg * (raw * MM_TO_IN) * std::fabs(along);

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
    pros::lcd::print(7, "reject: disagree %.1f", hi - lo);
    return false;
  }

  float field_pos = sum / n;
  float nx = p.x, ny = p.y;
  if (axis == 0)
    nx = field_pos;
  else
    ny = field_pos;
  chassis.setPose(nx, ny, p.theta);
  printf("wall_reset: %c=%.2f (spread %.2f)\n", axis == 0 ? 'x' : 'y',
         field_pos, hi - lo);
  pros::lcd::print(7, "reset %c=%.2f", axis == 0 ? 'x' : 'y', field_pos);
  request_particle_reset(nx - 1.5f, nx + 1.5f, ny - 1.5f, ny + 1.5f);
  return true;
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

// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================
// ===================================================================================

void initialize() {}

void disabled() {}

void competition_initialize() {
  chassis.calibrate(true);
  pros::lcd::initialize();

  chassis.setPose(START_X, START_Y, START_THETA);
  mcl.init_box(START_X - 3, START_X + 3, START_Y - 3, START_Y + 3);
  last_odom = {START_X, START_Y};

  global_localize();

  mcl_task = new pros::Task([] { mcl_loop(); });
  mcl_apply_enabled = true;
}

void autonomous() {
  shitter.move(-90);
  rotate.move_absolute(720, 200);

  chassis.moveToPoint(START_X, START_Y - 5, 650,
                      {.forwards = false, .minSpeed = 85});
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

  chassis.moveToPose(20, 20, 90, 10000, {.forwards = false});
  chassis.waitUntilDone();
}

void opcontrol() {
  Task ok1(buttonFunctionPinControls);
  Task ok2(buttonFunctionCasControls);
  Task ok3(buttonFunctionShitControls);

  chassis.setBrakeMode(E_MOTOR_BRAKE_COAST);

  rotate.brake();

  while (true) {
    // bool still = std::abs(userInput.get_analog(ANALOG_LEFT_Y)) < 5 &&
    //              std::abs(userInput.get_analog(ANALOG_RIGHT_X)) < 5;

    // // A: wall reset on the front sensors (robot must be still)
    // if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) &&
    //     still) {
    //   wall_reset({FRONT_L, FRONT_R});
    // }
    // // UP: full localize from every sensor that sees a wall (robot must be
    // // still)
    // if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) &&
    //     still) {
    //   global_localize();
    // }
    // // B: toggle continuous MCL correction
    // if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
    //   mcl_apply_enabled = !mcl_apply_enabled;
    //   printf("MCL apply: %d\n", (int)mcl_apply_enabled.load());
    // }
    // // X: print one labeled snapshot (do this at each tape-measured spot)
    // if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
    //   snapshot_request = true;
    // }
    // // Y: nudge pose 3 in off (correction test)
    // if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
    //   nudge_y_request = 3.0f;
    // }

    chassis.arcade(userInput.get_analog(ANALOG_LEFT_Y),
                   userInput.get_analog(ANALOG_RIGHT_X), false, 0.40);

    delay(10);
  }
}