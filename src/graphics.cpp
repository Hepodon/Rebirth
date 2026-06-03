#include "graphics.hpp"
#include "autons.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_style_gen.h"
#include "liblvgl/core/lv_obj_tree.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/lvgl.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_palette.h"
#include "lvgl_Customs.hpp"
#include "main.h"
#include "portDef.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include <exception>

lv_color_t bg_color;

lv_obj_t *startScreen;
lv_obj_t *autonScreen;
lv_obj_t *diagScreen;

screenStatus currentScreen = START;

static float motorData[sizeof(motors) / sizeof(motors[0])][3];

namespace start {
lv_obj_t *autonButton;
lv_obj_t *diagButton;
lv_obj_t *title;
} // namespace start

namespace auton {
lv_obj_t *backButton;
lv_obj_t *title;
lv_obj_t *leftAutonButton;
lv_obj_t *rightAutonButton;
lv_obj_t *soloAutonButton;
lv_obj_t *skillsAutonButton;
} // namespace auton

namespace heading {
lv_obj_t *headingChart;
Chartseries headingStru;
} // namespace heading

void loadStartScreen(lv_event_t *e);
void loadAutonScreen(lv_event_t *e);
void loadDiagScreen(lv_event_t *e);
void loadTempScreen(lv_event_t *e);
void loadTorqueScreen(lv_event_t *e);
void loadSpeedScreen(lv_event_t *e);
void loadHeadingScreen(lv_event_t *e);

lv_obj_t *backDiagButton;

int navStartIndex = 0;
const int visibleButtons = 4;

struct NavItem {
  const char *label;
  lv_event_cb_t cb;
};

std::vector<NavItem> navItems = {{"Temp", loadTempScreen},
                                 {"Torque", loadTorqueScreen},
                                 {"Speed", loadSpeedScreen},
                                 {"Heading", loadHeadingScreen}};

lv_obj_t *diagArcs[sizeof(motors) / sizeof(motors[0])];

lv_obj_t *navContainer = nullptr;

void refreshNavBar() {
  lv_obj_clean(navContainer);

  createLvglButton(
      navContainer, "<",
      [](lv_event_t *e) {
        if (navStartIndex > 0) {
          navStartIndex--;
          refreshNavBar();
        }
      },
      40, 30, LV_ALIGN_LEFT_MID, 0, 0, LV_PALETTE_LIGHT_BLUE, 0);

  for (int i = 0; i < visibleButtons; i++) {
    int idx = navStartIndex + i;
    if (idx >= (int)navItems.size())
      break;

    createLvglButton(navContainer, navItems[idx].label, navItems[idx].cb, 90,
                     30, LV_ALIGN_LEFT_MID, 45 + (i * 90), 0,
                     LV_PALETTE_LIGHT_BLUE, 0);
  }

  createLvglButton(
      navContainer, ">",
      [](lv_event_t *e) {
        if (navStartIndex + visibleButtons < (int)navItems.size()) {
          navStartIndex++;
          refreshNavBar();
        }
      },
      40, 30, LV_ALIGN_RIGHT_MID, 0, 0, LV_PALETTE_LIGHT_BLUE, 0);
}

void createNavBar(lv_obj_t *parent) {
  navStartIndex = 0;
  lv_obj_set_style_bg_color(parent, bg_color, 0);

  backDiagButton =
      createLvglButton(parent, "X", loadStartScreen, 30, 30,
                       LV_ALIGN_BOTTOM_LEFT, 5, -5, LV_PALETTE_PURPLE, 0);

  navContainer = lv_obj_create(parent);
  lv_obj_set_size(navContainer, 480, 40);
  lv_obj_align(navContainer, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_bg_opa(navContainer, 0, 0);
  lv_obj_remove_flag(navContainer, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_border_width(navContainer, 0, 0);

  refreshNavBar();
}

void update_arc_color(lv_obj_t *arc, int value, int maxValue) {
  lv_color_t color;
  color = lv_palette_main(value > maxValue * 0.85   ? LV_PALETTE_RED
                          : value > maxValue * 0.75 ? LV_PALETTE_DEEP_ORANGE
                          : value > maxValue * 0.65 ? LV_PALETTE_ORANGE
                          : value > maxValue * 0.50 ? LV_PALETTE_YELLOW
                          : value > maxValue * 0.35 ? LV_PALETTE_GREEN
                                                    : LV_PALETTE_BLUE);

  lv_obj_set_style_arc_color(arc, color, LV_PART_INDICATOR);
}

static bool tasksRunning = false;
static pros::Task *masterUpdater = nullptr;
static pros::Task *lvglUpdater = nullptr;

void startUpdaterTasks() {
  if (tasksRunning)
    return;
  tasksRunning = true;

  masterUpdater = new pros::Task([] {
    while (true) {
      for (int i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
        if (currentScreen == TEMP) {
          motorData[i][0] =
              (float)pros::c::motor_get_temperature(motors[i].port[0]);
        } else if (currentScreen == TORQUE) {
          motorData[i][1] =
              (float)pros::c::motor_get_torque(motors[i].port[0]) * 100;
        } else if (currentScreen == SPEED) {
          motorData[i][2] =
              (float)pros::c::motor_get_actual_velocity(motors[i].port[0]);
        }
      }
      pros::delay(20);
    }
  });
  static pros::Mutex lvgl_mutex;

  lvglUpdater = new pros::Task([] {
    while (true) {
      if (currentScreen == TEMP || currentScreen == TORQUE ||
          currentScreen == SPEED) {
        lvgl_mutex.take(TIMEOUT_MAX);
        for (int i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
          if (currentScreen == TEMP) {
            int val = (int)motorData[i][0];
            lv_arc_set_value(diagArcs[i], val);
            update_arc_color(diagArcs[i], val - 20, 42);
          } else if (currentScreen == TORQUE) {
            int val = (int)motorData[i][1];
            lv_arc_set_value(diagArcs[i], val);
            update_arc_color(diagArcs[i], val, 100);
          } else if (currentScreen == SPEED) {
            int val = (int)motorData[i][2];
            lv_arc_set_value(diagArcs[i], val);
            update_arc_color(diagArcs[i], val, motors[i].RPM);
          }
        }
        lvgl_mutex.give();
      }
      pros::delay(100);
    }
  });
}
void loadStartScreen(lv_event_t *e) {
  currentScreen = START;
  lv_screen_load_anim(startScreen, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 500, 150,
                      false);
}

void loadAutonScreen(lv_event_t *e) {
  lv_obj_clean(autonScreen);
  currentScreen = AUTON;

  auton::title =
      createLVGLText(autonScreen, "22204M Auton", LV_ALIGN_TOP_MID, 0, 6);
  lv_obj_set_style_text_font(auton::title, &lv_font_montserrat_16, 0);

  auton::backButton =
      createLvglButton(autonScreen, "X", loadStartScreen, 40, 40,
                       LV_ALIGN_TOP_LEFT, 14, 14, LV_PALETTE_LIGHT_BLUE, 5);

  auton::leftAutonButton = createLvglButton(
      autonScreen, "Left Auton", leftAuton, 115, 50, LV_ALIGN_CENTER, -60, 0);

  auton::rightAutonButton = createLvglButton(
      autonScreen, "Right Auton", rightAuton, 115, 50, LV_ALIGN_CENTER, 60, 0);

  auton::soloAutonButton =
      createLvglButton(autonScreen, "Right Solo Auton", soloRightAuton, 230, 60,
                       LV_ALIGN_CENTER, 0, -60);

  auton::skillsAutonButton =
      createLvglButton(autonScreen, "Skills Auton", skillsAuton, 230, 60,
                       LV_ALIGN_CENTER, 0, 60);

  lv_screen_load_anim(autonScreen, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 150, false);
}
void loadDiagScreen(lv_event_t *e) {
  if (navContainer == nullptr) {
    for (int i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
      diagArcs[i] = createLVGLArc(diagScreen, 0, 100, i, motors[i].name);
    }
    heading::headingChart =
        createLVGLChart(heading::headingStru, diagScreen, -180, 180);
    lv_obj_add_flag(heading::headingChart, LV_OBJ_FLAG_HIDDEN);
    createNavBar(diagScreen);
  }

  startUpdaterTasks();
  loadTempScreen(nullptr);
  lv_screen_load(diagScreen);
}

void loadTempScreen(lv_event_t *e) {
  if (heading::headingChart != nullptr) {
    lv_obj_add_flag(heading::headingChart, LV_OBJ_FLAG_HIDDEN);
  }
  for (int i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    lv_arc_set_range(diagArcs[i], 20, 62);
    lv_arc_set_value(diagArcs[i], 20);
    lv_obj_remove_flag(diagArcs[i], LV_OBJ_FLAG_HIDDEN);
  }
  currentScreen = TEMP;
}

void loadTorqueScreen(lv_event_t *e) {
  if (heading::headingChart != nullptr) {
    lv_obj_add_flag(heading::headingChart, LV_OBJ_FLAG_HIDDEN);
  }
  for (int i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    lv_arc_set_range(diagArcs[i], 0, 100);
    lv_arc_set_value(diagArcs[i], 0);
    lv_obj_remove_flag(diagArcs[i], LV_OBJ_FLAG_HIDDEN);
  }
  currentScreen = TORQUE;
}

void loadSpeedScreen(lv_event_t *e) {
  if (heading::headingChart != nullptr) {
    lv_obj_add_flag(heading::headingChart, LV_OBJ_FLAG_HIDDEN);
  }
  for (int i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    lv_arc_set_range(diagArcs[i], 0, motors[i].RPM);
    lv_arc_set_value(diagArcs[i], 0);
    lv_obj_remove_flag(diagArcs[i], LV_OBJ_FLAG_HIDDEN);
  }
  currentScreen = SPEED;
}

void loadHeadingScreen(lv_event_t *e) {
  for (int i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
    lv_obj_add_flag(diagArcs[i], LV_OBJ_FLAG_HIDDEN);
  }
  if (heading::headingChart != nullptr) {
    lv_obj_remove_flag(heading::headingChart, LV_OBJ_FLAG_HIDDEN);
  }
  currentScreen = HEADING;
}
void screen_init() {
  startScreen = lv_obj_create(NULL);
  autonScreen = lv_obj_create(NULL);
  diagScreen = lv_obj_create(NULL);
  heading::headingChart = nullptr;

  lv_obj_set_style_bg_grad_color(startScreen, bg_color, 0);
  lv_obj_set_style_bg_grad_color(autonScreen, bg_color, 0);
  lv_obj_set_style_bg_grad_color(diagScreen, bg_color, 0);

  start::autonButton = createLvglButton(startScreen, "Auton", loadAutonScreen,
                                        200, 65, LV_ALIGN_CENTER, 130, 20);
  start::diagButton = createLvglButton(startScreen, "Diag", loadDiagScreen, 200,
                                       65, LV_ALIGN_CENTER, -130, 20);
  start::title =
      createLVGLText(startScreen, "22204M", LV_ALIGN_TOP_MID, 0, -15);

  lv_screen_load(startScreen);
}