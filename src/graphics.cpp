#include "graphics.hpp"
#include "autons.hpp"
#include "liblvgl/core/lv_obj_style_gen.h"
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

// Defualt bg Color
lv_color_t bg_color = {lv_palette_main(LV_PALETTE_BLUE_GREY)};

lv_obj_t *startScreen;

screenStatus currentScreen = START;

namespace start {
lv_obj_t *autonButton;
lv_obj_t *diagButton;
lv_obj_t *title;
} // namespace start

lv_obj_t *autonScreen;

namespace auton {
lv_obj_t *backButton;
lv_obj_t *title;
lv_obj_t *leftAutonButton;
lv_obj_t *rightAutonButton;
lv_obj_t *soloAutonButton;
lv_obj_t *skillsAutonButton;

} // namespace auton

lv_obj_t *diagScreen;

lv_obj_t *tempScreen;
lv_obj_t *torqueScreen;
lv_obj_t *speedScreen;
lv_obj_t *headingScreen;

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

lv_obj_t *navContainer = nullptr;

void refreshNavBar(lv_obj_t *parent) {
  lv_obj_clean(navContainer);

  // LEFT
  createLvglButton(
      navContainer, "<",
      [](lv_event_t *e) {
        if (navStartIndex > 0) {
          navStartIndex--;
          refreshNavBar(navContainer);
        }
      },
      40, 30, LV_ALIGN_LEFT_MID, 0, 0, LV_PALETTE_LIGHT_BLUE, 0);

  // BUTTONS
  for (int i = 0; i < visibleButtons; i++) {
    int idx = navStartIndex + i;
    if (idx >= navItems.size())
      break;

    createLvglButton(navContainer, navItems[idx].label, navItems[idx].cb, 90,
                     30, LV_ALIGN_LEFT_MID, 45 + (i * 90), 0,
                     LV_PALETTE_LIGHT_BLUE, 0);
  }

  // RIGHT
  createLvglButton(
      navContainer, ">",
      [](lv_event_t *e) {
        if (navStartIndex + visibleButtons < navItems.size()) {
          navStartIndex++;
          refreshNavBar(navContainer);
        }
      },
      40, 30, LV_ALIGN_RIGHT_MID, 0, 0, LV_PALETTE_LIGHT_BLUE, 0);
}

void createNavBar(lv_obj_t *parent) {
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

  refreshNavBar(parent);
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

pros::Task masterUpdater([] {
  while (true) {
    for (int i = 0; i < sizeof(motors) / sizeof(motors[0]); i++) {
      if (currentScreen == TEMP) {
        // Temp
        motors[i].port[1] = pros::c::motor_get_temperature(motors[i].port[0]);
      } else if (currentScreen == TORQUE) {
        // Torque
        motors[i].port[2] = pros::c::motor_get_torque(motors[i].port[0]);
      } else if (currentScreen == SPEED) {
        // Speed
        motors[i].port[3] =
            pros::c::motor_get_actual_velocity(motors[i].port[0]);
      }
    }
    pros::delay(50);
  }
});

void loadStartScreen(lv_event_t *e) {
  currentScreen = START;
  lv_screen_load_anim(startScreen, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 500, 150,
                      false);
}

void loadAutonScreen(lv_event_t *e) {

  currentScreen = AUTON;

  auton::title =
      createLVGLText(autonScreen, "22204M Auton", LV_ALIGN_TOP_MID, 0, 6);

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

  lv_obj_set_style_text_font(auton::title, &lv_font_montserrat_16, 0);
  lv_screen_load_anim(autonScreen, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 150, false);
}

void loadDiagScreen(lv_event_t *e) {
  createNavBar(diagScreen);
  currentScreen = START;
  lv_screen_load_anim(diagScreen, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 250, false);
}

void screenInit() {
  startScreen = lv_obj_create(NULL);
  autonScreen = lv_obj_create(NULL);
  diagScreen = lv_obj_create(NULL);

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