#include "main.h"
#include "portDef.hpp"
#include "pros/rtos.hpp"
#include "autons.hpp"

AutonSel selectedAutonomo = Blue_R;
bool aready = false;

void L_rightAuton(lv_event_t *e) {
  selectedAutonomo = Blue_R;
}

void L_leftAuton(lv_event_t *e) {
  selectedAutonomo = Blue_L;
}

void R_rightAuton(lv_event_t *e) {
  selectedAutonomo = Red_R;
}

void R_leftAuton(lv_event_t *e) {
  selectedAutonomo = Red_L;
}

void skillsAuton(lv_event_t *e) {}