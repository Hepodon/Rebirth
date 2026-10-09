#pragma once
#include "main.h"
#include "portDef.hpp"

enum AutonSel {
    Blue_L, Blue_R, Red_L, Red_R, Skills
};
extern AutonSel selectedAutonomo;


extern bool aready;

extern void L_rightAuton(lv_event_t *e);
extern void L_leftAuton(lv_event_t *e);
extern void R_rightAuton(lv_event_t *e);
extern void R_leftAuton(lv_event_t *e);
extern void skillsAuton(lv_event_t *e);