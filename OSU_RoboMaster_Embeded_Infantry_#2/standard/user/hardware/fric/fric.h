#ifndef FRIC_H
#define FRIC_H
#include "main.h"

#define Fric_UP 1400//摩擦轮右键转速
#define Fric_DOWN 1300//摩擦轮左键转速             1355 28m/s   1375 30//1350 27.5//1300 24.5
#define Fric_OFF 1000

extern void fric_PWM_configuration(void);
extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
