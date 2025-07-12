#include "definitions.h"

#ifndef display_h
#define display_h

extern bool dimmed;
extern bool disp_updateRequired;

void display_init();
void setBrightness();
void update_display();
void display_off();

void drawIdleScreen();
void drawGbWIdleScreen();
void drawGrindingOrPurgingScreen();
void drawGrindingGbwScreen();
void drawCalibratingScreen();
void drawMenuScreen(int menuNum, int numItems, int selectedIdx);
void drawMenuValueScreen(int menuNum, int selectedIdx);


#endif