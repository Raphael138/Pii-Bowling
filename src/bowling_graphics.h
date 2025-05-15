#ifndef BOWLING_GRAPHICS_H
#define BOWLING_GRAPHICS_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "bowling_physics.h"

// Drawing constants
#define BALL_DISPLAY_RADIUS 8
#define HOLE_DISPLAY_RADIUS 2
#define _pin_radius 8

// Bowling display constants
#define _pin_distance 24
#define _bowling_alley_width 160

void drawRotatedPin(int x, int y, float angle_degrees, bool erase);
void drawPin(int idx, Pin game_pins[], BowlingSimulation* bowling_simulation);
void drawBall(Vector2D position, Vector2D velocity, char COLOR); 
void drawGutters();
void drawBackground();

#endif