/**
 * bowling_physics.h
 * Header file for bowling pin and ball collision physics
 */

#ifndef BOWLING_PHYSICS_H
#define BOWLING_PHYSICS_H

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "mfe_algorithm.h"

// Vector structure for 2D physics
typedef struct {
    float x;
    float y;
} Vector2D;

// Pin structure for physics simulation
typedef struct {
    Vector2D position;
    Vector2D velocity;
    float angle;
    float angular_vel;
    bool is_standing;
    bool is_hit;
    bool to_simulate;
} PhysicsPin;

// Ball structure
typedef struct {
    Vector2D position;
    Vector2D velocity;
    float initial_speed;
    float spin;
    bool is_active;
} Ball;

// Pin arrangement structure
typedef struct {
    PhysicsPin pins[10];
    size_t pin_count;
} PinArrangement;

// Bowling simulation state
typedef struct {
    Ball ball;
    PinArrangement pins;
    int frame_count;
    bool simulation_complete;
} BowlingSimulation;

// Defining a pin struct
typedef struct {
    float pin_radius;
    float pin_x;
    float pin_y;
    bool fallen;
} Pin;

// Screen/convertion constants
#define BOWLING_LANE_LENGTH 18.29  // Standard bowling lane length in meters
#define LANE_TO_PIXELS (_width)
#define GYRO_SPEED_TO_SCREEN_SPEED 4* LANE_TO_PIXELS / BOWLING_LANE_LENGTH
#define _width 640
#define _center_y 240
#define _bowling_alley_width 160
#define _left_gutter _center_y - _bowling_alley_width/2
#define _right_gutter _center_y + _bowling_alley_width/2

// Physics constants
#define GRAVITY_PIXELS 0.05f
#define FRICTION 0.8f
#define BALL_FRICTION 0.15f
#define ELASTICITY 0.6f         
#define PIN_MASS 1.0f
#define BALL_MASS 20.0f
#define PIN_TOPPLE_THRESHOLD 0.1f
#define BALL_RADIUS 10
#define PIN_RADIUS 15.0f
#define time_delta .015f
#define spin_impact_constant 30.0f

// Vector operations
Vector2D vector_add(Vector2D a, Vector2D b);
Vector2D vector_subtract(Vector2D a, Vector2D b);
Vector2D vector_scale(Vector2D a, float scale);
float vector_length(Vector2D a);
float vector_dot(Vector2D a, Vector2D b);
Vector2D vector_normalize(Vector2D a);


void initialize_simulation(BowlingSimulation* bowling_simulation, BowlingTrajectory* bowling_trajectory, Pin pins[]);
void bowling_physics_update(BowlingSimulation* bowling_simulation, Pin pins[]);

#endif