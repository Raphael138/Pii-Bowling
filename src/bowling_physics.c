/**
 * Implementation of bowling_physics_update function for the bowling game physics engine
 */

 #include "bowling_physics.h"
 #include <stdio.h>
 #include <string.h>
 #include <math.h>

 // Access to the game's Pin structure
 typedef struct {
   uint pin_radius;
   uint pin_x;
   uint pin_y;
   bool fallen;
 } GamePin;
 
 // Vector operations
 Vector2D vector_add(Vector2D a, Vector2D b) {
     Vector2D result = {a.x + b.x, a.y + b.y};
     return result;
 }
 
 Vector2D vector_subtract(Vector2D a, Vector2D b) {
     Vector2D result = {a.x - b.x, a.y - b.y};
     return result;
 }
 
Vector2D vector_scale(Vector2D a, float scale) {
     Vector2D result = {a.x * scale, a.y * scale};
     return result;
 }
 
float vector_length(Vector2D a) {
     return sqrtf(a.x * a.x + a.y * a.y);
 }
 
float vector_dot(Vector2D a, Vector2D b) {
     return a.x * b.x + a.y * b.y;
 }
 
Vector2D vector_normalize(Vector2D a) {
     float length = vector_length(a);
     if (length < 0.0001f) {
         Vector2D zero = {0, 0};
         return zero;
     }
     Vector2D result = {a.x / length, a.y / length};
     return result;
 }
 
// Initialize the bowling simulation
void initialize_simulation(BowlingSimulation* simulation, BowlingTrajectory* bowling_trajectory, Pin game_pins[]) {
    // Clear the structure
    memset(simulation, 0, sizeof(BowlingSimulation));
    
    // Setup pins in standard following given 10-pin arrangement
    simulation->pins.pin_count = 10;
        
    for (size_t i = 0; i < simulation->pins.pin_count; i++) {
        simulation->pins.pins[i].position.x = game_pins[i].pin_x;
        simulation->pins.pins[i].position.y = game_pins[i].pin_y;
        simulation->pins.pins[i].is_standing = !game_pins[i].fallen;
        simulation->pins.pins[i].angle = 0.0f;
        simulation->pins.pins[i].is_hit = false;
        simulation->pins.pins[i].to_simulate = !game_pins[i].fallen;
    }
    
    // Setup ball
    simulation->ball.is_active = true;
    simulation->ball.spin = bowling_trajectory->spin_rate;
    simulation->ball.position.x = 0;
    simulation->ball.position.y = bowling_trajectory->y_pos;
    float speed_pixels = bowling_trajectory->release_speed * GYRO_SPEED_TO_SCREEN_SPEED;
    simulation->ball.velocity.x = speed_pixels * cosf(bowling_trajectory->direction_angle);
    simulation->ball.velocity.y = speed_pixels * sinf(bowling_trajectory->direction_angle);
    simulation->ball.initial_speed = sqrtf(simulation->ball.velocity.x * simulation->ball.velocity.x + simulation->ball.velocity.y * simulation->ball.velocity.y);

    simulation->frame_count = 0;
    simulation->simulation_complete = false;
}

// Check if two circles are colliding
static bool check_circle_collision(Vector2D center1, float radius1, 
                                Vector2D center2, float radius2) {
    Vector2D diff = vector_subtract(center1, center2);
    float distance_squared = diff.x * diff.x + diff.y * diff.y;
    float min_distance = radius1 + radius2;
    
    return distance_squared < (min_distance * min_distance);
}
 
// Handle collision between ball and pin
static void handle_ball_pin_collision(Ball* ball, PhysicsPin* pin, float pin_radius) {
    // Calculate the vector from the ball to the pin
    Vector2D normal = vector_subtract(pin->position, ball->position);
    float distance = vector_length(normal);

    if (distance >= (BALL_RADIUS + pin_radius)) return; // If no collision, return

    // Mark the pin as hit
    pin->is_hit = true;

    // Normalize the collision normal
    normal = vector_normalize(normal);

    // Calculate relative velocity
    Vector2D rel_velocity = vector_subtract(ball->velocity, pin->velocity);

    // Calculate impulse
    float vel_along_normal = vector_dot(rel_velocity, normal);

    // If the objects are moving away from each other, don't apply impulse
    if (vel_along_normal > 0) {
        return;
    }

    // Calculate impulse scalar
    float impulse_scalar = (1.0f + ELASTICITY) * vel_along_normal / 
                        (1.0f / BALL_MASS + 1.0f / PIN_MASS);

    // Apply impulse to both objects
    Vector2D impulse = vector_scale(normal, impulse_scalar);
    
    // Apply impulse to the pin
    pin->velocity = vector_add(pin->velocity, vector_scale(impulse, 1.0f / PIN_MASS));
    
    // Apply impulse to the ball (new code)
    ball->velocity.x -= impulse.x * (1.0f / BALL_MASS);
    ball->velocity.y -= impulse.y * (1.0f / BALL_MASS);

    // Apply angular velocity to the pin based on collision
    Vector2D perp = {-normal.y, normal.x};
    float angular_impulse = vector_dot(perp, rel_velocity) * 0.2f;
    pin->angular_vel += angular_impulse;
    
    // Adjust ball spin based on collision (optional)
    ball->spin += angular_impulse * 0.1f;
}
 
// Handle collision between pins
static void handle_pin_pin_collision(PhysicsPin* pin1, PhysicsPin* pin2, float pin_radius) {
    // Calculate the vector from pin1 to pin2
    Vector2D normal = vector_subtract(pin2->position, pin1->position);
    float distance = vector_length(normal);
    
    // If no collision, return
    if (distance >= (2 * pin_radius)) {
        return;
    }
     
     // Normalize the collision normal
     normal = vector_normalize(normal);
     
     // Calculate relative velocity
     Vector2D rel_velocity = vector_subtract(pin1->velocity, pin2->velocity);
     
     // Calculate impulse
     float vel_along_normal = vector_dot(rel_velocity, normal);
     
     // If the objects are moving away from each other, don't apply impulse
     if (vel_along_normal > 0) {
         return;
     }
     
     // Calculate impulse scalar
     float impulse_scalar = -(1.0f + ELASTICITY) * vel_along_normal / 
                           (1.0f / PIN_MASS + 1.0f / PIN_MASS);
     
     // Apply impulse to pin velocities
     Vector2D impulse = vector_scale(normal, impulse_scalar);
     
     pin1->velocity = vector_subtract(pin1->velocity, 
                                    vector_scale(impulse, 1.0f / PIN_MASS));
     
     pin2->velocity = vector_add(pin2->velocity, 
                               vector_scale(impulse, 1.0f / PIN_MASS));
     
     // Apply angular velocity based on collision
     Vector2D perp = {-normal.y, normal.x};
     float angular_impulse = vector_dot(perp, rel_velocity) * 0.2f;
     pin1->angular_vel -= angular_impulse;
     pin2->angular_vel += angular_impulse;
     
     // Separate the objects to prevent sticking
     float overlap = (2 * pin_radius) - distance;
     Vector2D separation = vector_scale(normal, overlap * 0.5f);
     
     pin1->position = vector_subtract(pin1->position, vector_scale(separation, 0.5f));
     pin2->position = vector_add(pin2->position, vector_scale(separation, 0.5f));
}
 
static void handle_pin_alley_collision(PhysicsPin* pin, float pin_radius, int center_y, int alley_width) {
    // Check for collision with left wall
     if (pin->position.y < center_y - alley_width / 2 + pin_radius) {
         pin->position.y = center_y - alley_width / 2 + pin_radius;
         pin->velocity.y = -pin->velocity.y * ELASTICITY;
         pin->angular_vel += pin->velocity.x * 0.05f; // Add some angular velocity
     }
     
     // Check for collision with right wall
     if (pin->position.y > center_y + alley_width / 2 - pin_radius) {
         pin->position.y = center_y + alley_width / 2 - pin_radius;
         pin->velocity.y = -pin->velocity.y * ELASTICITY;
         pin->angular_vel -= pin->velocity.x * 0.05f; // Add some angular velocity
     }
}
 
// Update pin physics
static void update_pin(PhysicsPin* pin, float pin_radius, float delta_time, int center_x, int alley_width) {
    // If the pin is knocked down and stopped moving, don't update
    if (!pin->is_standing && vector_length(pin->velocity) < 0.01f && 
        fabsf(pin->angular_vel) < 0.01f) {
        return;
    }

    // Apply friction
    pin->velocity = vector_scale(pin->velocity, FRICTION);
    pin->angular_vel *= FRICTION;     

    // Update position based on velocity
    pin->position = vector_add(pin->position, 
                            vector_scale(pin->velocity, delta_time));

    // Update angle based on angular velocity
    pin->angle += pin->angular_vel * delta_time;
    
    // Consider a pin knocked down if its angle exceeds threshold
    if (fabsf(pin->angle) > PIN_TOPPLE_THRESHOLD && pin->is_standing) {
        pin->is_standing = false;
    }

    // Handle collision with alley walls
    handle_pin_alley_collision(pin, pin_radius, center_x, alley_width);
}
 
// Update ball velocity and position
void update_ball(BowlingSimulation* bowling_simulation) {
    // If hit either gutter, zero out y velocity
    if (bowling_simulation->ball.position.y + bowling_simulation->ball.velocity.y * time_delta <= _left_gutter || bowling_simulation->ball.position.y + bowling_simulation->ball.velocity.y * time_delta >= _right_gutter) {
        bowling_simulation->ball.velocity.y = 0;
        bowling_simulation->ball.position.y = bowling_simulation->ball.position.y > _center_y ? _right_gutter: _left_gutter;
        bowling_simulation->ball.position.x += bowling_simulation->ball.velocity.x * time_delta;
        return;
    }
    
    // Update position
    bowling_simulation->ball.position.x += bowling_simulation->ball.velocity.x * time_delta;
    bowling_simulation->ball.position.y += bowling_simulation->ball.velocity.y * time_delta;
    
    // Update speed and spin
    bowling_simulation->ball.velocity.x *= (1.0f - BALL_FRICTION * time_delta);
    bowling_simulation->ball.velocity.y *= (1.0f - BALL_FRICTION * time_delta);
    bowling_simulation->ball.spin *= (1.0f - BALL_FRICTION * 0.3f * time_delta);

    // Update speed given spin
    float current_speed = sqrtf(bowling_simulation->ball.velocity.x * bowling_simulation->ball.velocity.x + bowling_simulation->ball.velocity.y * bowling_simulation->ball.velocity.y);
    float speed_ratio = current_speed / bowling_simulation->ball.initial_speed;
    float spin_factor = 1.0f / (0.1f + speed_ratio * speed_ratio); 
    bowling_simulation->ball.velocity.y += bowling_simulation->ball.spin * spin_impact_constant * spin_factor * time_delta;
}

// Main physics update function that integrates with the game loop
void bowling_physics_update(BowlingSimulation* bowling_simulation, Pin pins[]) {
    // Update ball velocity
    update_ball(bowling_simulation);
    
    // Check for collisions between ball and pins
    for (size_t i = 0; i < bowling_simulation->pins.pin_count; i++) {
        if (bowling_simulation->pins.pins[i].to_simulate)
            handle_ball_pin_collision(&bowling_simulation->ball, &bowling_simulation->pins.pins[i], PIN_RADIUS);
    }

    // Update pins
    for (size_t i = 0; i < bowling_simulation->pins.pin_count; i++) {
        if (bowling_simulation->pins.pins[i].to_simulate)
            update_pin(&bowling_simulation->pins.pins[i], PIN_RADIUS, time_delta, _center_y, _bowling_alley_width);

        // Check for pin-pin collisions
        for (size_t j = i + 1; j < bowling_simulation->pins.pin_count; j++) {
            if (bowling_simulation->pins.pins[i].to_simulate)
                handle_pin_pin_collision(&bowling_simulation->pins.pins[i], &bowling_simulation->pins.pins[j], PIN_RADIUS);
        }
    }

    // Increment frame count
    bowling_simulation->frame_count++;

    // If ball location is outside -> Mark simulation as complete
    float ball_speed = sqrtf(bowling_simulation->ball.velocity.x * bowling_simulation->ball.velocity.x + bowling_simulation->ball.velocity.y * bowling_simulation->ball.velocity.y);
    if (bowling_simulation->ball.position.x>_width || ball_speed < 1) {
        bowling_simulation->simulation_complete = true;
    }
}