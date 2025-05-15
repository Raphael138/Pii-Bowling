#include <math.h>
#include "bowling_graphics.h"
#include "vga16_graphics.h"
#include "bowling_physics.h"

#define DEG2RAD(angle) ((angle) * M_PI / 180.0)
#define RAD2DEG(angle) ((angle) * 180.0 / M_PI)

char pin_pixels[27][9] = {
  {16, 16, 16, WHITE, WHITE, WHITE, 16, 16, 16},
  {16, 16, WHITE, WHITE, WHITE, WHITE, WHITE, 16, 16},
  {16, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, 16},
  {16, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, 16},
  {16, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, 16},
  {16, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, 16},
  {16, 16, RED, RED, RED, RED, RED, 16, 16},
  {16, 16, RED, RED, RED, RED, RED, 16, 16},
  {16, 16, 16, WHITE, WHITE, WHITE, 16, 16, 16},
  {16, 16, 16, WHITE, WHITE, WHITE, 16, 16, 16},
  {16, 16, 16, WHITE, WHITE, WHITE, 16, 16, 16},
  {16, 16, WHITE, WHITE, WHITE, WHITE, WHITE, 16, 16},
  {16, 16, WHITE, WHITE, WHITE, WHITE, WHITE, 16, 16},
  {16, RED, RED, RED, RED, RED, RED, RED, 16},
  {16, RED, RED, RED, RED, RED, RED, RED, 16},
  {WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE},
  {WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE},
  {WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE},
  {WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE},
  {WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE},
  {WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE},
  {WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE},
  {WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE},
  {WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE},
  {16, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, 16},
  {16, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, 16},
  {16, 16, WHITE, WHITE, WHITE, WHITE, WHITE, 16, 16},
};

void drawRotatedPin(int x, int y, float angle_degrees, bool erase) {
  float angle_rad = DEG2RAD(angle_degrees);
  float cos_a = cos(angle_rad);
  float sin_a = sin(angle_rad);

  float cx = 9.0 / 2.0;
  float cy = 27.0 / 2.0;

  // Use inverse rotation technique (scan destination image)
  // This eliminates holes by ensuring every output pixel gets a value
  for (int out_y = 0; out_y < 30; out_y++) {  // Use a safe bounding box
    for (int out_x = -10; out_x < 20; out_x++) {
      if (erase) {
        drawPixel(x + out_x, y + out_y, BLACK);
        continue;
      }
      // Translate to rotation center
      float fx = out_x - cx;
      float fy = out_y - cy;
      
      // Apply inverse rotation
      float src_x = fx * cos_a + fy * sin_a;
      float src_y = -fx * sin_a + fy * cos_a;
      
      // Translate back
      src_x += cx;
      src_y += cy;
      
      // Get source pixel if within bounds
      if (src_x >= 0 && src_x < 9 && src_y >= 0 && src_y < 27) {
        int src_px = round(src_x);
        int src_py = round(src_y);
        char color = pin_pixels[src_py][src_px];
        if (color >= 0 && color < 16) {
          drawPixel(x + out_x, y + out_y, color);
        }
      }
    }
  }
}

void drawPin(int idx, Pin game_pins[], BowlingSimulation* bowling_simulation) {
  if (!game_pins[idx].fallen) {
    // Draw standing pin as white with concentric red circles
    fillCircle(game_pins[idx].pin_x, game_pins[idx].pin_y, game_pins[idx].pin_radius, WHITE);
    drawCircle(game_pins[idx].pin_x, game_pins[idx].pin_y, game_pins[idx].pin_radius - 3, RED);
  } else {
    fillCircle(game_pins[idx].pin_x, game_pins[idx].pin_y, game_pins[idx].pin_radius+10, BLACK);

    // For fallen pins, calculate the direction the pin fell based on physics
    float dx = bowling_simulation->pins.pins[idx].position.x - game_pins[idx].pin_x;
    float dy = bowling_simulation->pins.pins[idx].position.y - game_pins[idx].pin_y;

    printf("TO REMOVE: %f %f\n", dx, dy);
    float dir_angle = atan2((double) dx, (double) dy);

    drawRotatedPin(game_pins[idx].pin_x, game_pins[idx].pin_y, RAD2DEG(dir_angle), false);
  }
}

void drawBall(Vector2D position, Vector2D velocity, char COLOR) {
  // Draw the ball
  fillCircle(position.x, position.y, BALL_DISPLAY_RADIUS, COLOR);
  
  // Calculate rotation angle based on distance traveled
  // We'll use the distance traveled to determine the rotation angle
  static float rotation = 0.0f;
  
  // Update rotation based on velocity
  float speed = sqrtf(velocity.x * velocity.x + velocity.y * velocity.y);
  rotation += speed * 0.01f; // Adjust this factor to control rotation speed
  
  // Keep rotation in [0, 2π) range
  if (rotation > 2 * M_PI) {
      rotation -= 2 * M_PI;
  }
  
  // Calculate positions for three holes based on rotation
  float angle1 = rotation;
  float angle2 = rotation + 2 * M_PI / 3;
  float angle3 = rotation + 4 * M_PI / 3;
  
  // Calculate offset for holes - slightly off-center
  float hole_offset = BALL_DISPLAY_RADIUS * 0.5f;
  
  // Draw the three holes - these will "rotate" as the ball moves
  // Only draw the holes if they would be visible (front half of the ball)
  if (cos(angle1) > 0) {
      fillCircle(position.x + cos(angle1) * hole_offset, 
                position.y + sin(angle1) * hole_offset, 
                HOLE_DISPLAY_RADIUS, BLACK);
  }
  
  if (cos(angle2) > 0) {
      fillCircle(position.x + cos(angle2) * hole_offset, 
                position.y + sin(angle2) * hole_offset, 
                HOLE_DISPLAY_RADIUS, BLACK);
  }
  
  if (cos(angle3) > 0) {
      fillCircle(position.x + cos(angle3) * hole_offset, 
                position.y + sin(angle3) * hole_offset, 
                HOLE_DISPLAY_RADIUS, BLACK);
  }
}

void drawGutters() {
  // Draw top gutter
  drawHLine(0, _center_y-_bowling_alley_width/2-_pin_radius, _width, WHITE);
  drawHLine(0, _center_y-_bowling_alley_width/2+_pin_radius, _width, WHITE);

  // Draw bottom gutter
  drawHLine(0, _center_y+_bowling_alley_width/2-_pin_radius, _width, WHITE);
  drawHLine(0, _center_y+_bowling_alley_width/2+_pin_radius, _width, WHITE);
  
  // Add 45-degree lines to top gutter
  // Distance between each line is 40 pixels
  for (int x = 0; x < _width; x += 40) {
    // For top gutter: from (x, top_gutter_top) to (x + gutter_height, top_gutter_bottom)
    int top_gutter_top = _center_y-_bowling_alley_width/2-_pin_radius;
    int top_gutter_bottom = _center_y-_bowling_alley_width/2+_pin_radius;
    int gutter_height = 2 * _pin_radius; // Height of the gutter
    
    // Draw the 45-degree line for top gutter
    drawLine(x, top_gutter_top, x + gutter_height, top_gutter_bottom, WHITE);
  }
  
  // Add 45-degree lines to bottom gutter
  // Distance between each line is 40 pixels
  for (int x = 0; x < _width; x += 40) {
    // For bottom gutter: from (x, bottom_gutter_top) to (x + gutter_height, bottom_gutter_bottom)
    int bottom_gutter_top = _center_y+_bowling_alley_width/2-_pin_radius;
    int bottom_gutter_bottom = _center_y+_bowling_alley_width/2+_pin_radius;
    int gutter_height = 2 * _pin_radius; // Height of the gutter
    
    // Draw the 45-degree line for bottom gutter
    drawLine(x, bottom_gutter_top, x + gutter_height, bottom_gutter_bottom, WHITE);
  }
}

void drawBackground() {
  int top_of_bottom = _center_y+_bowling_alley_width+_pin_radius;
  int top_of_top = _center_y - _bowling_alley_width - _pin_radius;
  int vertical_pin_distance = _pin_distance * 1.5;

  // Draw bottom lane
  for (int i = 0; i < 5; i++) {
    int x = _width-115 + _pin_distance * i;
    int y = top_of_bottom - (i-1) * (vertical_pin_distance/2); // Y position of the pin
      // For each pin in the row 
      for (int j = 0; j < i; j++) {
        fillCircle(x, y, _pin_radius, WHITE);
        drawCircle(x, y, _pin_radius - 3, RED);
            
        y += vertical_pin_distance; // Move to the next pin in the row
    }
  }

  // Draw top lane
  int x = _width-115 + _pin_distance * 4;
  int y = top_of_top - 3 * (vertical_pin_distance/2);
  for (int i =0; i < 4; i++) {
    fillCircle(x, y, _pin_radius, WHITE);
    drawCircle(x, y, _pin_radius - 3, RED);
        
    y += vertical_pin_distance; // Move to the next pin in the row
  }

  // Blank out almost all balls
  fillRect(600, 20, 20, 120, BLACK);

  // Blank out top ball
  for (int j = 12; j < 20; j++)
    for (int i = 613; i < 613 + (j-12); i++) 
      drawPixel(i, j, BLACK);
  
}