/**
* Wii Bowling Motion Feature Extraction
* 
* This implementation processes IMU data from a bowling swing to estimate
* the trajectory and motion characteristics of a virtual bowling ball.
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "imu_container.h"
#include "mfe_algorithm.h"

// Constants
#define SAMPLE_RATE 100.0
#define AVG_ARM_LENGTH 0.8
#define DEG_RAD_CONVERSION (M_PI / 180.0)

#define MIN_DATA_SIZE 20

#define MY_PI 3.14159265358979323846

// Low-pass filter to reduce noise
void apply_low_pass_filter(IMU_Buffer* buffer, float alpha) {
  if (buffer->size < 2) return;
  for (size_t i = 1; i < buffer->size; i++) {
    buffer->data[i].ax = alpha * buffer->data[i].ax + (1 - alpha) * buffer->data[i-1].ax;
    buffer->data[i].ay = alpha * buffer->data[i].ay + (1 - alpha) * buffer->data[i-1].ay;
    buffer->data[i].az = alpha * buffer->data[i].az + (1 - alpha) * buffer->data[i-1].az;
    buffer->data[i].gx = alpha * buffer->data[i].gx + (1 - alpha) * buffer->data[i-1].gx;
    buffer->data[i].gy = alpha * buffer->data[i].gy + (1 - alpha) * buffer->data[i-1].gy;
    buffer->data[i].gz = alpha * buffer->data[i].gz + (1 - alpha) * buffer->data[i-1].gz;
  }
}

// Find the start and end indices of the throwing motion and check for swing validity 
// Improved validation function that better rejects random shaking
bool validate_swing(IMU_Buffer* buffer) {
  if (buffer->size < MIN_DATA_SIZE) {
    printf("Buffer size: %zu (minimum required: %d)\n", buffer->size, (int)MIN_DATA_SIZE);
    return false;
  }

  // Adjust thresholds to better match your data
  const float BACKSWING_THRESHOLD = 30.0f;  // Lowered from 40.0
  const float FORWARD_SWING_THRESHOLD = 40.0f;  // Lowered from 40.0
  const float ACCEL_MAG_THRESHOLD = 1.0f;  // Keep the same

  bool swing_started = false;
  size_t swing_start_idx = 0;
  size_t forward_swing_start_idx = 0;
  float max_accel = 0.0;
  float max_gyro = 0.0;
  
  bool backswing_detected = false;
  bool forward_swing_detected = false;
  
  // Counters for swing phase detection - more lenient requirements
  int backswing_count = 0;
  int forward_swing_count = 0;
  
  // First pass - detect swing phases and capture statistics
  for (size_t i = 1; i < buffer->size; i++) {
    // Calculate magnitudes
    float accel_mag = sqrt(
        buffer->data[i].ax * buffer->data[i].ax + 
        buffer->data[i].ay * buffer->data[i].ay + 
        buffer->data[i].az * buffer->data[i].az
    );
    
    float gyro_mag = sqrt(
        buffer->data[i].gx * buffer->data[i].gx + 
        buffer->data[i].gy * buffer->data[i].gy + 
        buffer->data[i].gz * buffer->data[i].gz
    );
    
    // Track maximum values
    if (accel_mag > max_accel) max_accel = accel_mag;
    if (gyro_mag > max_gyro) max_gyro = gyro_mag;
    
    // Use a sliding window approach rather than requiring consecutive frames
    // This is more robust to glitches and noisy data
    
    // For backswing detection - look for negative z rotation
    if (buffer->data[i].gz < -BACKSWING_THRESHOLD) {
      backswing_count++;
      
      // Mark the beginning of potential swing
      if (!swing_started) {
        swing_start_idx = i;
        swing_started = true;
      }
    }
    
    // For forward swing detection - look for positive z rotation
    // Only check if we've already detected some backswing motion
    if (swing_started && buffer->data[i].gz > FORWARD_SWING_THRESHOLD) {
      forward_swing_count++;
      
      // Mark the beginning of forward swing if not yet marked
      if (!forward_swing_detected && forward_swing_count == 1) {
        forward_swing_start_idx = i;
      }
    }
  }
  
  // Check if we have enough frames in each phase to constitute a valid swing
  // Much more lenient than before - only need 3 frames in each phase
  backswing_detected = backswing_count >= 3;
  forward_swing_detected = forward_swing_count >= 3;
  
  // Calculate swing time
  float swing_time = 0.0f;
  if (backswing_detected && forward_swing_detected) {
    swing_time = (float)(buffer->size - swing_start_idx) / SAMPLE_RATE;
  }
  
  // Check for pattern consistency
  bool swing_sequence_valid = false;
  if (backswing_detected && forward_swing_detected) {
    // Make sure forward swing happens after backswing
    swing_sequence_valid = forward_swing_start_idx > swing_start_idx;
  }
  
  // Second pass - calculate statistical features for shake detection
  float sum_accel = 0.0f;
  float sum_accel_squared = 0.0f;
  int accel_count = 0;
  
  // Count direction changes in z-gyro
  int direction_changes = 0;
  int prev_sign = 0;
  
  for (size_t i = 1; i < buffer->size; i++) {
    // Acceleration stats
    float accel_mag = sqrt(
        buffer->data[i].ax * buffer->data[i].ax + 
        buffer->data[i].ay * buffer->data[i].ay + 
        buffer->data[i].az * buffer->data[i].az
    );
    
    sum_accel += accel_mag;
    sum_accel_squared += accel_mag * accel_mag;
    accel_count++;
    
    // Direction change detection - more tolerant of small fluctuations
    int current_sign = 0;
    if (buffer->data[i].gz > 10.0f) current_sign = 1;
    else if (buffer->data[i].gz < -10.0f) current_sign = -1;
    
    if (prev_sign != 0 && current_sign != 0 && current_sign != prev_sign) {
      direction_changes++;
    }
    
    if (current_sign != 0) {
      prev_sign = current_sign;
    }
  }
  
  float mean_accel = sum_accel / accel_count;
  float variance_accel = (sum_accel_squared / accel_count) - (mean_accel * mean_accel);
  
  // A real bowling swing should have 1-3 major direction changes
  bool consistent_direction = (direction_changes >= 1 && direction_changes <= 5);
  
  // Shaking tends to have higher variance
  bool smooth_motion = (variance_accel < 6.0f);  // Slightly more lenient
  
  // Validate the throw - more lenient criteria
  bool valid_swing = backswing_detected && forward_swing_detected && 
                    swing_sequence_valid && consistent_direction && 
                    max_accel >= ACCEL_MAG_THRESHOLD && 
                    swing_time >= 0.3f && swing_time <= 3.0f;
  
  // Print diagnostic information
  printf("Backswing count: %d frames\n", backswing_count);
  printf("Forward swing count: %d frames\n", forward_swing_count);
  printf("Direction changes: %d\n", direction_changes);
  printf("Swing time: %.2f seconds\n", swing_time);
  printf("Max acceleration: %.2f\n", max_accel);
  printf("Max gyro: %.2f\n", max_gyro);
  printf("Variance in acceleration: %.2f\n", variance_accel);
  
  if (!valid_swing) {
    if (!backswing_detected) {
      printf("Backswing not detected (need at least 3 frames)\n");
    } else if (!forward_swing_detected) {
      printf("Forward swing not detected (need at least 3 frames)\n");
    } else if (!swing_sequence_valid) {
      printf("Invalid swing sequence (forward swing before backswing)\n");
    } else if (!consistent_direction) {
      printf("Too many direction changes: %d (expect 1-5)\n", direction_changes);
    } else if (swing_time < 0.3f || swing_time > 3.0f) {
      printf("Swing duration outside valid range: %.2f seconds\n", swing_time);
    } else {
      printf("Motion doesn't match bowling pattern\n");
    }
  } else {
    printf("Valid bowling swing detected\n");
  }
  
  return valid_swing;
}
 
// Parse IMU stream to get release speed 
void calculate_speed(IMU_Buffer* buffer, float* speed) {  
  // Find final speed by using high-pass filter and maximum speed
  float max_speed = 0.0f;
  float high_pass_speed = 0.0f;
  float high_pass_alpha = 0.8f;

  // We're primarily interested in the x-axis rotation for speed
  for (size_t i = 1; i < buffer->size; i++) {
    float current_speed = - buffer->data[i].gx*AVG_ARM_LENGTH*DEG_RAD_CONVERSION;
    high_pass_speed = current_speed * high_pass_alpha + high_pass_speed * (1 - high_pass_alpha);
    max_speed = fmaxf(max_speed, current_speed);
  }

  // Calculate final speed as a combination of max speed and high-pass filtered speed
  *speed = max_speed*0.5f + high_pass_speed*0.5f;
}
 
void calculate_spin(IMU_Buffer* buffer, float* spin) {
  float max_spin = 0.0f;
  float high_pass_spin = 0.0f;
  float high_pass_alpha = 0.9f;
  float max_abs_spin = 0.0f;
  int max_spin_sign = 1;

  // We're primarily interested in the y-axis rotation for spin
  for (size_t i = 0; i < buffer->size; i++) {
    float current_spin = buffer->data[i].gy*DEG_RAD_CONVERSION;
    high_pass_spin = current_spin * high_pass_alpha + high_pass_spin * (1 - high_pass_alpha);
    
    // Check if absolute value is greater than current max
    if (fabsf(current_spin) > max_abs_spin) {
      max_abs_spin = fabsf(current_spin);
      max_spin_sign = (current_spin >= 0) ? 1 : -1;
    }
    
    // Maintain the signed max value
    max_spin = max_abs_spin * max_spin_sign;
    
  }
  printf("Both spins: %f %f\n", max_spin, high_pass_spin);
  *spin = max_spin*0.35f + high_pass_spin*0.65f;
}

// Main feature extraction function
void extract_bowling_features(IMU_Buffer* buffer, BowlingTrajectory* trajectory) { 
  // Default to invalid throw
  trajectory->is_valid_throw = false;
  
  // Apply filtering to reduce noise
  apply_low_pass_filter(buffer, 0.8f);
  
  if (!validate_swing(buffer)) {
    // No valid throw detected
    return;
  } 
  
  // Calculate release speed
  calculate_speed(buffer, &trajectory->release_speed);

  // Calculate spin rate
  calculate_spin(buffer, &trajectory->spin_rate);
  
  // Mark as valid throw
  trajectory->is_valid_throw = true;
}
 
// Used for debugging -> prints info about trajectory
void print_trajectory_info(BowlingTrajectory traj) {
  if (!traj.is_valid_throw) {
    printf("No valid throw detected!\n");
    return;
  }
  
  printf("\nBowling Trajectory Analysis:\n");
  printf("---------------------------\n");
  printf("Initial speed: %.2f m/s\n", traj.release_speed);
  printf("Direction Angle: %.2f degrees (0 = straight)\n", traj.direction_angle * 180.0f / MY_PI);
  printf("Spin Rate: %.2f rad/s\n", traj.spin_rate);
  printf("Axis Position: %.3f \n", traj.y_pos);
}