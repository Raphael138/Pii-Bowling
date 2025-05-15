#ifndef MFE_ALGORITHM_H
#define MFE_ALGORITHM_H

#include "imu_container.h"

 // Define trajectory output structure
 typedef struct {
  float release_speed;
  float direction_angle;
  float spin_rate;
  float y_pos;
  bool is_valid_throw;
} BowlingTrajectory;

void extract_bowling_features(IMU_Buffer* buffer, BowlingTrajectory* trajectory);
void print_trajectory_info(BowlingTrajectory traj);

#endif