#ifndef BOWLING_SCORE_H
#define BOWLING_SCORE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// Struct to represent a single bowling frame
typedef struct {
    int first_roll;         // Pins knocked down on first roll
    int second_roll;        // Pins knocked down on second roll
    int third_roll;         // Only used in 10th frame if first roll is strike or spare
    bool is_strike;         // True if first roll knocked down all 10 pins
    bool is_spare;          // True if first + second roll equals 10
    int frame_score;        // Score for this individual frame
    int running_total;      // Running total including this frame
} BowlingFrame;

// Struct to represent a complete bowling game
typedef struct {
    BowlingFrame frames[10];    // 10 frames in a standard bowling game
    int current_frame;          // Current frame (0-9)
    int current_roll;           // Current roll within the frame (0 for first, 1 for second, 2 for third in final frame)
    int total_score;            // Total score for the game
    bool game_complete;         // Flag indicating if the game is complete
    
    // Player information
    char player_name[20];       // Player name (optional)
    
    // Game statistics - these can be useful for display or analysis
    int strike_count;           // Total number of strikes
    int spare_count;            // Total number of spares
    int gutter_count;           // Total number of gutter balls (0 pins)
    int open_frame_count;       // Frames that are neither strikes nor spares
    
    // Best stats
    int best_frame_score;       // Highest single frame score
    int best_frame_index;       // Index of the frame with highest score
} BowlingScore;

//define functions 
void initBowlingScore(BowlingScore* score, const char* name);
void recordRoll(BowlingScore* score, int pins);
void calculateScore(BowlingScore* score);
void resetBowlingScore(BowlingScore* score);

#endif