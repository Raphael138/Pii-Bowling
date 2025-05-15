#include "bowling_score.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// Function to initialize a new bowling score
void initBowlingScore(BowlingScore* score, const char* name) {
    // Initialize all frames to zero
    for (int i = 0; i < 10; i++) {
        score->frames[i].first_roll = -1;      // -1 indicates not rolled yet
        score->frames[i].second_roll = -1;     // -1 indicates not rolled yet
        score->frames[i].third_roll = -1;      // -1 indicates not rolled yet
        score->frames[i].is_strike = false;
        score->frames[i].is_spare = false;
        score->frames[i].frame_score = 0;
        score->frames[i].running_total = 0;
    }
    
    // Initialize game state
    score->current_frame = 0;
    score->current_roll = 0;
    score->total_score = 0;
    score->game_complete = false;
    
    // Initialize player info
    strncpy(score->player_name, name, 19);
    score->player_name[19] = '\0';  // Ensure null termination
    
    // Initialize statistics
    score->strike_count = 0;
    score->spare_count = 0;
    score->gutter_count = 0;
    score->open_frame_count = 0;
    score->best_frame_score = 0;
    score->best_frame_index = 0;
}

// Function to record a roll in the current frame
void recordRoll(BowlingScore* score, int pins) {
    // Make sure game isn't already complete
    if (score->game_complete) {
        return;
    }
    
    // Get current frame
    BowlingFrame* frame = &score->frames[score->current_frame];
    
    // Record pins based on current roll
    if (score->current_roll == 0) {
        // First roll of the frame
        frame->first_roll = pins;
        
        // Check if it's a strike
        if (pins == 10) {
            frame->is_strike = true;
            score->strike_count++;
            
            // Move to next frame unless we're in the 10th
            if (score->current_frame < 9) {
                score->current_frame++;
            } else {
                score->current_roll = 1;  // Move to second roll in 10th frame
            }
        } else {
            if (pins == 0) {
                score->gutter_count++;
            }
            score->current_roll = 1;  // Move to second roll
        }
    } 
    else if (score->current_roll == 1) {
        // Second roll of the frame
        frame->second_roll = pins;
        
        // Check if it's a spare
        if (!frame->is_strike && frame->first_roll + pins == 10) {
            frame->is_spare = true;
            score->spare_count++;
        } 
        else if (frame->first_roll + pins < 10) {
            // Open frame
            score->open_frame_count++;
        }
        
        if (pins == 0) {
            score->gutter_count++;
        }
        
        // 10th frame special handling
        if (score->current_frame == 9) {
            if (frame->is_strike || frame->is_spare) {
                score->current_roll = 2;  // Move to third roll for bonus
            } else {
                score->game_complete = true;
            }
        } else {
            // Move to next frame
            score->current_frame++;
            score->current_roll = 0;
        }
    } 
    else if (score->current_roll == 2 && score->current_frame == 9) {
        // Third roll (only possible in 10th frame after strike or spare)
        frame->third_roll = pins;
        
        if (pins == 0) {
            score->gutter_count++;
        }
        
        // Game is now complete
        score->game_complete = true;
    }
    
    // Calculate updated scores
    calculateScore(score);
}

// Function to calculate the current score
void calculateScore(BowlingScore* score) {
    score->total_score = 0;
    
    for (int i = 0; i < 10; i++) {
        BowlingFrame* frame = &score->frames[i];
        int frame_score = 0;
        
        // Skip frames not yet rolled
        if (frame->first_roll == -1) {
            break;
        }
        
        if (frame->is_strike) {
            // Strike: 10 plus pins from next two rolls
            frame_score = 10;
            
            // Look ahead to next rolls
            if (i < 9) {
                // If next frame has first roll
                if (score->frames[i+1].first_roll != -1) {
                    frame_score += score->frames[i+1].first_roll;
                    
                    // Second roll after strike could be in next frame or two frames ahead
                    if (score->frames[i+1].is_strike) {
                        // If next frame is also a strike
                        if (i < 8 && score->frames[i+2].first_roll != -1) {
                            frame_score += score->frames[i+2].first_roll;
                        } else if (i == 8 && score->frames[i+1].second_roll != -1) {
                            // Special case for strike in 9th followed by strike in 10th
                            frame_score += score->frames[i+1].second_roll;
                        }
                    } else if (score->frames[i+1].second_roll != -1) {
                        // Second roll of next frame
                        frame_score += score->frames[i+1].second_roll;
                    }
                }
            } else {
                // 10th frame
                if (frame->second_roll != -1) {
                    frame_score += frame->second_roll;
                    if (frame->third_roll != -1) {
                        frame_score += frame->third_roll;
                    }
                }
            }
        } 
        else if (frame->is_spare) {
            // Spare: 10 plus pins from next roll
            frame_score = 10;
            
            // Look ahead to next roll
            if (i < 9) {
                if (score->frames[i+1].first_roll != -1) {
                    frame_score += score->frames[i+1].first_roll;
                }
            } else {
                // 10th frame
                if (frame->third_roll != -1) {
                    frame_score += frame->third_roll;
                }
            }
        } 
        else {
            // Open frame
            if (frame->first_roll != -1) {
                frame_score += frame->first_roll;
            }
            if (frame->second_roll != -1) {
                frame_score += frame->second_roll;
            }
        }
        
        // Update the frame score
        frame->frame_score = frame_score;
        
        // Update running total
        score->total_score += frame_score;
        frame->running_total = score->total_score;
        
        // Update best frame
        if (frame_score > score->best_frame_score) {
            score->best_frame_score = frame_score;
            score->best_frame_index = i;
        }
    }
}

// Function to reset the game
void resetBowlingScore(BowlingScore* score) {
    // Keep the player name but reset everything else
    char name[20];
    strncpy(name, score->player_name, 19);
    name[19] = '\0';
    
    // Initialize to defaults
    initBowlingScore(score, name);
}