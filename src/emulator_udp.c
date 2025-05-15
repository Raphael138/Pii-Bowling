
//
// This version of the game control code is set up to RECIEVE data through UDP
//
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hardware/uart.h"

// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"

// Include custom libraries
#include "vga16_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_3.h"

#include "mfe_algorithm.h"
#include "imu_container.h"
#include "bowling_physics.h"
#include "bowling_score.h"
#include "bowling_graphics.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stdio.h"

// Include udp libraries
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/opt.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/dns.h"
#include "lwip/netif.h"

#define DEG2RAD(angle) ((angle) * M_PI / 180.0)

// UDP constants
#define UDP_PORT 1234
#define udp_target_pico "10.49.244.190"

#define UDP_MSG_LEN_MAX 128

// VGA screen constants
#define _height 480
#define _width 640
#define _center_x 320
#define _center_y 240

// ms per frame
#define FRAME_RATE 33

// Game phase constant
volatile int game_phase = 0;
volatile float moving_traj_direction = 0;
volatile float moving_traj_dir_angle_increment = 0.05;
volatile float traj_dir_y_pos[] = {_center_y-2*_bowling_alley_width/6, _center_y-_bowling_alley_width/6, _center_y, _center_y+_bowling_alley_width/6, _center_y+2*_bowling_alley_width/6};
volatile int traj_dir_y_pos_idx = 2;
volatile int traj_dir_y_pos_idx_change = 1;

// Game play variables 
volatile int page; 
volatile int frame_score; 
volatile int game_player_idx = 0;

// Pin array
Pin game_pins[10];

// Instatiating the pins
void initPins() {
  int pin_idx = 0;
  int vertical_pin_distance = _pin_distance * 1.5;

  // For each row
  for (int i = 0; i < 5; i++) {
      int x = _width-115 + _pin_distance * i; // X position of the pin
      int y = _center_y - (i-1) * (vertical_pin_distance/2); // Y position of the pin

      // For each pin in the row 
      for (int j = 0; j < i; j++) {
          game_pins[pin_idx].pin_radius = _pin_radius;
          game_pins[pin_idx].pin_x = x;
          game_pins[pin_idx].pin_y = y;
          game_pins[pin_idx].fallen = false;
          
          pin_idx++; // Move to the next pin in the array
          y += vertical_pin_distance; // Move to the next pin in the row
      }
  }
}

// Updates the pin simulated status
void updatePinStatus(BowlingSimulation* bowling_simulation){
  for (int i = 0; i < bowling_simulation->pins.pin_count; i++)
    bowling_simulation->pins.pins[i].to_simulate = bowling_simulation->pins.pins[i].is_standing;
  
}

// RECV DATA
static struct udp_pcb *udp_rx_pcb;
fix15 received_data[UDP_MSG_LEN_MAX]; 
struct pt_sem new_message;
struct pbuf *p;

volatile bool new_trajectory = false;

// WIFI 
#define WIFI_SSID "RedRover"
#define WIFI_PASSWORD NULL

// Initialize IMU_buffer, BowlingTrajectory, etc...
IMU_Buffer imu_buffer;
BowlingTrajectory bowling_traj;
BowlingSimulation bowling_simulation;
BowlingScore gamescores[2]; 

void drawArrow(float angle, char COLOR, float arrow_base_y) {
    // Draw an arrow in the direction of the throw
    float x = 5 + cos(angle) * 40; // X position of the arrow tip
    float y = arrow_base_y + sin(angle) * 40; // Y position of the arrow tip

    drawLine(5, arrow_base_y, x, y, COLOR); // Draw the arrow line
    drawLine(x, y, x - cos(angle - 0.3) * 10, y - sin(angle - 0.3) * 10, COLOR); // Draw left side of arrowhead
    drawLine(x, y, x - cos(angle + 0.3) * 10, y - sin(angle + 0.3) * 10, COLOR); // Draw right side of arrowhead
}

void animateBowlingFigure(int centerX, int centerY) {
  int width = 250;
  int height = 120;

  int figureX = centerX - 80;
  int figureY = centerY;

  // Ball initial position closer to figure
  int ballX = figureX + 30;
  int ballY = figureY + 25;

  // Make frame
  drawRect(centerX - width / 2 - 2, centerY - height / 2 - 2, width+4, height+4, RED);
  drawRect(centerX - width / 2 - 1, centerY - height / 2 - 1, width+2, height+2, RED);
  fillCircle(figureX - 3, figureY - 2, 1, WHITE);
  fillCircle(figureX + 3, figureY - 2, 1, WHITE);
  drawVLine(figureX, figureY+1, 2, WHITE);
  drawCircleHelper(figureX, figureY + 3, 3, 0xC, WHITE);
  
  // Draw stick figure head
  drawCircle(figureX, figureY, 10, WHITE);

  // Draw rest of body
  drawVLine(figureX, figureY + 10, 30, WHITE); // body
  drawLine(figureX, figureY + 40, figureX - 10, figureY + 60, WHITE); // left leg
  drawLine(figureX, figureY + 40, figureX + 10, figureY + 60, WHITE); // right leg
  drawLine(figureX, centerY + 20, figureX - 9, centerY + 36, WHITE); // left arm leg

  // Draw pin
  drawRotatedPin(centerX+94, centerY+height/2-28, 0, false);

  // Calculate arm and ball positions along an arc
  float angle;
  float pin_angle=0;
  short armEndX, armEndY;
  short ballPosX, ballPosY;
  
  float cx = 4.5;
  float cy = 13.5;

  // Animate arm movement and ball leaving the hand
  for (int step = -30; step < 75; step++) {   
    if (step < 37) drawLine(figureX, figureY + 20, armEndX, armEndY, BLACK);
    if (step >= 56 && step < 65) {
      drawRotatedPin(centerX+94+8*sin(DEG2RAD(pin_angle)), centerY+height/2-28+8*sin(DEG2RAD(pin_angle)), pin_angle, true);
      drawHLine(centerX + 84, centerY + height / 2, 40, RED);
      drawHLine(centerX + 84, centerY + height / 2 + 1, 40, RED);    
    }

    fillCircle(ballPosX, ballPosY, 5, BLACK);
    drawVLine(figureX, figureY + 10, 30, WHITE); // body
    drawLine(figureX, figureY + 40, figureX - 10, figureY + 60, WHITE); // left leg
    drawLine(figureX, figureY + 40, figureX + 10, figureY + 60, WHITE); // right leg
    drawLine(figureX, centerY + 20, figureX - 9, centerY + 36, WHITE); // left arm leg
    
    if (step < 10) {
      // Backswing - arm moves backward and up in an arc
      angle = -0.8 - (step * 0.05);
      
      // Calculate arm endpoint using polar coordinates
      armEndX = figureX + (short)(20 * sin(angle));
      armEndY = figureY + 20 + (short)(20 * cos(angle));
      
      // Ball follows the arm
      ballPosX = armEndX;
      ballPosY = armEndY;
      
      // Draw arm and ball
      drawLine(figureX, figureY + 20, armEndX, armEndY, WHITE);
      fillCircle(ballPosX, ballPosY, 5, RED);
    } else {
      // Forward swing and release
      angle = -1.3 + ((step - 10) * 0.07);  // Convert step to radians for forward swing
      
      // Calculate arm endpoint
      armEndX = figureX + (short)(20 * sin(angle));
      armEndY = figureY + 20 + (short)(20 * cos(angle));
      
      // Draw arm
      if (step < 37) drawLine(figureX, figureY + 20, armEndX, armEndY, WHITE);
      
      if (step < 20) {
        // Ball still in hand
        ballPosX = armEndX;
        ballPosY = armEndY;
      } else {        
        // Time since release
        if (step < 56) {
          ballPosX += 5;
          ballPosY += 5;
          if (ballPosY + 6 > centerY + height/2) ballPosY = centerY + height/2 - 6;
        } else if (step < 65){
          // Pin falling
          pin_angle += 10.0;
          drawRotatedPin(centerX+94+8*sin(DEG2RAD(pin_angle)), centerY+height/2-28+8*sin(DEG2RAD(pin_angle)), pin_angle, false);
          drawHLine(centerX + 84, centerY + height / 2, 40, RED);
          drawHLine(centerX + 84, centerY + height / 2 + 1, 40, RED);    
        }
      }
      
      // Draw ball
      fillCircle(ballPosX, ballPosY, 5, RED);
      sleep_ms(30);
    }
    if (page != 0) return;
  }

  // Blank out last ball, pin position, and arm displays
  drawLine(figureX, figureY + 20, armEndX, armEndY, BLACK);
  fillCircle(ballPosX, ballPosY, 5, BLACK);
  drawRotatedPin(centerX+94+8*sin(DEG2RAD(pin_angle)), centerY+height/2-28+8*sin(DEG2RAD(pin_angle)), pin_angle, true);
  drawHLine(centerX + 84, centerY + height / 2, 40, RED);
  drawHLine(centerX + 84, centerY + height / 2 + 1, 40, RED);
}

void displayWelcomeScreen() {
  // Clear the screen to black
  fillRect(0, 0, _width, _height, BLACK);

  // Set up text for title
  setTextSize(3);  // Larger text for title
  setTextColor(WHITE);
  setCursor(_center_x-200, _height - 150);
  writeString("Welcome to P   Bowling");
  drawRotatedPin(338, 325, 0, false);
  drawRotatedPin(352, 325, 0, false);

  // Set up text for instructions
  setTextSize(2);  // Smaller text for instructions
  setCursor(_center_x - 160, _height - 100);
  writeString("Press any button to begin");
  
  int drawn_pins_specs[12][3] = {
    {50, 50, 10}, {600, 400, 40}, {600, 50, 60}, {80, 440, 10}, {10, 270, 270}, 
    {580, 250, 170}, {150, 235, 340}, {400, 245, 110}, {320, 20, 80}, {500, 30, 210},
    {320, 430, 65}, {200, 15, 65}
  };
  Vector2D drawn_balls_sepcs[5][2] = {
    {{50, 200}, {4, 2}}, {{630, 325}, {20, 50}}, {{610, 180}, {2,3}}, {{520 ,450}, {35, 41}}, {{280, 315}, {10, 0}}
  };

  for (int i = 0; i < 12; i++) drawRotatedPin(drawn_pins_specs[i][0], drawn_pins_specs[i][1], drawn_pins_specs[i][2], false);
  for (int i = 0; i < 5; i++) drawBall(drawn_balls_sepcs[i][0], drawn_balls_sepcs[i][1], RED);

  while (page==0)
    animateBowlingFigure(320,150); // TODO: Fix this
}

void displayWinner() {
  // Blank out screen
  fillRect(0, 0, _width, _height, BLACK);

  int winner = 1; 
  int player_one_score = gamescores[0].total_score; 
  int player_two_score = gamescores[1].total_score; 

  if (player_one_score == player_two_score){
    winner = 0; 
  } else if(player_one_score > player_one_score) {
    winner = 1; 
  } else {
    winner = 2; 
  }

  char buffer[100];

  // Winning screen
  setTextSize(3);
  setTextColor(WHITE);
  setCursor(_center_x-200, _height - 150);
  if (winner == 0 ){
    sprintf(buffer, "TIE!");
  }
  else {
    sprintf(buffer, "Winner is Player #%d", winner);
  }
  writeString(buffer);

  sleep_ms(30);
}

void displayScoreboard() {
  // Draw the outer frame of the scoreboard
  drawLine(10, 10, 610, 10, BLUE);
  drawLine(10, 10, 10, 130, BLUE);
 
  // Draw the inner rectangle of the scoreboard
  drawRect(20, 20, 600, 120, BLUE);
  
  // Connect outer frame to inner rectangle with diagonal lines
  drawLine(10, 10, 20, 20, BLUE);
  drawLine(20, 140, 10, 130, BLUE);
  drawLine(620, 20, 610, 10, BLUE);
 
  // Draw horizontal divider lines
  drawHLine(20, 40, 600, BLUE);
  drawHLine(20, 90, 600, BLUE);
 
  // Set up "Player" label
  setTextSize(1.5);  
  setTextColor(WHITE);
  setCursor(25, 27);
  writeString("Player");
 
  // Player 1 number
  setTextSize(3);  
  setTextColor(WHITE);
  setCursor(46, 55);
  writeString("1");
 
  // Player 2 number
  setTextSize(3);  
  setCursor(46, 105);
  writeString("2");
 
  // Vertical line separating player numbers from player names
  drawVLine(100, 20, 120, BLUE);
 
  // Draw 10 frame columns
  for (int i = 0; i < 10; i++){
    // Vertical line for left edge of each frame
    drawVLine(100+52*i, 20, 120, BLUE);
 
    // Player 1 frame subdivisions for first/second rolls
    drawVLine(100+52*i+17, 40, 17, BLUE);
    drawVLine(100+52*i+34, 40, 17, BLUE);
    drawHLine(100+52*i, 57, 34, BLUE);
    
    // Player 2 frame subdivisions for first/second rolls
    drawVLine(100+52*i+17, 90, 17, BLUE);
    drawVLine(100+52*i+34, 90, 17, BLUE);
    drawHLine(100+52*i, 107, 34, BLUE);
  }
 
  // Draw horizontal lines for the last frame (which can have 3 rolls)
  drawHLine(568, 57, 52, BLUE);
  drawHLine(568, 107, 52, BLUE);
 
  // Add frame numbers at the top of each column
  setTextSize(1.5);
  char buffer[100];
  for (int i = 0; i < 10; i++) {
    sprintf(buffer, "Frame %d", i+1);
    setCursor(100+52*i+2, 27);
    writeString(buffer);
  }
 }

void updateScore(int player_idx) {
  int current_roll = 0; 

  // Count how many pins were knocked down in this roll
  for (int i = 0; i < 10; i++) {
    if (game_pins[i].fallen == true){
      current_roll++; 
    }
  }

  // Store current frame and roll before they get updated
  int frame = gamescores[player_idx].current_frame; 
  int roll = gamescores[player_idx].current_roll; 

  // Record this roll and update game state
  recordRoll(&gamescores[player_idx], current_roll-frame_score);

  char buffer[100];  

  // Handle first roll display
  if(roll == 0){

    setTextSize(1); 
    setCursor(106 + frame * 52, 47 + 50 * player_idx);
    // Show X for strike, otherwise show pins knocked down
    if(gamescores[player_idx].frames[frame].is_strike) sprintf(buffer, "X");
    else sprintf(buffer, "%d", gamescores[player_idx].frames[frame].first_roll);
    writeString(buffer);

    // For strikes, immediately display running score total
    if (gamescores[player_idx].frames[frame].is_strike){
      setTextSize(2);
      sprintf(buffer, "%d", gamescores[player_idx].total_score);
      setCursor(121 + frame * 52, 67 + 50 * player_idx);
      writeString(buffer);
    }
  }
  // Handle second roll display
  else if(roll == 1){
    setTextSize(1); 
    setCursor(124 + frame * 52, 47 + 50 * player_idx);
    // Show appropriate symbol for spare or strike, or pins knocked down
    if(gamescores[player_idx].frames[frame].is_spare) sprintf(buffer, "/");
    else if((current_roll-frame_score)==10) sprintf(buffer, "X");
    else sprintf(buffer, "%d", gamescores[player_idx].frames[frame].second_roll);
    writeString(buffer);

    // Display total score except in certain final frame scenarios
    if (frame != 9 || (frame==9 && !(gamescores[player_idx].frames[frame].is_strike) && !(gamescores[player_idx].frames[frame].is_spare))){
      setTextSize(2);
      sprintf(buffer, "%d", gamescores[player_idx].total_score);
      setCursor(121 + frame * 52, 67 + 50 * player_idx);
      writeString(buffer);
    }
  }
  // Handle third roll (only possible in 10th frame)
  else if(roll == 2){
    setTextSize(1);
    // Show X for strike, otherwise show pins knocked down
    if((current_roll-frame_score)==10) sprintf(buffer, "X");
    else sprintf(buffer, "%d", gamescores[player_idx].frames[frame].third_roll);
    setCursor(140, 40 + 50 * player_idx);
    writeString(buffer);

    // Display final score
    setTextSize(2);
    sprintf(buffer, "%d", gamescores[player_idx].total_score);
    setCursor(121 + frame * 52, 67 + 50 * player_idx);
    writeString(buffer);
  }

  // Update running total of pins knocked down
  frame_score = frame_score + current_roll; 
}

void playScreen() {
  int game_counter;

  initBowlingScore(&gamescores[0], "player_one");
  initBowlingScore(&gamescores[1], "player_two");

  // Blank out screen, display pins/scoreboard
  fillRect(0, 0, _width, _height, BLACK);
  drawBackground();
  displayScoreboard();
  for (int i = 0; i <10; i++) drawPin(i, game_pins, &bowling_simulation); // Draw the pins

  while (true) {
    uint64_t main_while_time_us = time_us_64();

    drawGutters();

    // Draw arrow in proper direction and location depending on game phase
    if (game_phase==0) {
      if (game_counter % 12 == 0) {
        drawArrow(moving_traj_direction, BLACK, traj_dir_y_pos[traj_dir_y_pos_idx]);
        traj_dir_y_pos_idx+= traj_dir_y_pos_idx_change;
        if (traj_dir_y_pos_idx == 4 || traj_dir_y_pos_idx == 0)
          traj_dir_y_pos_idx_change = -traj_dir_y_pos_idx_change; // Change direction of y position index
        drawArrow(moving_traj_direction, GREEN, traj_dir_y_pos[traj_dir_y_pos_idx]);
      }
    }
    else if (game_phase == 1) {
      drawArrow(moving_traj_direction, BLACK, bowling_traj.y_pos);
      moving_traj_direction = moving_traj_direction + moving_traj_dir_angle_increment;
      if (moving_traj_direction > M_PI/2 || moving_traj_direction < -M_PI/2)
        moving_traj_dir_angle_increment = -moving_traj_dir_angle_increment;
      drawArrow(moving_traj_direction, GREEN, bowling_traj.y_pos);
    } else if (game_phase == 3) {
      drawArrow(bowling_traj.direction_angle, bowling_traj.y_pos, GREEN);
    }
  
    if (new_trajectory) {    
      printf("New trajectory detected\n"); 

      // Extract bowling features
      extract_bowling_features(&imu_buffer, &bowling_traj);
      print_trajectory_info(bowling_traj);

      initialize_simulation(&bowling_simulation, &bowling_traj, game_pins); 
      
      // Run physics engine simulation
      while (!bowling_simulation.simulation_complete) {
        uint64_t time_us = time_us_64();
        
        // Erase the ball and redraw the gutters
        drawBall(bowling_simulation.ball.position, bowling_simulation.ball.velocity, BLACK);
        drawGutters();

        bowling_physics_update(&bowling_simulation, game_pins); // Step through the simulation
        
        // Draw the new ball
        drawBall(bowling_simulation.ball.position, bowling_simulation.ball.velocity, RED);

        // Update and draw pins
        for (int i = 0; i < 10; i++) {
            if (game_pins[i].fallen==bowling_simulation.pins.pins[i].is_standing) game_pins[i].fallen = true;
            if (bowling_simulation.pins.pins[i].to_simulate)
              drawPin(i, game_pins, &bowling_simulation);
        }
                
        int time_to_sleep = FRAME_RATE - (time_us_64() - time_us)/1000; // Calculate time to sleep in ms

        if (time_to_sleep > 0) sleep_ms(time_to_sleep); // Sleep for the calculated time
        else printf("Failed to meet time deadline with time_to_sleep=%d\n", time_to_sleep);
      }

      //Update scoreboard with new throw value (needs to be parameterized for players)
      updateScore(game_player_idx); 
      
      if (gamescores[game_player_idx].current_roll==0){ 
        game_player_idx = (game_player_idx + 1) % 2;
        frame_score = 0; 
      } 
      if (gamescores[game_player_idx].current_roll==0 || (frame_score==10 && gamescores[game_player_idx].current_frame==9)) {
        initPins();
      }
      updatePinStatus(&bowling_simulation); // Ensure that pins which do not need to be simulated are not simulated anymore
      initialize_simulation(&bowling_simulation, &bowling_traj, game_pins);

      // Reset to phase 0
      IMU_Buffer_Clear(&imu_buffer);
      new_trajectory=false;
      game_phase = 0; // Reset game phase
      moving_traj_direction = 0; // Reset moving direction
      traj_dir_y_pos_idx = 2; // Reset y position index
  
      fillRect(0, 141, _width, _bowling_alley_width + 40, BLACK); // Clear the screen
      for (int i = 0; i <10; i++) 
        if (bowling_simulation.pins.pins[i].to_simulate)
          drawPin(i, game_pins, &bowling_simulation); // Draw the pins

      if (gamescores[1].game_complete && gamescores[0].game_complete) {
        page = 2;
        return;
      }
    }

    int time_to_sleep = FRAME_RATE - (time_us_64() - main_while_time_us)/1000; // Calculate time to sleep in ms

    if (time_to_sleep > 0) sleep_ms(time_to_sleep); // Sleep for the calculated time
    else printf("Failed to meet time deadline with time_to_sleep=%d\n", time_to_sleep);

    game_counter++;
  }
}

// Thread that draws to VGA displays
static PT_THREAD (protothread_vga(struct pt *pt))
{
    PT_BEGIN(pt) ;

    while (true) {
      if (page==0)
        displayWelcomeScreen();
      if (page==1)
        playScreen();
      if (page==2)
        displayWinner();
    }

    PT_END(pt);
}

// Entry point for core 1 (VGA display)
void core1_entry() {
  pt_add_thread(protothread_vga) ;
  pt_schedule_start ;
}

//==================================================
// UDP async receive callback setup
// NOTE that udpecho_raw_recv is triggered by a signal
// directly from the LWIP package -- not from your code
// this callback juswt copies out the packet string
// and sets a "new data" semaphore
// This runs in an ISR -- KEEP IT SHORT!!!

#if LWIP_UDP

static void udpecho_raw_recv(void *arg, struct udp_pcb *upcb, struct pbuf *p,
  const ip_addr_t *addr, u16_t port){
  LWIP_UNUSED_ARG(arg);
  // printf("indifr udp callback\n");

  // Check that there's something in the pbuf
  if (p != NULL) {
    // Copy the contents of the payload
    memcpy(received_data, p->payload, UDP_MSG_LEN_MAX) ;
    // Semaphore-signal a thread
    // printf("semophore triggered\n");
    PT_SEM_SAFE_SIGNAL(pt, &new_message) ;
    // Reset the payload buffer
    memset(p->payload, 0, UDP_MSG_LEN_MAX+1);
    // Free the PBUF
    pbuf_free(p);
  }
  else printf("NULL pt in callback");
}

//recv callback 
void udpecho_raw_init(void){

  // Initialize the RX protocol control block
  udp_rx_pcb  = udp_new_ip_type(IPADDR_TYPE_ANY);
  p = pbuf_alloc(PBUF_TRANSPORT, UDP_MSG_LEN_MAX, PBUF_RAM);

  // static ip_addr_t addr;
  // ipaddr_aton(udp_target_pico, &addr);

  // Make certain that the pcb has initialized, else print a message
  if (udp_rx_pcb != NULL) {
    // printf("pcb intitialized\n");
    // Err_t object for error codes
    err_t err;
    // Bind this PCB to our assigned IP, and our chosen port. Received messages
    // to this port will be directed to our bound protocol control block.
    err = udp_bind(udp_rx_pcb, netif_ip4_addr(netif_list), UDP_PORT);
    // Check that the bind was successful, else print a message
    if (err == ERR_OK) {
      // Setup the receive callback function
      udp_recv(udp_rx_pcb, udpecho_raw_recv, NULL);
      printf("setup cb\n");
    } else {
      printf("bind error");
    }
  } else {
    printf("udp_rx_pcb error");
  }
}

#endif /* LWIP_UDP */

static PT_THREAD (protothread_receive(struct pt *pt))
{
  // Begin thread
  PT_BEGIN(pt) ;
  int last_time_stamp = -1;

  while(1) {
    // Wait on a semaphore
    PT_SEM_SAFE_WAIT(pt, &new_message) ;

    float ax = fix2float15(received_data[0]);
    float ay = fix2float15(received_data[1]);
    float az = fix2float15(received_data[2]);
    float gx = fix2float15(received_data[3]);
    float gy = fix2float15(received_data[4]);
    float gz = fix2float15(received_data[5]);
    int time_stamp = fix2int15(received_data[6]);

    // Print qreceived message
    if (time_stamp != last_time_stamp) {
      // End of the swing (button 1)
      if (time_stamp == -1) {
        if (page == 0 || page == 2) page = (page+1) % 3; 
        else if (game_phase == 2) {
          printf("End of data\n");
          new_trajectory = true;
        }
      } 
      // Secondary button press 
      else if (fabs(ax+2)<0.001 && fabs(ay+2)<0.001 && fabs(az+2)<0.001) {
        if (page == 0 || page == 2 ) page = (page+1) % 3; 
        else if (game_phase == 0 && page == 1) {
          printf("Moving y-position: %f\n", traj_dir_y_pos[traj_dir_y_pos_idx]);
          game_phase = 1;
          bowling_traj.y_pos = traj_dir_y_pos[traj_dir_y_pos_idx];
        } else if (game_phase == 1  && page == 1) {
          printf("Moving direction angle set to: %f\n", moving_traj_direction);
          game_phase = 2;
          bowling_traj.direction_angle = moving_traj_direction;
        }
      } 
      // SWING BUTTON
      else {
        if (game_phase == 2) {
          IMU_Buffer_Add(&imu_buffer, ax, ay, az, gx, gy, gz, (uint32_t) time_stamp);
          printf("Data: %f %f %f2. %f %f %f %d\n", ax, ay, az, gx, gy, gz, time_stamp);
        }
      }
    }
    last_time_stamp = time_stamp;
  }

  // End thread
  PT_END(pt) ;
}

// Main (runs on core 0)
int main() {
    // Initialize stdio
    stdio_init_all();
    printf("Running emulator_udp\n");

    // Initialize VGA
    initVGA() ;

    ////////////////////////////////////////////////////////////////////////
    /////////////////////// Init CYW43 architecture ////////////////////////
    ////////////////////////////////////////////////////////////////////////
    if (cyw43_arch_init()) {
      printf("failed to initialise\n");
      return 1;
    }

    cyw43_arch_enable_sta_mode();

    initPins();
    if (!IMU_Buffer_Init(&imu_buffer, 1000)) {
        printf("Failed to initialize buffer\n");
        return 1;
    }

    ////////////////////////////////////////////////////////////////////////
    /////////////////////// Init UDP and Wifi Setup ////////////////////////
    ////////////////////////////////////////////////////////////////////////
    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
        printf("IP Address: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    }
    
    PT_SEM_INIT(&new_message, 0) ;

    //============================
    // UDP recenve ISR routines
    //============================
    udpecho_raw_init();

    //========================================
    // start core 1 with the VGA thread
    //========================================
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    pt_add_thread(protothread_receive);
    pt_schedule_start ;
}