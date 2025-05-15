#ifndef IMU_CONTAINER_H
#define IMU_CONTAINER_H

#include <stdio.h>
#include "pico/stdlib.h"

// Structs to store IMU data
typedef struct {
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t timestamp;
} IMU_Data;

typedef struct {
  IMU_Data* data;
  size_t size;
  size_t capacity;
} IMU_Buffer;

// Functions to add to buffers
bool IMU_Buffer_Init(IMU_Buffer* buffer, size_t initial_capacity);
bool IMU_Buffer_Add(IMU_Buffer* buffer, float ax, float ay, float az, float gx, float gy, float gz, uint32_t timestamp);
bool IMU_Buffer_Clear(IMU_Buffer* buffer);

#endif