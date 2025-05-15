#include "imu_container.h"

#include <stdlib.h>

// Initialize the buffer with an initial capacity
bool IMU_Buffer_Init(IMU_Buffer* buffer, size_t initial_capacity) {
  buffer->data = (IMU_Data*)malloc(initial_capacity * sizeof(IMU_Data));
  if (buffer->data == NULL) {
      return false;  // Allocation failed
  }
  
  buffer->size = 0;
  buffer->capacity = initial_capacity;
  return true;
}

// Add a new IMU_Data to the IMU_buffer
bool IMU_Buffer_Add_Data(IMU_Buffer* buffer, IMU_Data new_data) {
  // Check if we need to resize
  if (buffer->size >= buffer->capacity) {
      // Double the capacity
      size_t new_capacity = buffer->capacity * 2;
      IMU_Data* new_data_ptr = (IMU_Data*)realloc(buffer->data, 
                                                 new_capacity * sizeof(IMU_Data));
      
      if (new_data_ptr == NULL) {
          return false;  // Reallocation failed
      }
      
      buffer->data = new_data_ptr;
      buffer->capacity = new_capacity;
  }
  
  // Add the new data
  buffer->data[buffer->size] = new_data;
  buffer->size++;
  
  return true;
}

bool IMU_Buffer_Clear(IMU_Buffer* buffer) {
  if (buffer == NULL) {
    return false;  // Invalid buffer pointer
  }
  
  // Reset size to 0 but keep the allocated memory
  buffer->size = 0;
  return true;
}

// Add a new IMU readings to the IMU_buffer
bool IMU_Buffer_Add(IMU_Buffer* buffer, float ax, float ay, float az, float gx, float gy, float gz, uint32_t timestamp) {
  IMU_Data new_data = {ax, ay, az, gx, gy, gz, timestamp};
  return IMU_Buffer_Add_Data(buffer, new_data);
}
