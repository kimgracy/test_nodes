// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_bboxes_msg:msg/VehiclePhase.idl
// generated code does not contain a copyright notice

#ifndef MY_BBOXES_MSG__MSG__DETAIL__VEHICLE_PHASE__STRUCT_H_
#define MY_BBOXES_MSG__MSG__DETAIL__VEHICLE_PHASE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'phase'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/VehiclePhase in the package my_bboxes_msg.
typedef struct my_bboxes_msg__msg__VehiclePhase
{
  rosidl_runtime_c__String phase;
} my_bboxes_msg__msg__VehiclePhase;

// Struct for a sequence of my_bboxes_msg__msg__VehiclePhase.
typedef struct my_bboxes_msg__msg__VehiclePhase__Sequence
{
  my_bboxes_msg__msg__VehiclePhase * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_bboxes_msg__msg__VehiclePhase__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_BBOXES_MSG__MSG__DETAIL__VEHICLE_PHASE__STRUCT_H_
