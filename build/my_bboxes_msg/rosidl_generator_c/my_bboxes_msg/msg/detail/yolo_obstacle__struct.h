// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_bboxes_msg:msg/YoloObstacle.idl
// generated code does not contain a copyright notice

#ifndef MY_BBOXES_MSG__MSG__DETAIL__YOLO_OBSTACLE__STRUCT_H_
#define MY_BBOXES_MSG__MSG__DETAIL__YOLO_OBSTACLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'label'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/YoloObstacle in the package my_bboxes_msg.
typedef struct my_bboxes_msg__msg__YoloObstacle
{
  rosidl_runtime_c__String label;
  float x;
  float y;
} my_bboxes_msg__msg__YoloObstacle;

// Struct for a sequence of my_bboxes_msg__msg__YoloObstacle.
typedef struct my_bboxes_msg__msg__YoloObstacle__Sequence
{
  my_bboxes_msg__msg__YoloObstacle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_bboxes_msg__msg__YoloObstacle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_BBOXES_MSG__MSG__DETAIL__YOLO_OBSTACLE__STRUCT_H_
