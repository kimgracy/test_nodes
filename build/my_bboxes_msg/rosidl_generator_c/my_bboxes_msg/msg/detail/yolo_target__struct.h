// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_bboxes_msg:msg/YoloTarget.idl
// generated code does not contain a copyright notice

#ifndef MY_BBOXES_MSG__MSG__DETAIL__YOLO_TARGET__STRUCT_H_
#define MY_BBOXES_MSG__MSG__DETAIL__YOLO_TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.h"

// Struct defined in msg/YoloTarget in the package my_bboxes_msg.
typedef struct my_bboxes_msg__msg__YoloTarget
{
  sensor_msgs__msg__Image image;
} my_bboxes_msg__msg__YoloTarget;

// Struct for a sequence of my_bboxes_msg__msg__YoloTarget.
typedef struct my_bboxes_msg__msg__YoloTarget__Sequence
{
  my_bboxes_msg__msg__YoloTarget * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_bboxes_msg__msg__YoloTarget__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_BBOXES_MSG__MSG__DETAIL__YOLO_TARGET__STRUCT_H_
