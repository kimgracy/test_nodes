// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_bboxes_msg:msg/YoloObstacle.idl
// generated code does not contain a copyright notice

#ifndef MY_BBOXES_MSG__MSG__DETAIL__YOLO_OBSTACLE__TRAITS_HPP_
#define MY_BBOXES_MSG__MSG__DETAIL__YOLO_OBSTACLE__TRAITS_HPP_

#include "my_bboxes_msg/msg/detail/yolo_obstacle__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<my_bboxes_msg::msg::YoloObstacle>()
{
  return "my_bboxes_msg::msg::YoloObstacle";
}

template<>
inline const char * name<my_bboxes_msg::msg::YoloObstacle>()
{
  return "my_bboxes_msg/msg/YoloObstacle";
}

template<>
struct has_fixed_size<my_bboxes_msg::msg::YoloObstacle>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<my_bboxes_msg::msg::YoloObstacle>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<my_bboxes_msg::msg::YoloObstacle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_BBOXES_MSG__MSG__DETAIL__YOLO_OBSTACLE__TRAITS_HPP_
