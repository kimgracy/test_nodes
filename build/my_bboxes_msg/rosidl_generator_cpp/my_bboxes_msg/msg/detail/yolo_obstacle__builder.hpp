// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_bboxes_msg:msg/YoloObstacle.idl
// generated code does not contain a copyright notice

#ifndef MY_BBOXES_MSG__MSG__DETAIL__YOLO_OBSTACLE__BUILDER_HPP_
#define MY_BBOXES_MSG__MSG__DETAIL__YOLO_OBSTACLE__BUILDER_HPP_

#include "my_bboxes_msg/msg/detail/yolo_obstacle__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace my_bboxes_msg
{

namespace msg
{

namespace builder
{

class Init_YoloObstacle_y
{
public:
  explicit Init_YoloObstacle_y(::my_bboxes_msg::msg::YoloObstacle & msg)
  : msg_(msg)
  {}
  ::my_bboxes_msg::msg::YoloObstacle y(::my_bboxes_msg::msg::YoloObstacle::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_bboxes_msg::msg::YoloObstacle msg_;
};

class Init_YoloObstacle_x
{
public:
  explicit Init_YoloObstacle_x(::my_bboxes_msg::msg::YoloObstacle & msg)
  : msg_(msg)
  {}
  Init_YoloObstacle_y x(::my_bboxes_msg::msg::YoloObstacle::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_YoloObstacle_y(msg_);
  }

private:
  ::my_bboxes_msg::msg::YoloObstacle msg_;
};

class Init_YoloObstacle_label
{
public:
  Init_YoloObstacle_label()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_YoloObstacle_x label(::my_bboxes_msg::msg::YoloObstacle::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_YoloObstacle_x(msg_);
  }

private:
  ::my_bboxes_msg::msg::YoloObstacle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_bboxes_msg::msg::YoloObstacle>()
{
  return my_bboxes_msg::msg::builder::Init_YoloObstacle_label();
}

}  // namespace my_bboxes_msg

#endif  // MY_BBOXES_MSG__MSG__DETAIL__YOLO_OBSTACLE__BUILDER_HPP_
