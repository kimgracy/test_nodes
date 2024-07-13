// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_bboxes_msg:msg/YoloTarget.idl
// generated code does not contain a copyright notice

#ifndef MY_BBOXES_MSG__MSG__DETAIL__YOLO_TARGET__BUILDER_HPP_
#define MY_BBOXES_MSG__MSG__DETAIL__YOLO_TARGET__BUILDER_HPP_

#include "my_bboxes_msg/msg/detail/yolo_target__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace my_bboxes_msg
{

namespace msg
{

namespace builder
{

class Init_YoloTarget_image
{
public:
  Init_YoloTarget_image()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_bboxes_msg::msg::YoloTarget image(::my_bboxes_msg::msg::YoloTarget::_image_type arg)
  {
    msg_.image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_bboxes_msg::msg::YoloTarget msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_bboxes_msg::msg::YoloTarget>()
{
  return my_bboxes_msg::msg::builder::Init_YoloTarget_image();
}

}  // namespace my_bboxes_msg

#endif  // MY_BBOXES_MSG__MSG__DETAIL__YOLO_TARGET__BUILDER_HPP_
