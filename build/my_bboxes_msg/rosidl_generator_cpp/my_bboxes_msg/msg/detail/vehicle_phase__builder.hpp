// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_bboxes_msg:msg/VehiclePhase.idl
// generated code does not contain a copyright notice

#ifndef MY_BBOXES_MSG__MSG__DETAIL__VEHICLE_PHASE__BUILDER_HPP_
#define MY_BBOXES_MSG__MSG__DETAIL__VEHICLE_PHASE__BUILDER_HPP_

#include "my_bboxes_msg/msg/detail/vehicle_phase__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace my_bboxes_msg
{

namespace msg
{

namespace builder
{

class Init_VehiclePhase_phase
{
public:
  Init_VehiclePhase_phase()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_bboxes_msg::msg::VehiclePhase phase(::my_bboxes_msg::msg::VehiclePhase::_phase_type arg)
  {
    msg_.phase = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_bboxes_msg::msg::VehiclePhase msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_bboxes_msg::msg::VehiclePhase>()
{
  return my_bboxes_msg::msg::builder::Init_VehiclePhase_phase();
}

}  // namespace my_bboxes_msg

#endif  // MY_BBOXES_MSG__MSG__DETAIL__VEHICLE_PHASE__BUILDER_HPP_
