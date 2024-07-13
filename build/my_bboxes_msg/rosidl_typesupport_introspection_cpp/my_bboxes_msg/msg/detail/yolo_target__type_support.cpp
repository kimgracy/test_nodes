// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from my_bboxes_msg:msg/YoloTarget.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "my_bboxes_msg/msg/detail/yolo_target__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace my_bboxes_msg
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void YoloTarget_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) my_bboxes_msg::msg::YoloTarget(_init);
}

void YoloTarget_fini_function(void * message_memory)
{
  auto typed_message = static_cast<my_bboxes_msg::msg::YoloTarget *>(message_memory);
  typed_message->~YoloTarget();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember YoloTarget_message_member_array[1] = {
  {
    "image",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::Image>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_bboxes_msg::msg::YoloTarget, image),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers YoloTarget_message_members = {
  "my_bboxes_msg::msg",  // message namespace
  "YoloTarget",  // message name
  1,  // number of fields
  sizeof(my_bboxes_msg::msg::YoloTarget),
  YoloTarget_message_member_array,  // message members
  YoloTarget_init_function,  // function to initialize message memory (memory has to be allocated)
  YoloTarget_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t YoloTarget_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &YoloTarget_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace my_bboxes_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<my_bboxes_msg::msg::YoloTarget>()
{
  return &::my_bboxes_msg::msg::rosidl_typesupport_introspection_cpp::YoloTarget_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, my_bboxes_msg, msg, YoloTarget)() {
  return &::my_bboxes_msg::msg::rosidl_typesupport_introspection_cpp::YoloTarget_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
