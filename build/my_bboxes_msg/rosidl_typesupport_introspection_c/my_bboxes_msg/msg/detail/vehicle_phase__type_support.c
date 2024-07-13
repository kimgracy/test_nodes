// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from my_bboxes_msg:msg/VehiclePhase.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "my_bboxes_msg/msg/detail/vehicle_phase__rosidl_typesupport_introspection_c.h"
#include "my_bboxes_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "my_bboxes_msg/msg/detail/vehicle_phase__functions.h"
#include "my_bboxes_msg/msg/detail/vehicle_phase__struct.h"


// Include directives for member types
// Member `phase`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void VehiclePhase__rosidl_typesupport_introspection_c__VehiclePhase_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  my_bboxes_msg__msg__VehiclePhase__init(message_memory);
}

void VehiclePhase__rosidl_typesupport_introspection_c__VehiclePhase_fini_function(void * message_memory)
{
  my_bboxes_msg__msg__VehiclePhase__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember VehiclePhase__rosidl_typesupport_introspection_c__VehiclePhase_message_member_array[1] = {
  {
    "phase",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_bboxes_msg__msg__VehiclePhase, phase),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers VehiclePhase__rosidl_typesupport_introspection_c__VehiclePhase_message_members = {
  "my_bboxes_msg__msg",  // message namespace
  "VehiclePhase",  // message name
  1,  // number of fields
  sizeof(my_bboxes_msg__msg__VehiclePhase),
  VehiclePhase__rosidl_typesupport_introspection_c__VehiclePhase_message_member_array,  // message members
  VehiclePhase__rosidl_typesupport_introspection_c__VehiclePhase_init_function,  // function to initialize message memory (memory has to be allocated)
  VehiclePhase__rosidl_typesupport_introspection_c__VehiclePhase_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t VehiclePhase__rosidl_typesupport_introspection_c__VehiclePhase_message_type_support_handle = {
  0,
  &VehiclePhase__rosidl_typesupport_introspection_c__VehiclePhase_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_my_bboxes_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_bboxes_msg, msg, VehiclePhase)() {
  if (!VehiclePhase__rosidl_typesupport_introspection_c__VehiclePhase_message_type_support_handle.typesupport_identifier) {
    VehiclePhase__rosidl_typesupport_introspection_c__VehiclePhase_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &VehiclePhase__rosidl_typesupport_introspection_c__VehiclePhase_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
