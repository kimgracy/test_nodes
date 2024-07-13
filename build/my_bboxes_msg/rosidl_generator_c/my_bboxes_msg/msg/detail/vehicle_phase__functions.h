// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from my_bboxes_msg:msg/VehiclePhase.idl
// generated code does not contain a copyright notice

#ifndef MY_BBOXES_MSG__MSG__DETAIL__VEHICLE_PHASE__FUNCTIONS_H_
#define MY_BBOXES_MSG__MSG__DETAIL__VEHICLE_PHASE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "my_bboxes_msg/msg/rosidl_generator_c__visibility_control.h"

#include "my_bboxes_msg/msg/detail/vehicle_phase__struct.h"

/// Initialize msg/VehiclePhase message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * my_bboxes_msg__msg__VehiclePhase
 * )) before or use
 * my_bboxes_msg__msg__VehiclePhase__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_my_bboxes_msg
bool
my_bboxes_msg__msg__VehiclePhase__init(my_bboxes_msg__msg__VehiclePhase * msg);

/// Finalize msg/VehiclePhase message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_bboxes_msg
void
my_bboxes_msg__msg__VehiclePhase__fini(my_bboxes_msg__msg__VehiclePhase * msg);

/// Create msg/VehiclePhase message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * my_bboxes_msg__msg__VehiclePhase__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_my_bboxes_msg
my_bboxes_msg__msg__VehiclePhase *
my_bboxes_msg__msg__VehiclePhase__create();

/// Destroy msg/VehiclePhase message.
/**
 * It calls
 * my_bboxes_msg__msg__VehiclePhase__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_bboxes_msg
void
my_bboxes_msg__msg__VehiclePhase__destroy(my_bboxes_msg__msg__VehiclePhase * msg);

/// Check for msg/VehiclePhase message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_bboxes_msg
bool
my_bboxes_msg__msg__VehiclePhase__are_equal(const my_bboxes_msg__msg__VehiclePhase * lhs, const my_bboxes_msg__msg__VehiclePhase * rhs);

/// Copy a msg/VehiclePhase message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_bboxes_msg
bool
my_bboxes_msg__msg__VehiclePhase__copy(
  const my_bboxes_msg__msg__VehiclePhase * input,
  my_bboxes_msg__msg__VehiclePhase * output);

/// Initialize array of msg/VehiclePhase messages.
/**
 * It allocates the memory for the number of elements and calls
 * my_bboxes_msg__msg__VehiclePhase__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_bboxes_msg
bool
my_bboxes_msg__msg__VehiclePhase__Sequence__init(my_bboxes_msg__msg__VehiclePhase__Sequence * array, size_t size);

/// Finalize array of msg/VehiclePhase messages.
/**
 * It calls
 * my_bboxes_msg__msg__VehiclePhase__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_bboxes_msg
void
my_bboxes_msg__msg__VehiclePhase__Sequence__fini(my_bboxes_msg__msg__VehiclePhase__Sequence * array);

/// Create array of msg/VehiclePhase messages.
/**
 * It allocates the memory for the array and calls
 * my_bboxes_msg__msg__VehiclePhase__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_my_bboxes_msg
my_bboxes_msg__msg__VehiclePhase__Sequence *
my_bboxes_msg__msg__VehiclePhase__Sequence__create(size_t size);

/// Destroy array of msg/VehiclePhase messages.
/**
 * It calls
 * my_bboxes_msg__msg__VehiclePhase__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_bboxes_msg
void
my_bboxes_msg__msg__VehiclePhase__Sequence__destroy(my_bboxes_msg__msg__VehiclePhase__Sequence * array);

/// Check for msg/VehiclePhase message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_bboxes_msg
bool
my_bboxes_msg__msg__VehiclePhase__Sequence__are_equal(const my_bboxes_msg__msg__VehiclePhase__Sequence * lhs, const my_bboxes_msg__msg__VehiclePhase__Sequence * rhs);

/// Copy an array of msg/VehiclePhase messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_bboxes_msg
bool
my_bboxes_msg__msg__VehiclePhase__Sequence__copy(
  const my_bboxes_msg__msg__VehiclePhase__Sequence * input,
  my_bboxes_msg__msg__VehiclePhase__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MY_BBOXES_MSG__MSG__DETAIL__VEHICLE_PHASE__FUNCTIONS_H_
