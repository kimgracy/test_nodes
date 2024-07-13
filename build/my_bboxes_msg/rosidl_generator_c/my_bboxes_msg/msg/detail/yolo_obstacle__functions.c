// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_bboxes_msg:msg/YoloObstacle.idl
// generated code does not contain a copyright notice
#include "my_bboxes_msg/msg/detail/yolo_obstacle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `label`
#include "rosidl_runtime_c/string_functions.h"

bool
my_bboxes_msg__msg__YoloObstacle__init(my_bboxes_msg__msg__YoloObstacle * msg)
{
  if (!msg) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__init(&msg->label)) {
    my_bboxes_msg__msg__YoloObstacle__fini(msg);
    return false;
  }
  // x
  // y
  return true;
}

void
my_bboxes_msg__msg__YoloObstacle__fini(my_bboxes_msg__msg__YoloObstacle * msg)
{
  if (!msg) {
    return;
  }
  // label
  rosidl_runtime_c__String__fini(&msg->label);
  // x
  // y
}

bool
my_bboxes_msg__msg__YoloObstacle__are_equal(const my_bboxes_msg__msg__YoloObstacle * lhs, const my_bboxes_msg__msg__YoloObstacle * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->label), &(rhs->label)))
  {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  return true;
}

bool
my_bboxes_msg__msg__YoloObstacle__copy(
  const my_bboxes_msg__msg__YoloObstacle * input,
  my_bboxes_msg__msg__YoloObstacle * output)
{
  if (!input || !output) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__copy(
      &(input->label), &(output->label)))
  {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  return true;
}

my_bboxes_msg__msg__YoloObstacle *
my_bboxes_msg__msg__YoloObstacle__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_bboxes_msg__msg__YoloObstacle * msg = (my_bboxes_msg__msg__YoloObstacle *)allocator.allocate(sizeof(my_bboxes_msg__msg__YoloObstacle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_bboxes_msg__msg__YoloObstacle));
  bool success = my_bboxes_msg__msg__YoloObstacle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_bboxes_msg__msg__YoloObstacle__destroy(my_bboxes_msg__msg__YoloObstacle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_bboxes_msg__msg__YoloObstacle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_bboxes_msg__msg__YoloObstacle__Sequence__init(my_bboxes_msg__msg__YoloObstacle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_bboxes_msg__msg__YoloObstacle * data = NULL;

  if (size) {
    data = (my_bboxes_msg__msg__YoloObstacle *)allocator.zero_allocate(size, sizeof(my_bboxes_msg__msg__YoloObstacle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_bboxes_msg__msg__YoloObstacle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_bboxes_msg__msg__YoloObstacle__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
my_bboxes_msg__msg__YoloObstacle__Sequence__fini(my_bboxes_msg__msg__YoloObstacle__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      my_bboxes_msg__msg__YoloObstacle__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

my_bboxes_msg__msg__YoloObstacle__Sequence *
my_bboxes_msg__msg__YoloObstacle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_bboxes_msg__msg__YoloObstacle__Sequence * array = (my_bboxes_msg__msg__YoloObstacle__Sequence *)allocator.allocate(sizeof(my_bboxes_msg__msg__YoloObstacle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_bboxes_msg__msg__YoloObstacle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_bboxes_msg__msg__YoloObstacle__Sequence__destroy(my_bboxes_msg__msg__YoloObstacle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_bboxes_msg__msg__YoloObstacle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_bboxes_msg__msg__YoloObstacle__Sequence__are_equal(const my_bboxes_msg__msg__YoloObstacle__Sequence * lhs, const my_bboxes_msg__msg__YoloObstacle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_bboxes_msg__msg__YoloObstacle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_bboxes_msg__msg__YoloObstacle__Sequence__copy(
  const my_bboxes_msg__msg__YoloObstacle__Sequence * input,
  my_bboxes_msg__msg__YoloObstacle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_bboxes_msg__msg__YoloObstacle);
    my_bboxes_msg__msg__YoloObstacle * data =
      (my_bboxes_msg__msg__YoloObstacle *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_bboxes_msg__msg__YoloObstacle__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          my_bboxes_msg__msg__YoloObstacle__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_bboxes_msg__msg__YoloObstacle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
