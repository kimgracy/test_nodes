// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_bboxes_msg:msg/YoloTarget.idl
// generated code does not contain a copyright notice
#include "my_bboxes_msg/msg/detail/yolo_target__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `image`
#include "sensor_msgs/msg/detail/image__functions.h"

bool
my_bboxes_msg__msg__YoloTarget__init(my_bboxes_msg__msg__YoloTarget * msg)
{
  if (!msg) {
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__init(&msg->image)) {
    my_bboxes_msg__msg__YoloTarget__fini(msg);
    return false;
  }
  return true;
}

void
my_bboxes_msg__msg__YoloTarget__fini(my_bboxes_msg__msg__YoloTarget * msg)
{
  if (!msg) {
    return;
  }
  // image
  sensor_msgs__msg__Image__fini(&msg->image);
}

bool
my_bboxes_msg__msg__YoloTarget__are_equal(const my_bboxes_msg__msg__YoloTarget * lhs, const my_bboxes_msg__msg__YoloTarget * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->image), &(rhs->image)))
  {
    return false;
  }
  return true;
}

bool
my_bboxes_msg__msg__YoloTarget__copy(
  const my_bboxes_msg__msg__YoloTarget * input,
  my_bboxes_msg__msg__YoloTarget * output)
{
  if (!input || !output) {
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__copy(
      &(input->image), &(output->image)))
  {
    return false;
  }
  return true;
}

my_bboxes_msg__msg__YoloTarget *
my_bboxes_msg__msg__YoloTarget__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_bboxes_msg__msg__YoloTarget * msg = (my_bboxes_msg__msg__YoloTarget *)allocator.allocate(sizeof(my_bboxes_msg__msg__YoloTarget), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_bboxes_msg__msg__YoloTarget));
  bool success = my_bboxes_msg__msg__YoloTarget__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_bboxes_msg__msg__YoloTarget__destroy(my_bboxes_msg__msg__YoloTarget * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_bboxes_msg__msg__YoloTarget__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_bboxes_msg__msg__YoloTarget__Sequence__init(my_bboxes_msg__msg__YoloTarget__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_bboxes_msg__msg__YoloTarget * data = NULL;

  if (size) {
    data = (my_bboxes_msg__msg__YoloTarget *)allocator.zero_allocate(size, sizeof(my_bboxes_msg__msg__YoloTarget), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_bboxes_msg__msg__YoloTarget__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_bboxes_msg__msg__YoloTarget__fini(&data[i - 1]);
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
my_bboxes_msg__msg__YoloTarget__Sequence__fini(my_bboxes_msg__msg__YoloTarget__Sequence * array)
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
      my_bboxes_msg__msg__YoloTarget__fini(&array->data[i]);
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

my_bboxes_msg__msg__YoloTarget__Sequence *
my_bboxes_msg__msg__YoloTarget__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_bboxes_msg__msg__YoloTarget__Sequence * array = (my_bboxes_msg__msg__YoloTarget__Sequence *)allocator.allocate(sizeof(my_bboxes_msg__msg__YoloTarget__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_bboxes_msg__msg__YoloTarget__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_bboxes_msg__msg__YoloTarget__Sequence__destroy(my_bboxes_msg__msg__YoloTarget__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_bboxes_msg__msg__YoloTarget__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_bboxes_msg__msg__YoloTarget__Sequence__are_equal(const my_bboxes_msg__msg__YoloTarget__Sequence * lhs, const my_bboxes_msg__msg__YoloTarget__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_bboxes_msg__msg__YoloTarget__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_bboxes_msg__msg__YoloTarget__Sequence__copy(
  const my_bboxes_msg__msg__YoloTarget__Sequence * input,
  my_bboxes_msg__msg__YoloTarget__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_bboxes_msg__msg__YoloTarget);
    my_bboxes_msg__msg__YoloTarget * data =
      (my_bboxes_msg__msg__YoloTarget *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_bboxes_msg__msg__YoloTarget__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          my_bboxes_msg__msg__YoloTarget__fini(&data[i]);
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
    if (!my_bboxes_msg__msg__YoloTarget__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
