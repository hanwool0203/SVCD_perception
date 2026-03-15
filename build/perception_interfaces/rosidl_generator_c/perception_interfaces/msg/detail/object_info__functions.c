// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from perception_interfaces:msg/ObjectInfo.idl
// generated code does not contain a copyright notice
#include "perception_interfaces/msg/detail/object_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `dimensions`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
perception_interfaces__msg__ObjectInfo__init(perception_interfaces__msg__ObjectInfo * msg)
{
  if (!msg) {
    return false;
  }
  // class_id
  // score
  // velocity_x
  // velocity_y
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    perception_interfaces__msg__ObjectInfo__fini(msg);
    return false;
  }
  // dimensions
  if (!geometry_msgs__msg__Vector3__init(&msg->dimensions)) {
    perception_interfaces__msg__ObjectInfo__fini(msg);
    return false;
  }
  return true;
}

void
perception_interfaces__msg__ObjectInfo__fini(perception_interfaces__msg__ObjectInfo * msg)
{
  if (!msg) {
    return;
  }
  // class_id
  // score
  // velocity_x
  // velocity_y
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // dimensions
  geometry_msgs__msg__Vector3__fini(&msg->dimensions);
}

bool
perception_interfaces__msg__ObjectInfo__are_equal(const perception_interfaces__msg__ObjectInfo * lhs, const perception_interfaces__msg__ObjectInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // class_id
  if (lhs->class_id != rhs->class_id) {
    return false;
  }
  // score
  if (lhs->score != rhs->score) {
    return false;
  }
  // velocity_x
  if (lhs->velocity_x != rhs->velocity_x) {
    return false;
  }
  // velocity_y
  if (lhs->velocity_y != rhs->velocity_y) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // dimensions
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->dimensions), &(rhs->dimensions)))
  {
    return false;
  }
  return true;
}

bool
perception_interfaces__msg__ObjectInfo__copy(
  const perception_interfaces__msg__ObjectInfo * input,
  perception_interfaces__msg__ObjectInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // class_id
  output->class_id = input->class_id;
  // score
  output->score = input->score;
  // velocity_x
  output->velocity_x = input->velocity_x;
  // velocity_y
  output->velocity_y = input->velocity_y;
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // dimensions
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->dimensions), &(output->dimensions)))
  {
    return false;
  }
  return true;
}

perception_interfaces__msg__ObjectInfo *
perception_interfaces__msg__ObjectInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  perception_interfaces__msg__ObjectInfo * msg = (perception_interfaces__msg__ObjectInfo *)allocator.allocate(sizeof(perception_interfaces__msg__ObjectInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(perception_interfaces__msg__ObjectInfo));
  bool success = perception_interfaces__msg__ObjectInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
perception_interfaces__msg__ObjectInfo__destroy(perception_interfaces__msg__ObjectInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    perception_interfaces__msg__ObjectInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
perception_interfaces__msg__ObjectInfo__Sequence__init(perception_interfaces__msg__ObjectInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  perception_interfaces__msg__ObjectInfo * data = NULL;

  if (size) {
    data = (perception_interfaces__msg__ObjectInfo *)allocator.zero_allocate(size, sizeof(perception_interfaces__msg__ObjectInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = perception_interfaces__msg__ObjectInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        perception_interfaces__msg__ObjectInfo__fini(&data[i - 1]);
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
perception_interfaces__msg__ObjectInfo__Sequence__fini(perception_interfaces__msg__ObjectInfo__Sequence * array)
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
      perception_interfaces__msg__ObjectInfo__fini(&array->data[i]);
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

perception_interfaces__msg__ObjectInfo__Sequence *
perception_interfaces__msg__ObjectInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  perception_interfaces__msg__ObjectInfo__Sequence * array = (perception_interfaces__msg__ObjectInfo__Sequence *)allocator.allocate(sizeof(perception_interfaces__msg__ObjectInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = perception_interfaces__msg__ObjectInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
perception_interfaces__msg__ObjectInfo__Sequence__destroy(perception_interfaces__msg__ObjectInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    perception_interfaces__msg__ObjectInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
perception_interfaces__msg__ObjectInfo__Sequence__are_equal(const perception_interfaces__msg__ObjectInfo__Sequence * lhs, const perception_interfaces__msg__ObjectInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!perception_interfaces__msg__ObjectInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
perception_interfaces__msg__ObjectInfo__Sequence__copy(
  const perception_interfaces__msg__ObjectInfo__Sequence * input,
  perception_interfaces__msg__ObjectInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(perception_interfaces__msg__ObjectInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    perception_interfaces__msg__ObjectInfo * data =
      (perception_interfaces__msg__ObjectInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!perception_interfaces__msg__ObjectInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          perception_interfaces__msg__ObjectInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!perception_interfaces__msg__ObjectInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
