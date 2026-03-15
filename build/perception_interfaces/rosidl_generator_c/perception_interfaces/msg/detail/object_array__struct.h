// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_interfaces:msg/ObjectArray.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_ARRAY__STRUCT_H_
#define PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'objects'
#include "perception_interfaces/msg/detail/object_info__struct.h"

/// Struct defined in msg/ObjectArray in the package perception_interfaces.
/**
  * perception_interfaces/msg/ObjectArray.msg
 */
typedef struct perception_interfaces__msg__ObjectArray
{
  /// 타임스탬프 및 프레임 ID
  std_msgs__msg__Header header;
  /// ObjectInfo의 배열
  perception_interfaces__msg__ObjectInfo__Sequence objects;
} perception_interfaces__msg__ObjectArray;

// Struct for a sequence of perception_interfaces__msg__ObjectArray.
typedef struct perception_interfaces__msg__ObjectArray__Sequence
{
  perception_interfaces__msg__ObjectArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_interfaces__msg__ObjectArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_ARRAY__STRUCT_H_
