// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_interfaces:msg/ObjectInfo.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_INFO__STRUCT_H_
#define PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'dimensions'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/ObjectInfo in the package perception_interfaces.
/**
  * perception_interfaces/msg/ObjectInfo.msg
 */
typedef struct perception_interfaces__msg__ObjectInfo
{
  /// 클래스 ID (0:Unknown, 1:Car, 9:Pedestrian 등)
  int32_t class_id;
  /// 인식 신뢰도 (0.0 ~ 1.0)
  float score;
  /// x축 속도 (m/s)
  float velocity_x;
  /// y축 속도 (m/s)
  float velocity_y;
  /// 중심 좌표(x,y,z) 및 회전(quaternion)
  geometry_msgs__msg__Pose pose;
  /// 물체 크기 (x, y, z)
  geometry_msgs__msg__Vector3 dimensions;
} perception_interfaces__msg__ObjectInfo;

// Struct for a sequence of perception_interfaces__msg__ObjectInfo.
typedef struct perception_interfaces__msg__ObjectInfo__Sequence
{
  perception_interfaces__msg__ObjectInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_interfaces__msg__ObjectInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_INFO__STRUCT_H_
