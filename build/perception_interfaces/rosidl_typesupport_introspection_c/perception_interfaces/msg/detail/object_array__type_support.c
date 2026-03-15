// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from perception_interfaces:msg/ObjectArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "perception_interfaces/msg/detail/object_array__rosidl_typesupport_introspection_c.h"
#include "perception_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "perception_interfaces/msg/detail/object_array__functions.h"
#include "perception_interfaces/msg/detail/object_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `objects`
#include "perception_interfaces/msg/object_info.h"
// Member `objects`
#include "perception_interfaces/msg/detail/object_info__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  perception_interfaces__msg__ObjectArray__init(message_memory);
}

void perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_fini_function(void * message_memory)
{
  perception_interfaces__msg__ObjectArray__fini(message_memory);
}

size_t perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__size_function__ObjectArray__objects(
  const void * untyped_member)
{
  const perception_interfaces__msg__ObjectInfo__Sequence * member =
    (const perception_interfaces__msg__ObjectInfo__Sequence *)(untyped_member);
  return member->size;
}

const void * perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__get_const_function__ObjectArray__objects(
  const void * untyped_member, size_t index)
{
  const perception_interfaces__msg__ObjectInfo__Sequence * member =
    (const perception_interfaces__msg__ObjectInfo__Sequence *)(untyped_member);
  return &member->data[index];
}

void * perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__get_function__ObjectArray__objects(
  void * untyped_member, size_t index)
{
  perception_interfaces__msg__ObjectInfo__Sequence * member =
    (perception_interfaces__msg__ObjectInfo__Sequence *)(untyped_member);
  return &member->data[index];
}

void perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__fetch_function__ObjectArray__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const perception_interfaces__msg__ObjectInfo * item =
    ((const perception_interfaces__msg__ObjectInfo *)
    perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__get_const_function__ObjectArray__objects(untyped_member, index));
  perception_interfaces__msg__ObjectInfo * value =
    (perception_interfaces__msg__ObjectInfo *)(untyped_value);
  *value = *item;
}

void perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__assign_function__ObjectArray__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  perception_interfaces__msg__ObjectInfo * item =
    ((perception_interfaces__msg__ObjectInfo *)
    perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__get_function__ObjectArray__objects(untyped_member, index));
  const perception_interfaces__msg__ObjectInfo * value =
    (const perception_interfaces__msg__ObjectInfo *)(untyped_value);
  *item = *value;
}

bool perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__resize_function__ObjectArray__objects(
  void * untyped_member, size_t size)
{
  perception_interfaces__msg__ObjectInfo__Sequence * member =
    (perception_interfaces__msg__ObjectInfo__Sequence *)(untyped_member);
  perception_interfaces__msg__ObjectInfo__Sequence__fini(member);
  return perception_interfaces__msg__ObjectInfo__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_interfaces__msg__ObjectArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(perception_interfaces__msg__ObjectArray, objects),  // bytes offset in struct
    NULL,  // default value
    perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__size_function__ObjectArray__objects,  // size() function pointer
    perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__get_const_function__ObjectArray__objects,  // get_const(index) function pointer
    perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__get_function__ObjectArray__objects,  // get(index) function pointer
    perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__fetch_function__ObjectArray__objects,  // fetch(index, &value) function pointer
    perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__assign_function__ObjectArray__objects,  // assign(index, value) function pointer
    perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__resize_function__ObjectArray__objects  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_members = {
  "perception_interfaces__msg",  // message namespace
  "ObjectArray",  // message name
  2,  // number of fields
  sizeof(perception_interfaces__msg__ObjectArray),
  perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_member_array,  // message members
  perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_init_function,  // function to initialize message memory (memory has to be allocated)
  perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_type_support_handle = {
  0,
  &perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_perception_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_interfaces, msg, ObjectArray)() {
  perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, perception_interfaces, msg, ObjectInfo)();
  if (!perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_type_support_handle.typesupport_identifier) {
    perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &perception_interfaces__msg__ObjectArray__rosidl_typesupport_introspection_c__ObjectArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
