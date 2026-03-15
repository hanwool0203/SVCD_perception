// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from perception_interfaces:msg/ObjectArray.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_ARRAY__BUILDER_HPP_
#define PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "perception_interfaces/msg/detail/object_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace perception_interfaces
{

namespace msg
{

namespace builder
{

class Init_ObjectArray_objects
{
public:
  explicit Init_ObjectArray_objects(::perception_interfaces::msg::ObjectArray & msg)
  : msg_(msg)
  {}
  ::perception_interfaces::msg::ObjectArray objects(::perception_interfaces::msg::ObjectArray::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::perception_interfaces::msg::ObjectArray msg_;
};

class Init_ObjectArray_header
{
public:
  Init_ObjectArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectArray_objects header(::perception_interfaces::msg::ObjectArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObjectArray_objects(msg_);
  }

private:
  ::perception_interfaces::msg::ObjectArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::perception_interfaces::msg::ObjectArray>()
{
  return perception_interfaces::msg::builder::Init_ObjectArray_header();
}

}  // namespace perception_interfaces

#endif  // PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_ARRAY__BUILDER_HPP_
