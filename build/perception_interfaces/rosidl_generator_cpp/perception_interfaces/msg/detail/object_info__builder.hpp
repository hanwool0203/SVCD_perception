// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from perception_interfaces:msg/ObjectInfo.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_INFO__BUILDER_HPP_
#define PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "perception_interfaces/msg/detail/object_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace perception_interfaces
{

namespace msg
{

namespace builder
{

class Init_ObjectInfo_dimensions
{
public:
  explicit Init_ObjectInfo_dimensions(::perception_interfaces::msg::ObjectInfo & msg)
  : msg_(msg)
  {}
  ::perception_interfaces::msg::ObjectInfo dimensions(::perception_interfaces::msg::ObjectInfo::_dimensions_type arg)
  {
    msg_.dimensions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::perception_interfaces::msg::ObjectInfo msg_;
};

class Init_ObjectInfo_pose
{
public:
  explicit Init_ObjectInfo_pose(::perception_interfaces::msg::ObjectInfo & msg)
  : msg_(msg)
  {}
  Init_ObjectInfo_dimensions pose(::perception_interfaces::msg::ObjectInfo::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_ObjectInfo_dimensions(msg_);
  }

private:
  ::perception_interfaces::msg::ObjectInfo msg_;
};

class Init_ObjectInfo_velocity_y
{
public:
  explicit Init_ObjectInfo_velocity_y(::perception_interfaces::msg::ObjectInfo & msg)
  : msg_(msg)
  {}
  Init_ObjectInfo_pose velocity_y(::perception_interfaces::msg::ObjectInfo::_velocity_y_type arg)
  {
    msg_.velocity_y = std::move(arg);
    return Init_ObjectInfo_pose(msg_);
  }

private:
  ::perception_interfaces::msg::ObjectInfo msg_;
};

class Init_ObjectInfo_velocity_x
{
public:
  explicit Init_ObjectInfo_velocity_x(::perception_interfaces::msg::ObjectInfo & msg)
  : msg_(msg)
  {}
  Init_ObjectInfo_velocity_y velocity_x(::perception_interfaces::msg::ObjectInfo::_velocity_x_type arg)
  {
    msg_.velocity_x = std::move(arg);
    return Init_ObjectInfo_velocity_y(msg_);
  }

private:
  ::perception_interfaces::msg::ObjectInfo msg_;
};

class Init_ObjectInfo_score
{
public:
  explicit Init_ObjectInfo_score(::perception_interfaces::msg::ObjectInfo & msg)
  : msg_(msg)
  {}
  Init_ObjectInfo_velocity_x score(::perception_interfaces::msg::ObjectInfo::_score_type arg)
  {
    msg_.score = std::move(arg);
    return Init_ObjectInfo_velocity_x(msg_);
  }

private:
  ::perception_interfaces::msg::ObjectInfo msg_;
};

class Init_ObjectInfo_class_id
{
public:
  Init_ObjectInfo_class_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectInfo_score class_id(::perception_interfaces::msg::ObjectInfo::_class_id_type arg)
  {
    msg_.class_id = std::move(arg);
    return Init_ObjectInfo_score(msg_);
  }

private:
  ::perception_interfaces::msg::ObjectInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::perception_interfaces::msg::ObjectInfo>()
{
  return perception_interfaces::msg::builder::Init_ObjectInfo_class_id();
}

}  // namespace perception_interfaces

#endif  // PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_INFO__BUILDER_HPP_
