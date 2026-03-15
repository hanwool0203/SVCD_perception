// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from perception_interfaces:msg/ObjectInfo.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_INFO__STRUCT_HPP_
#define PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'dimensions'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__perception_interfaces__msg__ObjectInfo __attribute__((deprecated))
#else
# define DEPRECATED__perception_interfaces__msg__ObjectInfo __declspec(deprecated)
#endif

namespace perception_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectInfo_
{
  using Type = ObjectInfo_<ContainerAllocator>;

  explicit ObjectInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init),
    dimensions(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = 0l;
      this->score = 0.0f;
      this->velocity_x = 0.0f;
      this->velocity_y = 0.0f;
    }
  }

  explicit ObjectInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init),
    dimensions(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = 0l;
      this->score = 0.0f;
      this->velocity_x = 0.0f;
      this->velocity_y = 0.0f;
    }
  }

  // field types and members
  using _class_id_type =
    int32_t;
  _class_id_type class_id;
  using _score_type =
    float;
  _score_type score;
  using _velocity_x_type =
    float;
  _velocity_x_type velocity_x;
  using _velocity_y_type =
    float;
  _velocity_y_type velocity_y;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _dimensions_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _dimensions_type dimensions;

  // setters for named parameter idiom
  Type & set__class_id(
    const int32_t & _arg)
  {
    this->class_id = _arg;
    return *this;
  }
  Type & set__score(
    const float & _arg)
  {
    this->score = _arg;
    return *this;
  }
  Type & set__velocity_x(
    const float & _arg)
  {
    this->velocity_x = _arg;
    return *this;
  }
  Type & set__velocity_y(
    const float & _arg)
  {
    this->velocity_y = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__dimensions(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->dimensions = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    perception_interfaces::msg::ObjectInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const perception_interfaces::msg::ObjectInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<perception_interfaces::msg::ObjectInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<perception_interfaces::msg::ObjectInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      perception_interfaces::msg::ObjectInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<perception_interfaces::msg::ObjectInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      perception_interfaces::msg::ObjectInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<perception_interfaces::msg::ObjectInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<perception_interfaces::msg::ObjectInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<perception_interfaces::msg::ObjectInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__perception_interfaces__msg__ObjectInfo
    std::shared_ptr<perception_interfaces::msg::ObjectInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__perception_interfaces__msg__ObjectInfo
    std::shared_ptr<perception_interfaces::msg::ObjectInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectInfo_ & other) const
  {
    if (this->class_id != other.class_id) {
      return false;
    }
    if (this->score != other.score) {
      return false;
    }
    if (this->velocity_x != other.velocity_x) {
      return false;
    }
    if (this->velocity_y != other.velocity_y) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->dimensions != other.dimensions) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectInfo_

// alias to use template instance with default allocator
using ObjectInfo =
  perception_interfaces::msg::ObjectInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace perception_interfaces

#endif  // PERCEPTION_INTERFACES__MSG__DETAIL__OBJECT_INFO__STRUCT_HPP_
