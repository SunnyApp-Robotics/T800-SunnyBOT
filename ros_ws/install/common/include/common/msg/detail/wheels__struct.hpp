// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from common:msg/Wheels.idl
// generated code does not contain a copyright notice

#ifndef COMMON__MSG__DETAIL__WHEELS__STRUCT_HPP_
#define COMMON__MSG__DETAIL__WHEELS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__common__msg__Wheels __attribute__((deprecated))
#else
# define DEPRECATED__common__msg__Wheels __declspec(deprecated)
#endif

namespace common
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Wheels_
{
  using Type = Wheels_<ContainerAllocator>;

  explicit Wheels_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 2>::iterator, float>(this->param.begin(), this->param.end(), 0.0f);
    }
  }

  explicit Wheels_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    param(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 2>::iterator, float>(this->param.begin(), this->param.end(), 0.0f);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _param_type =
    std::array<float, 2>;
  _param_type param;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__param(
    const std::array<float, 2> & _arg)
  {
    this->param = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    common::msg::Wheels_<ContainerAllocator> *;
  using ConstRawPtr =
    const common::msg::Wheels_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<common::msg::Wheels_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<common::msg::Wheels_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      common::msg::Wheels_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<common::msg::Wheels_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      common::msg::Wheels_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<common::msg::Wheels_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<common::msg::Wheels_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<common::msg::Wheels_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__common__msg__Wheels
    std::shared_ptr<common::msg::Wheels_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__common__msg__Wheels
    std::shared_ptr<common::msg::Wheels_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Wheels_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->param != other.param) {
      return false;
    }
    return true;
  }
  bool operator!=(const Wheels_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Wheels_

// alias to use template instance with default allocator
using Wheels =
  common::msg::Wheels_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace common

#endif  // COMMON__MSG__DETAIL__WHEELS__STRUCT_HPP_
