// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from common:msg/Wheels.idl
// generated code does not contain a copyright notice

#ifndef COMMON__MSG__DETAIL__WHEELS__BUILDER_HPP_
#define COMMON__MSG__DETAIL__WHEELS__BUILDER_HPP_

#include "common/msg/detail/wheels__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace common
{

namespace msg
{

namespace builder
{

class Init_Wheels_param
{
public:
  explicit Init_Wheels_param(::common::msg::Wheels & msg)
  : msg_(msg)
  {}
  ::common::msg::Wheels param(::common::msg::Wheels::_param_type arg)
  {
    msg_.param = std::move(arg);
    return std::move(msg_);
  }

private:
  ::common::msg::Wheels msg_;
};

class Init_Wheels_header
{
public:
  Init_Wheels_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Wheels_param header(::common::msg::Wheels::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Wheels_param(msg_);
  }

private:
  ::common::msg::Wheels msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::common::msg::Wheels>()
{
  return common::msg::builder::Init_Wheels_header();
}

}  // namespace common

#endif  // COMMON__MSG__DETAIL__WHEELS__BUILDER_HPP_
