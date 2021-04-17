// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from common:msg/Wheels.idl
// generated code does not contain a copyright notice

#ifndef COMMON__MSG__DETAIL__WHEELS__TRAITS_HPP_
#define COMMON__MSG__DETAIL__WHEELS__TRAITS_HPP_

#include "common/msg/detail/wheels__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<common::msg::Wheels>()
{
  return "common::msg::Wheels";
}

template<>
inline const char * name<common::msg::Wheels>()
{
  return "common/msg/Wheels";
}

template<>
struct has_fixed_size<common::msg::Wheels>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<common::msg::Wheels>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<common::msg::Wheels>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // COMMON__MSG__DETAIL__WHEELS__TRAITS_HPP_
