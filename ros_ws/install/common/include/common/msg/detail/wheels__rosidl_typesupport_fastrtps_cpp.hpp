// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from common:msg/Wheels.idl
// generated code does not contain a copyright notice

#ifndef COMMON__MSG__DETAIL__WHEELS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define COMMON__MSG__DETAIL__WHEELS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "common/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "common/msg/detail/wheels__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace common
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_common
cdr_serialize(
  const common::msg::Wheels & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_common
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  common::msg::Wheels & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_common
get_serialized_size(
  const common::msg::Wheels & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_common
max_serialized_size_Wheels(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace common

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_common
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, common, msg, Wheels)();

#ifdef __cplusplus
}
#endif

#endif  // COMMON__MSG__DETAIL__WHEELS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
