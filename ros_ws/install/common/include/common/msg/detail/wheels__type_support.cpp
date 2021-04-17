// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from common:msg/Wheels.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "common/msg/detail/wheels__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace common
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Wheels_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) common::msg::Wheels(_init);
}

void Wheels_fini_function(void * message_memory)
{
  auto typed_message = static_cast<common::msg::Wheels *>(message_memory);
  typed_message->~Wheels();
}

size_t size_function__Wheels__param(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__Wheels__param(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__Wheels__param(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 2> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Wheels_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(common::msg::Wheels, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "param",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(common::msg::Wheels, param),  // bytes offset in struct
    nullptr,  // default value
    size_function__Wheels__param,  // size() function pointer
    get_const_function__Wheels__param,  // get_const(index) function pointer
    get_function__Wheels__param,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Wheels_message_members = {
  "common::msg",  // message namespace
  "Wheels",  // message name
  2,  // number of fields
  sizeof(common::msg::Wheels),
  Wheels_message_member_array,  // message members
  Wheels_init_function,  // function to initialize message memory (memory has to be allocated)
  Wheels_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Wheels_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Wheels_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace common


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<common::msg::Wheels>()
{
  return &::common::msg::rosidl_typesupport_introspection_cpp::Wheels_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, common, msg, Wheels)() {
  return &::common::msg::rosidl_typesupport_introspection_cpp::Wheels_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
