// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from common:msg/Wheels.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "common/msg/detail/wheels__rosidl_typesupport_introspection_c.h"
#include "common/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "common/msg/detail/wheels__functions.h"
#include "common/msg/detail/wheels__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Wheels__rosidl_typesupport_introspection_c__Wheels_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  common__msg__Wheels__init(message_memory);
}

void Wheels__rosidl_typesupport_introspection_c__Wheels_fini_function(void * message_memory)
{
  common__msg__Wheels__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Wheels__rosidl_typesupport_introspection_c__Wheels_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(common__msg__Wheels, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "param",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(common__msg__Wheels, param),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Wheels__rosidl_typesupport_introspection_c__Wheels_message_members = {
  "common__msg",  // message namespace
  "Wheels",  // message name
  2,  // number of fields
  sizeof(common__msg__Wheels),
  Wheels__rosidl_typesupport_introspection_c__Wheels_message_member_array,  // message members
  Wheels__rosidl_typesupport_introspection_c__Wheels_init_function,  // function to initialize message memory (memory has to be allocated)
  Wheels__rosidl_typesupport_introspection_c__Wheels_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Wheels__rosidl_typesupport_introspection_c__Wheels_message_type_support_handle = {
  0,
  &Wheels__rosidl_typesupport_introspection_c__Wheels_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_common
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, common, msg, Wheels)() {
  Wheels__rosidl_typesupport_introspection_c__Wheels_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!Wheels__rosidl_typesupport_introspection_c__Wheels_message_type_support_handle.typesupport_identifier) {
    Wheels__rosidl_typesupport_introspection_c__Wheels_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Wheels__rosidl_typesupport_introspection_c__Wheels_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
