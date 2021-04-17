// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from common:msg/Wheels.idl
// generated code does not contain a copyright notice

#ifndef COMMON__MSG__DETAIL__WHEELS__STRUCT_H_
#define COMMON__MSG__DETAIL__WHEELS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/Wheels in the package common.
typedef struct common__msg__Wheels
{
  std_msgs__msg__Header header;
  float param[2];
} common__msg__Wheels;

// Struct for a sequence of common__msg__Wheels.
typedef struct common__msg__Wheels__Sequence
{
  common__msg__Wheels * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} common__msg__Wheels__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // COMMON__MSG__DETAIL__WHEELS__STRUCT_H_
