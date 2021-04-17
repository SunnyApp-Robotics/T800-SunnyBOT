// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from common:msg/Wheels.idl
// generated code does not contain a copyright notice
#include "common/msg/detail/wheels__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
common__msg__Wheels__init(common__msg__Wheels * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    common__msg__Wheels__fini(msg);
    return false;
  }
  // param
  return true;
}

void
common__msg__Wheels__fini(common__msg__Wheels * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // param
}

common__msg__Wheels *
common__msg__Wheels__create()
{
  common__msg__Wheels * msg = (common__msg__Wheels *)malloc(sizeof(common__msg__Wheels));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(common__msg__Wheels));
  bool success = common__msg__Wheels__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
common__msg__Wheels__destroy(common__msg__Wheels * msg)
{
  if (msg) {
    common__msg__Wheels__fini(msg);
  }
  free(msg);
}


bool
common__msg__Wheels__Sequence__init(common__msg__Wheels__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  common__msg__Wheels * data = NULL;
  if (size) {
    data = (common__msg__Wheels *)calloc(size, sizeof(common__msg__Wheels));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = common__msg__Wheels__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        common__msg__Wheels__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
common__msg__Wheels__Sequence__fini(common__msg__Wheels__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      common__msg__Wheels__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

common__msg__Wheels__Sequence *
common__msg__Wheels__Sequence__create(size_t size)
{
  common__msg__Wheels__Sequence * array = (common__msg__Wheels__Sequence *)malloc(sizeof(common__msg__Wheels__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = common__msg__Wheels__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
common__msg__Wheels__Sequence__destroy(common__msg__Wheels__Sequence * array)
{
  if (array) {
    common__msg__Wheels__Sequence__fini(array);
  }
  free(array);
}
