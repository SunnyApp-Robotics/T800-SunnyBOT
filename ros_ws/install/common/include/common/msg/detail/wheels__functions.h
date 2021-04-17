// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from common:msg/Wheels.idl
// generated code does not contain a copyright notice

#ifndef COMMON__MSG__DETAIL__WHEELS__FUNCTIONS_H_
#define COMMON__MSG__DETAIL__WHEELS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "common/msg/rosidl_generator_c__visibility_control.h"

#include "common/msg/detail/wheels__struct.h"

/// Initialize msg/Wheels message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * common__msg__Wheels
 * )) before or use
 * common__msg__Wheels__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_common
bool
common__msg__Wheels__init(common__msg__Wheels * msg);

/// Finalize msg/Wheels message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_common
void
common__msg__Wheels__fini(common__msg__Wheels * msg);

/// Create msg/Wheels message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * common__msg__Wheels__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_common
common__msg__Wheels *
common__msg__Wheels__create();

/// Destroy msg/Wheels message.
/**
 * It calls
 * common__msg__Wheels__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_common
void
common__msg__Wheels__destroy(common__msg__Wheels * msg);


/// Initialize array of msg/Wheels messages.
/**
 * It allocates the memory for the number of elements and calls
 * common__msg__Wheels__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_common
bool
common__msg__Wheels__Sequence__init(common__msg__Wheels__Sequence * array, size_t size);

/// Finalize array of msg/Wheels messages.
/**
 * It calls
 * common__msg__Wheels__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_common
void
common__msg__Wheels__Sequence__fini(common__msg__Wheels__Sequence * array);

/// Create array of msg/Wheels messages.
/**
 * It allocates the memory for the array and calls
 * common__msg__Wheels__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_common
common__msg__Wheels__Sequence *
common__msg__Wheels__Sequence__create(size_t size);

/// Destroy array of msg/Wheels messages.
/**
 * It calls
 * common__msg__Wheels__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_common
void
common__msg__Wheels__Sequence__destroy(common__msg__Wheels__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // COMMON__MSG__DETAIL__WHEELS__FUNCTIONS_H_
