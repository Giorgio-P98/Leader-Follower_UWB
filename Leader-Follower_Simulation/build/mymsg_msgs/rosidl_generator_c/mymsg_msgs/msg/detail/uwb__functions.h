// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mymsg_msgs:msg/Uwb.idl
// generated code does not contain a copyright notice

#ifndef MYMSG_MSGS__MSG__DETAIL__UWB__FUNCTIONS_H_
#define MYMSG_MSGS__MSG__DETAIL__UWB__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "mymsg_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "mymsg_msgs/msg/detail/uwb__struct.h"

/// Initialize msg/Uwb message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mymsg_msgs__msg__Uwb
 * )) before or use
 * mymsg_msgs__msg__Uwb__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mymsg_msgs
bool
mymsg_msgs__msg__Uwb__init(mymsg_msgs__msg__Uwb * msg);

/// Finalize msg/Uwb message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mymsg_msgs
void
mymsg_msgs__msg__Uwb__fini(mymsg_msgs__msg__Uwb * msg);

/// Create msg/Uwb message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mymsg_msgs__msg__Uwb__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mymsg_msgs
mymsg_msgs__msg__Uwb *
mymsg_msgs__msg__Uwb__create();

/// Destroy msg/Uwb message.
/**
 * It calls
 * mymsg_msgs__msg__Uwb__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mymsg_msgs
void
mymsg_msgs__msg__Uwb__destroy(mymsg_msgs__msg__Uwb * msg);

/// Check for msg/Uwb message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mymsg_msgs
bool
mymsg_msgs__msg__Uwb__are_equal(const mymsg_msgs__msg__Uwb * lhs, const mymsg_msgs__msg__Uwb * rhs);

/// Copy a msg/Uwb message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mymsg_msgs
bool
mymsg_msgs__msg__Uwb__copy(
  const mymsg_msgs__msg__Uwb * input,
  mymsg_msgs__msg__Uwb * output);

/// Initialize array of msg/Uwb messages.
/**
 * It allocates the memory for the number of elements and calls
 * mymsg_msgs__msg__Uwb__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mymsg_msgs
bool
mymsg_msgs__msg__Uwb__Sequence__init(mymsg_msgs__msg__Uwb__Sequence * array, size_t size);

/// Finalize array of msg/Uwb messages.
/**
 * It calls
 * mymsg_msgs__msg__Uwb__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mymsg_msgs
void
mymsg_msgs__msg__Uwb__Sequence__fini(mymsg_msgs__msg__Uwb__Sequence * array);

/// Create array of msg/Uwb messages.
/**
 * It allocates the memory for the array and calls
 * mymsg_msgs__msg__Uwb__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mymsg_msgs
mymsg_msgs__msg__Uwb__Sequence *
mymsg_msgs__msg__Uwb__Sequence__create(size_t size);

/// Destroy array of msg/Uwb messages.
/**
 * It calls
 * mymsg_msgs__msg__Uwb__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mymsg_msgs
void
mymsg_msgs__msg__Uwb__Sequence__destroy(mymsg_msgs__msg__Uwb__Sequence * array);

/// Check for msg/Uwb message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mymsg_msgs
bool
mymsg_msgs__msg__Uwb__Sequence__are_equal(const mymsg_msgs__msg__Uwb__Sequence * lhs, const mymsg_msgs__msg__Uwb__Sequence * rhs);

/// Copy an array of msg/Uwb messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mymsg_msgs
bool
mymsg_msgs__msg__Uwb__Sequence__copy(
  const mymsg_msgs__msg__Uwb__Sequence * input,
  mymsg_msgs__msg__Uwb__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MYMSG_MSGS__MSG__DETAIL__UWB__FUNCTIONS_H_
