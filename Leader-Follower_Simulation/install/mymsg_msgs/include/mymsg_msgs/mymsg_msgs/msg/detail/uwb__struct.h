// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mymsg_msgs:msg/Uwb.idl
// generated code does not contain a copyright notice

#ifndef MYMSG_MSGS__MSG__DETAIL__UWB__STRUCT_H_
#define MYMSG_MSGS__MSG__DETAIL__UWB__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Uwb in the package mymsg_msgs.
typedef struct mymsg_msgs__msg__Uwb
{
  double range;
  double aoa;
} mymsg_msgs__msg__Uwb;

// Struct for a sequence of mymsg_msgs__msg__Uwb.
typedef struct mymsg_msgs__msg__Uwb__Sequence
{
  mymsg_msgs__msg__Uwb * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mymsg_msgs__msg__Uwb__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MYMSG_MSGS__MSG__DETAIL__UWB__STRUCT_H_
