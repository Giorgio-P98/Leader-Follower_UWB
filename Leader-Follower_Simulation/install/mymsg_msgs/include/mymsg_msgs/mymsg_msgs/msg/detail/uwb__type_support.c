// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mymsg_msgs:msg/Uwb.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mymsg_msgs/msg/detail/uwb__rosidl_typesupport_introspection_c.h"
#include "mymsg_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mymsg_msgs/msg/detail/uwb__functions.h"
#include "mymsg_msgs/msg/detail/uwb__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mymsg_msgs__msg__Uwb__rosidl_typesupport_introspection_c__Uwb_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mymsg_msgs__msg__Uwb__init(message_memory);
}

void mymsg_msgs__msg__Uwb__rosidl_typesupport_introspection_c__Uwb_fini_function(void * message_memory)
{
  mymsg_msgs__msg__Uwb__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mymsg_msgs__msg__Uwb__rosidl_typesupport_introspection_c__Uwb_message_member_array[2] = {
  {
    "range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mymsg_msgs__msg__Uwb, range),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "aoa",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mymsg_msgs__msg__Uwb, aoa),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mymsg_msgs__msg__Uwb__rosidl_typesupport_introspection_c__Uwb_message_members = {
  "mymsg_msgs__msg",  // message namespace
  "Uwb",  // message name
  2,  // number of fields
  sizeof(mymsg_msgs__msg__Uwb),
  mymsg_msgs__msg__Uwb__rosidl_typesupport_introspection_c__Uwb_message_member_array,  // message members
  mymsg_msgs__msg__Uwb__rosidl_typesupport_introspection_c__Uwb_init_function,  // function to initialize message memory (memory has to be allocated)
  mymsg_msgs__msg__Uwb__rosidl_typesupport_introspection_c__Uwb_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mymsg_msgs__msg__Uwb__rosidl_typesupport_introspection_c__Uwb_message_type_support_handle = {
  0,
  &mymsg_msgs__msg__Uwb__rosidl_typesupport_introspection_c__Uwb_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mymsg_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mymsg_msgs, msg, Uwb)() {
  if (!mymsg_msgs__msg__Uwb__rosidl_typesupport_introspection_c__Uwb_message_type_support_handle.typesupport_identifier) {
    mymsg_msgs__msg__Uwb__rosidl_typesupport_introspection_c__Uwb_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mymsg_msgs__msg__Uwb__rosidl_typesupport_introspection_c__Uwb_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
