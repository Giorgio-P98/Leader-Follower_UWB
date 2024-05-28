// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mymsg_msgs:msg/Uwb.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mymsg_msgs/msg/detail/uwb__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mymsg_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Uwb_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mymsg_msgs::msg::Uwb(_init);
}

void Uwb_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mymsg_msgs::msg::Uwb *>(message_memory);
  typed_message->~Uwb();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Uwb_message_member_array[2] = {
  {
    "range",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mymsg_msgs::msg::Uwb, range),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "aoa",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mymsg_msgs::msg::Uwb, aoa),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Uwb_message_members = {
  "mymsg_msgs::msg",  // message namespace
  "Uwb",  // message name
  2,  // number of fields
  sizeof(mymsg_msgs::msg::Uwb),
  Uwb_message_member_array,  // message members
  Uwb_init_function,  // function to initialize message memory (memory has to be allocated)
  Uwb_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Uwb_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Uwb_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mymsg_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mymsg_msgs::msg::Uwb>()
{
  return &::mymsg_msgs::msg::rosidl_typesupport_introspection_cpp::Uwb_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mymsg_msgs, msg, Uwb)() {
  return &::mymsg_msgs::msg::rosidl_typesupport_introspection_cpp::Uwb_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
