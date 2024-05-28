// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mymsg_msgs:msg/Uwb.idl
// generated code does not contain a copyright notice

#ifndef MYMSG_MSGS__MSG__DETAIL__UWB__TRAITS_HPP_
#define MYMSG_MSGS__MSG__DETAIL__UWB__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mymsg_msgs/msg/detail/uwb__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mymsg_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Uwb & msg,
  std::ostream & out)
{
  out << "{";
  // member: range
  {
    out << "range: ";
    rosidl_generator_traits::value_to_yaml(msg.range, out);
    out << ", ";
  }

  // member: aoa
  {
    out << "aoa: ";
    rosidl_generator_traits::value_to_yaml(msg.aoa, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Uwb & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range: ";
    rosidl_generator_traits::value_to_yaml(msg.range, out);
    out << "\n";
  }

  // member: aoa
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aoa: ";
    rosidl_generator_traits::value_to_yaml(msg.aoa, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Uwb & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mymsg_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mymsg_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mymsg_msgs::msg::Uwb & msg,
  std::ostream & out, size_t indentation = 0)
{
  mymsg_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mymsg_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mymsg_msgs::msg::Uwb & msg)
{
  return mymsg_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mymsg_msgs::msg::Uwb>()
{
  return "mymsg_msgs::msg::Uwb";
}

template<>
inline const char * name<mymsg_msgs::msg::Uwb>()
{
  return "mymsg_msgs/msg/Uwb";
}

template<>
struct has_fixed_size<mymsg_msgs::msg::Uwb>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mymsg_msgs::msg::Uwb>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mymsg_msgs::msg::Uwb>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MYMSG_MSGS__MSG__DETAIL__UWB__TRAITS_HPP_
