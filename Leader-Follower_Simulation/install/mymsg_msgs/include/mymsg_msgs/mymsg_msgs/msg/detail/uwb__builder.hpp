// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mymsg_msgs:msg/Uwb.idl
// generated code does not contain a copyright notice

#ifndef MYMSG_MSGS__MSG__DETAIL__UWB__BUILDER_HPP_
#define MYMSG_MSGS__MSG__DETAIL__UWB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mymsg_msgs/msg/detail/uwb__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mymsg_msgs
{

namespace msg
{

namespace builder
{

class Init_Uwb_aoa
{
public:
  explicit Init_Uwb_aoa(::mymsg_msgs::msg::Uwb & msg)
  : msg_(msg)
  {}
  ::mymsg_msgs::msg::Uwb aoa(::mymsg_msgs::msg::Uwb::_aoa_type arg)
  {
    msg_.aoa = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mymsg_msgs::msg::Uwb msg_;
};

class Init_Uwb_range
{
public:
  Init_Uwb_range()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Uwb_aoa range(::mymsg_msgs::msg::Uwb::_range_type arg)
  {
    msg_.range = std::move(arg);
    return Init_Uwb_aoa(msg_);
  }

private:
  ::mymsg_msgs::msg::Uwb msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mymsg_msgs::msg::Uwb>()
{
  return mymsg_msgs::msg::builder::Init_Uwb_range();
}

}  // namespace mymsg_msgs

#endif  // MYMSG_MSGS__MSG__DETAIL__UWB__BUILDER_HPP_
