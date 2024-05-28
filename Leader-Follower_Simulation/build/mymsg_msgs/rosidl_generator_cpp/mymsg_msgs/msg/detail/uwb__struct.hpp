// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mymsg_msgs:msg/Uwb.idl
// generated code does not contain a copyright notice

#ifndef MYMSG_MSGS__MSG__DETAIL__UWB__STRUCT_HPP_
#define MYMSG_MSGS__MSG__DETAIL__UWB__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mymsg_msgs__msg__Uwb __attribute__((deprecated))
#else
# define DEPRECATED__mymsg_msgs__msg__Uwb __declspec(deprecated)
#endif

namespace mymsg_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Uwb_
{
  using Type = Uwb_<ContainerAllocator>;

  explicit Uwb_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->range = 0.0;
      this->aoa = 0.0;
    }
  }

  explicit Uwb_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->range = 0.0;
      this->aoa = 0.0;
    }
  }

  // field types and members
  using _range_type =
    double;
  _range_type range;
  using _aoa_type =
    double;
  _aoa_type aoa;

  // setters for named parameter idiom
  Type & set__range(
    const double & _arg)
  {
    this->range = _arg;
    return *this;
  }
  Type & set__aoa(
    const double & _arg)
  {
    this->aoa = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mymsg_msgs::msg::Uwb_<ContainerAllocator> *;
  using ConstRawPtr =
    const mymsg_msgs::msg::Uwb_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mymsg_msgs::msg::Uwb_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mymsg_msgs::msg::Uwb_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mymsg_msgs::msg::Uwb_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mymsg_msgs::msg::Uwb_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mymsg_msgs::msg::Uwb_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mymsg_msgs::msg::Uwb_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mymsg_msgs::msg::Uwb_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mymsg_msgs::msg::Uwb_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mymsg_msgs__msg__Uwb
    std::shared_ptr<mymsg_msgs::msg::Uwb_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mymsg_msgs__msg__Uwb
    std::shared_ptr<mymsg_msgs::msg::Uwb_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Uwb_ & other) const
  {
    if (this->range != other.range) {
      return false;
    }
    if (this->aoa != other.aoa) {
      return false;
    }
    return true;
  }
  bool operator!=(const Uwb_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Uwb_

// alias to use template instance with default allocator
using Uwb =
  mymsg_msgs::msg::Uwb_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mymsg_msgs

#endif  // MYMSG_MSGS__MSG__DETAIL__UWB__STRUCT_HPP_
