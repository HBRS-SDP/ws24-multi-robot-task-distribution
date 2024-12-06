// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot3_msgs:msg/VersionInfo.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__MSG__DETAIL__VERSION_INFO__STRUCT_HPP_
#define TURTLEBOT3_MSGS__MSG__DETAIL__VERSION_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtlebot3_msgs__msg__VersionInfo __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_msgs__msg__VersionInfo __declspec(deprecated)
#endif

namespace turtlebot3_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VersionInfo_
{
  using Type = VersionInfo_<ContainerAllocator>;

  explicit VersionInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->hardware = "";
      this->firmware = "";
      this->software = "";
    }
  }

  explicit VersionInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : hardware(_alloc),
    firmware(_alloc),
    software(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->hardware = "";
      this->firmware = "";
      this->software = "";
    }
  }

  // field types and members
  using _hardware_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _hardware_type hardware;
  using _firmware_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _firmware_type firmware;
  using _software_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _software_type software;

  // setters for named parameter idiom
  Type & set__hardware(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->hardware = _arg;
    return *this;
  }
  Type & set__firmware(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->firmware = _arg;
    return *this;
  }
  Type & set__software(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->software = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_msgs::msg::VersionInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_msgs::msg::VersionInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_msgs::msg::VersionInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_msgs::msg::VersionInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::msg::VersionInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::msg::VersionInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::msg::VersionInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::msg::VersionInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_msgs::msg::VersionInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_msgs::msg::VersionInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_msgs__msg__VersionInfo
    std::shared_ptr<turtlebot3_msgs::msg::VersionInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_msgs__msg__VersionInfo
    std::shared_ptr<turtlebot3_msgs::msg::VersionInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VersionInfo_ & other) const
  {
    if (this->hardware != other.hardware) {
      return false;
    }
    if (this->firmware != other.firmware) {
      return false;
    }
    if (this->software != other.software) {
      return false;
    }
    return true;
  }
  bool operator!=(const VersionInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VersionInfo_

// alias to use template instance with default allocator
using VersionInfo =
  turtlebot3_msgs::msg::VersionInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace turtlebot3_msgs

#endif  // TURTLEBOT3_MSGS__MSG__DETAIL__VERSION_INFO__STRUCT_HPP_
