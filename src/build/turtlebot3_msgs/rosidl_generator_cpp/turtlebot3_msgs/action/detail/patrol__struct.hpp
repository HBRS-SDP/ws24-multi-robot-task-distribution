// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot3_msgs:action/Patrol.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_MSGS__ACTION__DETAIL__PATROL__STRUCT_HPP_
#define TURTLEBOT3_MSGS__ACTION__DETAIL__PATROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtlebot3_msgs__action__Patrol_Goal __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_msgs__action__Patrol_Goal __declspec(deprecated)
#endif

namespace turtlebot3_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Patrol_Goal_
{
  using Type = Patrol_Goal_<ContainerAllocator>;

  explicit Patrol_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->radius = 0.0f;
    }
  }

  explicit Patrol_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->radius = 0.0f;
    }
  }

  // field types and members
  using _radius_type =
    float;
  _radius_type radius;

  // setters for named parameter idiom
  Type & set__radius(
    const float & _arg)
  {
    this->radius = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_msgs::action::Patrol_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_msgs::action::Patrol_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_Goal
    std::shared_ptr<turtlebot3_msgs::action::Patrol_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_Goal
    std::shared_ptr<turtlebot3_msgs::action::Patrol_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Patrol_Goal_ & other) const
  {
    if (this->radius != other.radius) {
      return false;
    }
    return true;
  }
  bool operator!=(const Patrol_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Patrol_Goal_

// alias to use template instance with default allocator
using Patrol_Goal =
  turtlebot3_msgs::action::Patrol_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace turtlebot3_msgs


#ifndef _WIN32
# define DEPRECATED__turtlebot3_msgs__action__Patrol_Result __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_msgs__action__Patrol_Result __declspec(deprecated)
#endif

namespace turtlebot3_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Patrol_Result_
{
  using Type = Patrol_Result_<ContainerAllocator>;

  explicit Patrol_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit Patrol_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_msgs::action::Patrol_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_msgs::action::Patrol_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_Result
    std::shared_ptr<turtlebot3_msgs::action::Patrol_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_Result
    std::shared_ptr<turtlebot3_msgs::action::Patrol_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Patrol_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const Patrol_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Patrol_Result_

// alias to use template instance with default allocator
using Patrol_Result =
  turtlebot3_msgs::action::Patrol_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace turtlebot3_msgs


#ifndef _WIN32
# define DEPRECATED__turtlebot3_msgs__action__Patrol_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_msgs__action__Patrol_Feedback __declspec(deprecated)
#endif

namespace turtlebot3_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Patrol_Feedback_
{
  using Type = Patrol_Feedback_<ContainerAllocator>;

  explicit Patrol_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_time = 0.0f;
    }
  }

  explicit Patrol_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_time = 0.0f;
    }
  }

  // field types and members
  using _left_time_type =
    float;
  _left_time_type left_time;

  // setters for named parameter idiom
  Type & set__left_time(
    const float & _arg)
  {
    this->left_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_msgs::action::Patrol_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_msgs::action::Patrol_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_Feedback
    std::shared_ptr<turtlebot3_msgs::action::Patrol_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_Feedback
    std::shared_ptr<turtlebot3_msgs::action::Patrol_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Patrol_Feedback_ & other) const
  {
    if (this->left_time != other.left_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const Patrol_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Patrol_Feedback_

// alias to use template instance with default allocator
using Patrol_Feedback =
  turtlebot3_msgs::action::Patrol_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace turtlebot3_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "turtlebot3_msgs/action/detail/patrol__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__turtlebot3_msgs__action__Patrol_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_msgs__action__Patrol_SendGoal_Request __declspec(deprecated)
#endif

namespace turtlebot3_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Patrol_SendGoal_Request_
{
  using Type = Patrol_SendGoal_Request_<ContainerAllocator>;

  explicit Patrol_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit Patrol_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    turtlebot3_msgs::action::Patrol_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const turtlebot3_msgs::action::Patrol_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_msgs::action::Patrol_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_msgs::action::Patrol_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_SendGoal_Request
    std::shared_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_SendGoal_Request
    std::shared_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Patrol_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const Patrol_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Patrol_SendGoal_Request_

// alias to use template instance with default allocator
using Patrol_SendGoal_Request =
  turtlebot3_msgs::action::Patrol_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace turtlebot3_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__turtlebot3_msgs__action__Patrol_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_msgs__action__Patrol_SendGoal_Response __declspec(deprecated)
#endif

namespace turtlebot3_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Patrol_SendGoal_Response_
{
  using Type = Patrol_SendGoal_Response_<ContainerAllocator>;

  explicit Patrol_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit Patrol_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_msgs::action::Patrol_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_msgs::action::Patrol_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_SendGoal_Response
    std::shared_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_SendGoal_Response
    std::shared_ptr<turtlebot3_msgs::action::Patrol_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Patrol_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Patrol_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Patrol_SendGoal_Response_

// alias to use template instance with default allocator
using Patrol_SendGoal_Response =
  turtlebot3_msgs::action::Patrol_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace turtlebot3_msgs

namespace turtlebot3_msgs
{

namespace action
{

struct Patrol_SendGoal
{
  using Request = turtlebot3_msgs::action::Patrol_SendGoal_Request;
  using Response = turtlebot3_msgs::action::Patrol_SendGoal_Response;
};

}  // namespace action

}  // namespace turtlebot3_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__turtlebot3_msgs__action__Patrol_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_msgs__action__Patrol_GetResult_Request __declspec(deprecated)
#endif

namespace turtlebot3_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Patrol_GetResult_Request_
{
  using Type = Patrol_GetResult_Request_<ContainerAllocator>;

  explicit Patrol_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit Patrol_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_msgs::action::Patrol_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_msgs::action::Patrol_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_GetResult_Request
    std::shared_ptr<turtlebot3_msgs::action::Patrol_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_GetResult_Request
    std::shared_ptr<turtlebot3_msgs::action::Patrol_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Patrol_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Patrol_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Patrol_GetResult_Request_

// alias to use template instance with default allocator
using Patrol_GetResult_Request =
  turtlebot3_msgs::action::Patrol_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace turtlebot3_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__turtlebot3_msgs__action__Patrol_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_msgs__action__Patrol_GetResult_Response __declspec(deprecated)
#endif

namespace turtlebot3_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Patrol_GetResult_Response_
{
  using Type = Patrol_GetResult_Response_<ContainerAllocator>;

  explicit Patrol_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit Patrol_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    turtlebot3_msgs::action::Patrol_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const turtlebot3_msgs::action::Patrol_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_msgs::action::Patrol_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_msgs::action::Patrol_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_GetResult_Response
    std::shared_ptr<turtlebot3_msgs::action::Patrol_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_GetResult_Response
    std::shared_ptr<turtlebot3_msgs::action::Patrol_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Patrol_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const Patrol_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Patrol_GetResult_Response_

// alias to use template instance with default allocator
using Patrol_GetResult_Response =
  turtlebot3_msgs::action::Patrol_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace turtlebot3_msgs

namespace turtlebot3_msgs
{

namespace action
{

struct Patrol_GetResult
{
  using Request = turtlebot3_msgs::action::Patrol_GetResult_Request;
  using Response = turtlebot3_msgs::action::Patrol_GetResult_Response;
};

}  // namespace action

}  // namespace turtlebot3_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "turtlebot3_msgs/action/detail/patrol__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__turtlebot3_msgs__action__Patrol_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_msgs__action__Patrol_FeedbackMessage __declspec(deprecated)
#endif

namespace turtlebot3_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Patrol_FeedbackMessage_
{
  using Type = Patrol_FeedbackMessage_<ContainerAllocator>;

  explicit Patrol_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit Patrol_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    turtlebot3_msgs::action::Patrol_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const turtlebot3_msgs::action::Patrol_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_msgs::action::Patrol_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_msgs::action::Patrol_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_msgs::action::Patrol_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_msgs::action::Patrol_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_msgs::action::Patrol_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_msgs::action::Patrol_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_FeedbackMessage
    std::shared_ptr<turtlebot3_msgs::action::Patrol_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_msgs__action__Patrol_FeedbackMessage
    std::shared_ptr<turtlebot3_msgs::action::Patrol_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Patrol_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const Patrol_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Patrol_FeedbackMessage_

// alias to use template instance with default allocator
using Patrol_FeedbackMessage =
  turtlebot3_msgs::action::Patrol_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace turtlebot3_msgs

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace turtlebot3_msgs
{

namespace action
{

struct Patrol
{
  /// The goal message defined in the action definition.
  using Goal = turtlebot3_msgs::action::Patrol_Goal;
  /// The result message defined in the action definition.
  using Result = turtlebot3_msgs::action::Patrol_Result;
  /// The feedback message defined in the action definition.
  using Feedback = turtlebot3_msgs::action::Patrol_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = turtlebot3_msgs::action::Patrol_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = turtlebot3_msgs::action::Patrol_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = turtlebot3_msgs::action::Patrol_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct Patrol Patrol;

}  // namespace action

}  // namespace turtlebot3_msgs

#endif  // TURTLEBOT3_MSGS__ACTION__DETAIL__PATROL__STRUCT_HPP_
