// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_bboxes_msg:msg/VehiclePhase.idl
// generated code does not contain a copyright notice

#ifndef MY_BBOXES_MSG__MSG__DETAIL__VEHICLE_PHASE__STRUCT_HPP_
#define MY_BBOXES_MSG__MSG__DETAIL__VEHICLE_PHASE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__my_bboxes_msg__msg__VehiclePhase __attribute__((deprecated))
#else
# define DEPRECATED__my_bboxes_msg__msg__VehiclePhase __declspec(deprecated)
#endif

namespace my_bboxes_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VehiclePhase_
{
  using Type = VehiclePhase_<ContainerAllocator>;

  explicit VehiclePhase_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->phase = "";
    }
  }

  explicit VehiclePhase_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : phase(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->phase = "";
    }
  }

  // field types and members
  using _phase_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _phase_type phase;

  // setters for named parameter idiom
  Type & set__phase(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->phase = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_bboxes_msg::msg::VehiclePhase_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_bboxes_msg::msg::VehiclePhase_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_bboxes_msg::msg::VehiclePhase_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_bboxes_msg::msg::VehiclePhase_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_bboxes_msg::msg::VehiclePhase_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_bboxes_msg::msg::VehiclePhase_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_bboxes_msg::msg::VehiclePhase_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_bboxes_msg::msg::VehiclePhase_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_bboxes_msg::msg::VehiclePhase_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_bboxes_msg::msg::VehiclePhase_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_bboxes_msg__msg__VehiclePhase
    std::shared_ptr<my_bboxes_msg::msg::VehiclePhase_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_bboxes_msg__msg__VehiclePhase
    std::shared_ptr<my_bboxes_msg::msg::VehiclePhase_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VehiclePhase_ & other) const
  {
    if (this->phase != other.phase) {
      return false;
    }
    return true;
  }
  bool operator!=(const VehiclePhase_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VehiclePhase_

// alias to use template instance with default allocator
using VehiclePhase =
  my_bboxes_msg::msg::VehiclePhase_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_bboxes_msg

#endif  // MY_BBOXES_MSG__MSG__DETAIL__VEHICLE_PHASE__STRUCT_HPP_
