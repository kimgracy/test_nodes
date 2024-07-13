// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_bboxes_msg:msg/YoloObstacle.idl
// generated code does not contain a copyright notice

#ifndef MY_BBOXES_MSG__MSG__DETAIL__YOLO_OBSTACLE__STRUCT_HPP_
#define MY_BBOXES_MSG__MSG__DETAIL__YOLO_OBSTACLE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__my_bboxes_msg__msg__YoloObstacle __attribute__((deprecated))
#else
# define DEPRECATED__my_bboxes_msg__msg__YoloObstacle __declspec(deprecated)
#endif

namespace my_bboxes_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct YoloObstacle_
{
  using Type = YoloObstacle_<ContainerAllocator>;

  explicit YoloObstacle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->label = "";
      this->x = 0.0f;
      this->y = 0.0f;
    }
  }

  explicit YoloObstacle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : label(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->label = "";
      this->x = 0.0f;
      this->y = 0.0f;
    }
  }

  // field types and members
  using _label_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _label_type label;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;

  // setters for named parameter idiom
  Type & set__label(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->label = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_bboxes_msg::msg::YoloObstacle_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_bboxes_msg::msg::YoloObstacle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_bboxes_msg::msg::YoloObstacle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_bboxes_msg::msg::YoloObstacle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_bboxes_msg::msg::YoloObstacle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_bboxes_msg::msg::YoloObstacle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_bboxes_msg::msg::YoloObstacle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_bboxes_msg::msg::YoloObstacle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_bboxes_msg::msg::YoloObstacle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_bboxes_msg::msg::YoloObstacle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_bboxes_msg__msg__YoloObstacle
    std::shared_ptr<my_bboxes_msg::msg::YoloObstacle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_bboxes_msg__msg__YoloObstacle
    std::shared_ptr<my_bboxes_msg::msg::YoloObstacle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const YoloObstacle_ & other) const
  {
    if (this->label != other.label) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const YoloObstacle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct YoloObstacle_

// alias to use template instance with default allocator
using YoloObstacle =
  my_bboxes_msg::msg::YoloObstacle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_bboxes_msg

#endif  // MY_BBOXES_MSG__MSG__DETAIL__YOLO_OBSTACLE__STRUCT_HPP_
