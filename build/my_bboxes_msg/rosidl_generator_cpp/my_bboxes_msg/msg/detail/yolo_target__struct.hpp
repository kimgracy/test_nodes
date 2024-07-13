// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_bboxes_msg:msg/YoloTarget.idl
// generated code does not contain a copyright notice

#ifndef MY_BBOXES_MSG__MSG__DETAIL__YOLO_TARGET__STRUCT_HPP_
#define MY_BBOXES_MSG__MSG__DETAIL__YOLO_TARGET__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__my_bboxes_msg__msg__YoloTarget __attribute__((deprecated))
#else
# define DEPRECATED__my_bboxes_msg__msg__YoloTarget __declspec(deprecated)
#endif

namespace my_bboxes_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct YoloTarget_
{
  using Type = YoloTarget_<ContainerAllocator>;

  explicit YoloTarget_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image(_init)
  {
    (void)_init;
  }

  explicit YoloTarget_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _image_type image;

  // setters for named parameter idiom
  Type & set__image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->image = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_bboxes_msg::msg::YoloTarget_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_bboxes_msg::msg::YoloTarget_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_bboxes_msg::msg::YoloTarget_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_bboxes_msg::msg::YoloTarget_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_bboxes_msg::msg::YoloTarget_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_bboxes_msg::msg::YoloTarget_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_bboxes_msg::msg::YoloTarget_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_bboxes_msg::msg::YoloTarget_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_bboxes_msg::msg::YoloTarget_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_bboxes_msg::msg::YoloTarget_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_bboxes_msg__msg__YoloTarget
    std::shared_ptr<my_bboxes_msg::msg::YoloTarget_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_bboxes_msg__msg__YoloTarget
    std::shared_ptr<my_bboxes_msg::msg::YoloTarget_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const YoloTarget_ & other) const
  {
    if (this->image != other.image) {
      return false;
    }
    return true;
  }
  bool operator!=(const YoloTarget_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct YoloTarget_

// alias to use template instance with default allocator
using YoloTarget =
  my_bboxes_msg::msg::YoloTarget_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_bboxes_msg

#endif  // MY_BBOXES_MSG__MSG__DETAIL__YOLO_TARGET__STRUCT_HPP_
