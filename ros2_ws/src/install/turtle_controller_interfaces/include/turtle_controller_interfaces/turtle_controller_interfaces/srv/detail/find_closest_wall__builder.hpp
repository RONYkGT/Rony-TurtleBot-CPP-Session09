// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtle_controller_interfaces:srv/FindClosestWall.idl
// generated code does not contain a copyright notice

#ifndef TURTLE_CONTROLLER_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_
#define TURTLE_CONTROLLER_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtle_controller_interfaces/srv/detail/find_closest_wall__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtle_controller_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtle_controller_interfaces::srv::FindClosestWall_Request>()
{
  return ::turtle_controller_interfaces::srv::FindClosestWall_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace turtle_controller_interfaces


namespace turtle_controller_interfaces
{

namespace srv
{

namespace builder
{

class Init_FindClosestWall_Response_angle
{
public:
  explicit Init_FindClosestWall_Response_angle(::turtle_controller_interfaces::srv::FindClosestWall_Response & msg)
  : msg_(msg)
  {}
  ::turtle_controller_interfaces::srv::FindClosestWall_Response angle(::turtle_controller_interfaces::srv::FindClosestWall_Response::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtle_controller_interfaces::srv::FindClosestWall_Response msg_;
};

class Init_FindClosestWall_Response_success
{
public:
  Init_FindClosestWall_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindClosestWall_Response_angle success(::turtle_controller_interfaces::srv::FindClosestWall_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_FindClosestWall_Response_angle(msg_);
  }

private:
  ::turtle_controller_interfaces::srv::FindClosestWall_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtle_controller_interfaces::srv::FindClosestWall_Response>()
{
  return turtle_controller_interfaces::srv::builder::Init_FindClosestWall_Response_success();
}

}  // namespace turtle_controller_interfaces

#endif  // TURTLE_CONTROLLER_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_
