// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtle_controller_interfaces:srv/FindClosestWall.idl
// generated code does not contain a copyright notice

#ifndef TURTLE_CONTROLLER_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__TRAITS_HPP_
#define TURTLE_CONTROLLER_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtle_controller_interfaces/srv/detail/find_closest_wall__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turtle_controller_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const FindClosestWall_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindClosestWall_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindClosestWall_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtle_controller_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtle_controller_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtle_controller_interfaces::srv::FindClosestWall_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtle_controller_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtle_controller_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtle_controller_interfaces::srv::FindClosestWall_Request & msg)
{
  return turtle_controller_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtle_controller_interfaces::srv::FindClosestWall_Request>()
{
  return "turtle_controller_interfaces::srv::FindClosestWall_Request";
}

template<>
inline const char * name<turtle_controller_interfaces::srv::FindClosestWall_Request>()
{
  return "turtle_controller_interfaces/srv/FindClosestWall_Request";
}

template<>
struct has_fixed_size<turtle_controller_interfaces::srv::FindClosestWall_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtle_controller_interfaces::srv::FindClosestWall_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtle_controller_interfaces::srv::FindClosestWall_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace turtle_controller_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const FindClosestWall_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FindClosestWall_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FindClosestWall_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtle_controller_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtle_controller_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtle_controller_interfaces::srv::FindClosestWall_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtle_controller_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtle_controller_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtle_controller_interfaces::srv::FindClosestWall_Response & msg)
{
  return turtle_controller_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtle_controller_interfaces::srv::FindClosestWall_Response>()
{
  return "turtle_controller_interfaces::srv::FindClosestWall_Response";
}

template<>
inline const char * name<turtle_controller_interfaces::srv::FindClosestWall_Response>()
{
  return "turtle_controller_interfaces/srv/FindClosestWall_Response";
}

template<>
struct has_fixed_size<turtle_controller_interfaces::srv::FindClosestWall_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtle_controller_interfaces::srv::FindClosestWall_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtle_controller_interfaces::srv::FindClosestWall_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtle_controller_interfaces::srv::FindClosestWall>()
{
  return "turtle_controller_interfaces::srv::FindClosestWall";
}

template<>
inline const char * name<turtle_controller_interfaces::srv::FindClosestWall>()
{
  return "turtle_controller_interfaces/srv/FindClosestWall";
}

template<>
struct has_fixed_size<turtle_controller_interfaces::srv::FindClosestWall>
  : std::integral_constant<
    bool,
    has_fixed_size<turtle_controller_interfaces::srv::FindClosestWall_Request>::value &&
    has_fixed_size<turtle_controller_interfaces::srv::FindClosestWall_Response>::value
  >
{
};

template<>
struct has_bounded_size<turtle_controller_interfaces::srv::FindClosestWall>
  : std::integral_constant<
    bool,
    has_bounded_size<turtle_controller_interfaces::srv::FindClosestWall_Request>::value &&
    has_bounded_size<turtle_controller_interfaces::srv::FindClosestWall_Response>::value
  >
{
};

template<>
struct is_service<turtle_controller_interfaces::srv::FindClosestWall>
  : std::true_type
{
};

template<>
struct is_service_request<turtle_controller_interfaces::srv::FindClosestWall_Request>
  : std::true_type
{
};

template<>
struct is_service_response<turtle_controller_interfaces::srv::FindClosestWall_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TURTLE_CONTROLLER_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__TRAITS_HPP_
