// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from turtle_controller_interfaces:srv/FindClosestWall.idl
// generated code does not contain a copyright notice

#ifndef TURTLE_CONTROLLER_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__FUNCTIONS_H_
#define TURTLE_CONTROLLER_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "turtle_controller_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "turtle_controller_interfaces/srv/detail/find_closest_wall__struct.h"

/// Initialize srv/FindClosestWall message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * turtle_controller_interfaces__srv__FindClosestWall_Request
 * )) before or use
 * turtle_controller_interfaces__srv__FindClosestWall_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
bool
turtle_controller_interfaces__srv__FindClosestWall_Request__init(turtle_controller_interfaces__srv__FindClosestWall_Request * msg);

/// Finalize srv/FindClosestWall message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
void
turtle_controller_interfaces__srv__FindClosestWall_Request__fini(turtle_controller_interfaces__srv__FindClosestWall_Request * msg);

/// Create srv/FindClosestWall message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * turtle_controller_interfaces__srv__FindClosestWall_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
turtle_controller_interfaces__srv__FindClosestWall_Request *
turtle_controller_interfaces__srv__FindClosestWall_Request__create();

/// Destroy srv/FindClosestWall message.
/**
 * It calls
 * turtle_controller_interfaces__srv__FindClosestWall_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
void
turtle_controller_interfaces__srv__FindClosestWall_Request__destroy(turtle_controller_interfaces__srv__FindClosestWall_Request * msg);

/// Check for srv/FindClosestWall message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
bool
turtle_controller_interfaces__srv__FindClosestWall_Request__are_equal(const turtle_controller_interfaces__srv__FindClosestWall_Request * lhs, const turtle_controller_interfaces__srv__FindClosestWall_Request * rhs);

/// Copy a srv/FindClosestWall message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
bool
turtle_controller_interfaces__srv__FindClosestWall_Request__copy(
  const turtle_controller_interfaces__srv__FindClosestWall_Request * input,
  turtle_controller_interfaces__srv__FindClosestWall_Request * output);

/// Initialize array of srv/FindClosestWall messages.
/**
 * It allocates the memory for the number of elements and calls
 * turtle_controller_interfaces__srv__FindClosestWall_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
bool
turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence__init(turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence * array, size_t size);

/// Finalize array of srv/FindClosestWall messages.
/**
 * It calls
 * turtle_controller_interfaces__srv__FindClosestWall_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
void
turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence__fini(turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence * array);

/// Create array of srv/FindClosestWall messages.
/**
 * It allocates the memory for the array and calls
 * turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence *
turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence__create(size_t size);

/// Destroy array of srv/FindClosestWall messages.
/**
 * It calls
 * turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
void
turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence__destroy(turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence * array);

/// Check for srv/FindClosestWall message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
bool
turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence__are_equal(const turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence * lhs, const turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence * rhs);

/// Copy an array of srv/FindClosestWall messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
bool
turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence__copy(
  const turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence * input,
  turtle_controller_interfaces__srv__FindClosestWall_Request__Sequence * output);

/// Initialize srv/FindClosestWall message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * turtle_controller_interfaces__srv__FindClosestWall_Response
 * )) before or use
 * turtle_controller_interfaces__srv__FindClosestWall_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
bool
turtle_controller_interfaces__srv__FindClosestWall_Response__init(turtle_controller_interfaces__srv__FindClosestWall_Response * msg);

/// Finalize srv/FindClosestWall message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
void
turtle_controller_interfaces__srv__FindClosestWall_Response__fini(turtle_controller_interfaces__srv__FindClosestWall_Response * msg);

/// Create srv/FindClosestWall message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * turtle_controller_interfaces__srv__FindClosestWall_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
turtle_controller_interfaces__srv__FindClosestWall_Response *
turtle_controller_interfaces__srv__FindClosestWall_Response__create();

/// Destroy srv/FindClosestWall message.
/**
 * It calls
 * turtle_controller_interfaces__srv__FindClosestWall_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
void
turtle_controller_interfaces__srv__FindClosestWall_Response__destroy(turtle_controller_interfaces__srv__FindClosestWall_Response * msg);

/// Check for srv/FindClosestWall message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
bool
turtle_controller_interfaces__srv__FindClosestWall_Response__are_equal(const turtle_controller_interfaces__srv__FindClosestWall_Response * lhs, const turtle_controller_interfaces__srv__FindClosestWall_Response * rhs);

/// Copy a srv/FindClosestWall message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
bool
turtle_controller_interfaces__srv__FindClosestWall_Response__copy(
  const turtle_controller_interfaces__srv__FindClosestWall_Response * input,
  turtle_controller_interfaces__srv__FindClosestWall_Response * output);

/// Initialize array of srv/FindClosestWall messages.
/**
 * It allocates the memory for the number of elements and calls
 * turtle_controller_interfaces__srv__FindClosestWall_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
bool
turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence__init(turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence * array, size_t size);

/// Finalize array of srv/FindClosestWall messages.
/**
 * It calls
 * turtle_controller_interfaces__srv__FindClosestWall_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
void
turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence__fini(turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence * array);

/// Create array of srv/FindClosestWall messages.
/**
 * It allocates the memory for the array and calls
 * turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence *
turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence__create(size_t size);

/// Destroy array of srv/FindClosestWall messages.
/**
 * It calls
 * turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
void
turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence__destroy(turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence * array);

/// Check for srv/FindClosestWall message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
bool
turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence__are_equal(const turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence * lhs, const turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence * rhs);

/// Copy an array of srv/FindClosestWall messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_turtle_controller_interfaces
bool
turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence__copy(
  const turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence * input,
  turtle_controller_interfaces__srv__FindClosestWall_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TURTLE_CONTROLLER_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__FUNCTIONS_H_
