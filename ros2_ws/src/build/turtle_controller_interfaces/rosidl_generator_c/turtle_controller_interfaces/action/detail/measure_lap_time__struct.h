// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtle_controller_interfaces:action/MeasureLapTime.idl
// generated code does not contain a copyright notice

#ifndef TURTLE_CONTROLLER_INTERFACES__ACTION__DETAIL__MEASURE_LAP_TIME__STRUCT_H_
#define TURTLE_CONTROLLER_INTERFACES__ACTION__DETAIL__MEASURE_LAP_TIME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/MeasureLapTime in the package turtle_controller_interfaces.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_Goal
{
  uint8_t structure_needs_at_least_one_member;
} turtle_controller_interfaces__action__MeasureLapTime_Goal;

// Struct for a sequence of turtle_controller_interfaces__action__MeasureLapTime_Goal.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_Goal__Sequence
{
  turtle_controller_interfaces__action__MeasureLapTime_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtle_controller_interfaces__action__MeasureLapTime_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/MeasureLapTime in the package turtle_controller_interfaces.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_Result
{
  float total_time;
} turtle_controller_interfaces__action__MeasureLapTime_Result;

// Struct for a sequence of turtle_controller_interfaces__action__MeasureLapTime_Result.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_Result__Sequence
{
  turtle_controller_interfaces__action__MeasureLapTime_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtle_controller_interfaces__action__MeasureLapTime_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/MeasureLapTime in the package turtle_controller_interfaces.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_Feedback
{
  float elapsed_time;
} turtle_controller_interfaces__action__MeasureLapTime_Feedback;

// Struct for a sequence of turtle_controller_interfaces__action__MeasureLapTime_Feedback.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_Feedback__Sequence
{
  turtle_controller_interfaces__action__MeasureLapTime_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtle_controller_interfaces__action__MeasureLapTime_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "turtle_controller_interfaces/action/detail/measure_lap_time__struct.h"

/// Struct defined in action/MeasureLapTime in the package turtle_controller_interfaces.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  turtle_controller_interfaces__action__MeasureLapTime_Goal goal;
} turtle_controller_interfaces__action__MeasureLapTime_SendGoal_Request;

// Struct for a sequence of turtle_controller_interfaces__action__MeasureLapTime_SendGoal_Request.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_SendGoal_Request__Sequence
{
  turtle_controller_interfaces__action__MeasureLapTime_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtle_controller_interfaces__action__MeasureLapTime_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/MeasureLapTime in the package turtle_controller_interfaces.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} turtle_controller_interfaces__action__MeasureLapTime_SendGoal_Response;

// Struct for a sequence of turtle_controller_interfaces__action__MeasureLapTime_SendGoal_Response.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_SendGoal_Response__Sequence
{
  turtle_controller_interfaces__action__MeasureLapTime_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtle_controller_interfaces__action__MeasureLapTime_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/MeasureLapTime in the package turtle_controller_interfaces.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} turtle_controller_interfaces__action__MeasureLapTime_GetResult_Request;

// Struct for a sequence of turtle_controller_interfaces__action__MeasureLapTime_GetResult_Request.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_GetResult_Request__Sequence
{
  turtle_controller_interfaces__action__MeasureLapTime_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtle_controller_interfaces__action__MeasureLapTime_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "turtle_controller_interfaces/action/detail/measure_lap_time__struct.h"

/// Struct defined in action/MeasureLapTime in the package turtle_controller_interfaces.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_GetResult_Response
{
  int8_t status;
  turtle_controller_interfaces__action__MeasureLapTime_Result result;
} turtle_controller_interfaces__action__MeasureLapTime_GetResult_Response;

// Struct for a sequence of turtle_controller_interfaces__action__MeasureLapTime_GetResult_Response.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_GetResult_Response__Sequence
{
  turtle_controller_interfaces__action__MeasureLapTime_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtle_controller_interfaces__action__MeasureLapTime_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "turtle_controller_interfaces/action/detail/measure_lap_time__struct.h"

/// Struct defined in action/MeasureLapTime in the package turtle_controller_interfaces.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  turtle_controller_interfaces__action__MeasureLapTime_Feedback feedback;
} turtle_controller_interfaces__action__MeasureLapTime_FeedbackMessage;

// Struct for a sequence of turtle_controller_interfaces__action__MeasureLapTime_FeedbackMessage.
typedef struct turtle_controller_interfaces__action__MeasureLapTime_FeedbackMessage__Sequence
{
  turtle_controller_interfaces__action__MeasureLapTime_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtle_controller_interfaces__action__MeasureLapTime_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLE_CONTROLLER_INTERFACES__ACTION__DETAIL__MEASURE_LAP_TIME__STRUCT_H_
