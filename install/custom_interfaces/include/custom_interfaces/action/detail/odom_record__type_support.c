// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_interfaces:action/OdomRecord.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_interfaces/action/detail/odom_record__rosidl_typesupport_introspection_c.h"
#include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_interfaces/action/detail/odom_record__functions.h"
#include "custom_interfaces/action/detail/odom_record__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void OdomRecord_Goal__rosidl_typesupport_introspection_c__OdomRecord_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__OdomRecord_Goal__init(message_memory);
}

void OdomRecord_Goal__rosidl_typesupport_introspection_c__OdomRecord_Goal_fini_function(void * message_memory)
{
  custom_interfaces__action__OdomRecord_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember OdomRecord_Goal__rosidl_typesupport_introspection_c__OdomRecord_Goal_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__OdomRecord_Goal, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers OdomRecord_Goal__rosidl_typesupport_introspection_c__OdomRecord_Goal_message_members = {
  "custom_interfaces__action",  // message namespace
  "OdomRecord_Goal",  // message name
  1,  // number of fields
  sizeof(custom_interfaces__action__OdomRecord_Goal),
  OdomRecord_Goal__rosidl_typesupport_introspection_c__OdomRecord_Goal_message_member_array,  // message members
  OdomRecord_Goal__rosidl_typesupport_introspection_c__OdomRecord_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  OdomRecord_Goal__rosidl_typesupport_introspection_c__OdomRecord_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t OdomRecord_Goal__rosidl_typesupport_introspection_c__OdomRecord_Goal_message_type_support_handle = {
  0,
  &OdomRecord_Goal__rosidl_typesupport_introspection_c__OdomRecord_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_Goal)() {
  if (!OdomRecord_Goal__rosidl_typesupport_introspection_c__OdomRecord_Goal_message_type_support_handle.typesupport_identifier) {
    OdomRecord_Goal__rosidl_typesupport_introspection_c__OdomRecord_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &OdomRecord_Goal__rosidl_typesupport_introspection_c__OdomRecord_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/action/detail/odom_record__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__functions.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__struct.h"


// Include directives for member types
// Member `list_of_odoms`
#include "geometry_msgs/msg/point.h"
// Member `list_of_odoms`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void OdomRecord_Result__rosidl_typesupport_introspection_c__OdomRecord_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__OdomRecord_Result__init(message_memory);
}

void OdomRecord_Result__rosidl_typesupport_introspection_c__OdomRecord_Result_fini_function(void * message_memory)
{
  custom_interfaces__action__OdomRecord_Result__fini(message_memory);
}

size_t OdomRecord_Result__rosidl_typesupport_introspection_c__size_function__Point__list_of_odoms(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * OdomRecord_Result__rosidl_typesupport_introspection_c__get_const_function__Point__list_of_odoms(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * OdomRecord_Result__rosidl_typesupport_introspection_c__get_function__Point__list_of_odoms(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

bool OdomRecord_Result__rosidl_typesupport_introspection_c__resize_function__Point__list_of_odoms(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember OdomRecord_Result__rosidl_typesupport_introspection_c__OdomRecord_Result_message_member_array[1] = {
  {
    "list_of_odoms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__OdomRecord_Result, list_of_odoms),  // bytes offset in struct
    NULL,  // default value
    OdomRecord_Result__rosidl_typesupport_introspection_c__size_function__Point__list_of_odoms,  // size() function pointer
    OdomRecord_Result__rosidl_typesupport_introspection_c__get_const_function__Point__list_of_odoms,  // get_const(index) function pointer
    OdomRecord_Result__rosidl_typesupport_introspection_c__get_function__Point__list_of_odoms,  // get(index) function pointer
    OdomRecord_Result__rosidl_typesupport_introspection_c__resize_function__Point__list_of_odoms  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers OdomRecord_Result__rosidl_typesupport_introspection_c__OdomRecord_Result_message_members = {
  "custom_interfaces__action",  // message namespace
  "OdomRecord_Result",  // message name
  1,  // number of fields
  sizeof(custom_interfaces__action__OdomRecord_Result),
  OdomRecord_Result__rosidl_typesupport_introspection_c__OdomRecord_Result_message_member_array,  // message members
  OdomRecord_Result__rosidl_typesupport_introspection_c__OdomRecord_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  OdomRecord_Result__rosidl_typesupport_introspection_c__OdomRecord_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t OdomRecord_Result__rosidl_typesupport_introspection_c__OdomRecord_Result_message_type_support_handle = {
  0,
  &OdomRecord_Result__rosidl_typesupport_introspection_c__OdomRecord_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_Result)() {
  OdomRecord_Result__rosidl_typesupport_introspection_c__OdomRecord_Result_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!OdomRecord_Result__rosidl_typesupport_introspection_c__OdomRecord_Result_message_type_support_handle.typesupport_identifier) {
    OdomRecord_Result__rosidl_typesupport_introspection_c__OdomRecord_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &OdomRecord_Result__rosidl_typesupport_introspection_c__OdomRecord_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/action/detail/odom_record__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__functions.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void OdomRecord_Feedback__rosidl_typesupport_introspection_c__OdomRecord_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__OdomRecord_Feedback__init(message_memory);
}

void OdomRecord_Feedback__rosidl_typesupport_introspection_c__OdomRecord_Feedback_fini_function(void * message_memory)
{
  custom_interfaces__action__OdomRecord_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember OdomRecord_Feedback__rosidl_typesupport_introspection_c__OdomRecord_Feedback_message_member_array[1] = {
  {
    "current_total",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__OdomRecord_Feedback, current_total),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers OdomRecord_Feedback__rosidl_typesupport_introspection_c__OdomRecord_Feedback_message_members = {
  "custom_interfaces__action",  // message namespace
  "OdomRecord_Feedback",  // message name
  1,  // number of fields
  sizeof(custom_interfaces__action__OdomRecord_Feedback),
  OdomRecord_Feedback__rosidl_typesupport_introspection_c__OdomRecord_Feedback_message_member_array,  // message members
  OdomRecord_Feedback__rosidl_typesupport_introspection_c__OdomRecord_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  OdomRecord_Feedback__rosidl_typesupport_introspection_c__OdomRecord_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t OdomRecord_Feedback__rosidl_typesupport_introspection_c__OdomRecord_Feedback_message_type_support_handle = {
  0,
  &OdomRecord_Feedback__rosidl_typesupport_introspection_c__OdomRecord_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_Feedback)() {
  if (!OdomRecord_Feedback__rosidl_typesupport_introspection_c__OdomRecord_Feedback_message_type_support_handle.typesupport_identifier) {
    OdomRecord_Feedback__rosidl_typesupport_introspection_c__OdomRecord_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &OdomRecord_Feedback__rosidl_typesupport_introspection_c__OdomRecord_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/action/detail/odom_record__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__functions.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "custom_interfaces/action/odom_record.h"
// Member `goal`
// already included above
// #include "custom_interfaces/action/detail/odom_record__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void OdomRecord_SendGoal_Request__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__OdomRecord_SendGoal_Request__init(message_memory);
}

void OdomRecord_SendGoal_Request__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_fini_function(void * message_memory)
{
  custom_interfaces__action__OdomRecord_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember OdomRecord_SendGoal_Request__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__OdomRecord_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__OdomRecord_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers OdomRecord_SendGoal_Request__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_message_members = {
  "custom_interfaces__action",  // message namespace
  "OdomRecord_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(custom_interfaces__action__OdomRecord_SendGoal_Request),
  OdomRecord_SendGoal_Request__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_message_member_array,  // message members
  OdomRecord_SendGoal_Request__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  OdomRecord_SendGoal_Request__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t OdomRecord_SendGoal_Request__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_message_type_support_handle = {
  0,
  &OdomRecord_SendGoal_Request__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_SendGoal_Request)() {
  OdomRecord_SendGoal_Request__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  OdomRecord_SendGoal_Request__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_Goal)();
  if (!OdomRecord_SendGoal_Request__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    OdomRecord_SendGoal_Request__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &OdomRecord_SendGoal_Request__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/action/detail/odom_record__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__functions.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void OdomRecord_SendGoal_Response__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__OdomRecord_SendGoal_Response__init(message_memory);
}

void OdomRecord_SendGoal_Response__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Response_fini_function(void * message_memory)
{
  custom_interfaces__action__OdomRecord_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember OdomRecord_SendGoal_Response__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__OdomRecord_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__OdomRecord_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers OdomRecord_SendGoal_Response__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Response_message_members = {
  "custom_interfaces__action",  // message namespace
  "OdomRecord_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(custom_interfaces__action__OdomRecord_SendGoal_Response),
  OdomRecord_SendGoal_Response__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Response_message_member_array,  // message members
  OdomRecord_SendGoal_Response__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  OdomRecord_SendGoal_Response__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t OdomRecord_SendGoal_Response__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Response_message_type_support_handle = {
  0,
  &OdomRecord_SendGoal_Response__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_SendGoal_Response)() {
  OdomRecord_SendGoal_Response__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!OdomRecord_SendGoal_Response__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    OdomRecord_SendGoal_Response__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &OdomRecord_SendGoal_Response__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_service_members = {
  "custom_interfaces__action",  // service namespace
  "OdomRecord_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_service_type_support_handle = {
  0,
  &custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_SendGoal)() {
  if (!custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_service_type_support_handle.typesupport_identifier) {
    custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_SendGoal_Response)()->data;
  }

  return &custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/action/detail/odom_record__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__functions.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void OdomRecord_GetResult_Request__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__OdomRecord_GetResult_Request__init(message_memory);
}

void OdomRecord_GetResult_Request__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Request_fini_function(void * message_memory)
{
  custom_interfaces__action__OdomRecord_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember OdomRecord_GetResult_Request__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__OdomRecord_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers OdomRecord_GetResult_Request__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Request_message_members = {
  "custom_interfaces__action",  // message namespace
  "OdomRecord_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(custom_interfaces__action__OdomRecord_GetResult_Request),
  OdomRecord_GetResult_Request__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Request_message_member_array,  // message members
  OdomRecord_GetResult_Request__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  OdomRecord_GetResult_Request__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t OdomRecord_GetResult_Request__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Request_message_type_support_handle = {
  0,
  &OdomRecord_GetResult_Request__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_GetResult_Request)() {
  OdomRecord_GetResult_Request__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!OdomRecord_GetResult_Request__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    OdomRecord_GetResult_Request__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &OdomRecord_GetResult_Request__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/action/detail/odom_record__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__functions.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "custom_interfaces/action/odom_record.h"
// Member `result`
// already included above
// #include "custom_interfaces/action/detail/odom_record__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void OdomRecord_GetResult_Response__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__OdomRecord_GetResult_Response__init(message_memory);
}

void OdomRecord_GetResult_Response__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Response_fini_function(void * message_memory)
{
  custom_interfaces__action__OdomRecord_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember OdomRecord_GetResult_Response__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__OdomRecord_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__OdomRecord_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers OdomRecord_GetResult_Response__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Response_message_members = {
  "custom_interfaces__action",  // message namespace
  "OdomRecord_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(custom_interfaces__action__OdomRecord_GetResult_Response),
  OdomRecord_GetResult_Response__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Response_message_member_array,  // message members
  OdomRecord_GetResult_Response__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  OdomRecord_GetResult_Response__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t OdomRecord_GetResult_Response__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Response_message_type_support_handle = {
  0,
  &OdomRecord_GetResult_Response__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_GetResult_Response)() {
  OdomRecord_GetResult_Response__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_Result)();
  if (!OdomRecord_GetResult_Response__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    OdomRecord_GetResult_Response__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &OdomRecord_GetResult_Response__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_GetResult_service_members = {
  "custom_interfaces__action",  // service namespace
  "OdomRecord_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_GetResult_service_type_support_handle = {
  0,
  &custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_GetResult)() {
  if (!custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_GetResult_service_type_support_handle.typesupport_identifier) {
    custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_GetResult_Response)()->data;
  }

  return &custom_interfaces__action__detail__odom_record__rosidl_typesupport_introspection_c__OdomRecord_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/action/detail/odom_record__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__functions.h"
// already included above
// #include "custom_interfaces/action/detail/odom_record__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "custom_interfaces/action/odom_record.h"
// Member `feedback`
// already included above
// #include "custom_interfaces/action/detail/odom_record__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void OdomRecord_FeedbackMessage__rosidl_typesupport_introspection_c__OdomRecord_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__action__OdomRecord_FeedbackMessage__init(message_memory);
}

void OdomRecord_FeedbackMessage__rosidl_typesupport_introspection_c__OdomRecord_FeedbackMessage_fini_function(void * message_memory)
{
  custom_interfaces__action__OdomRecord_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember OdomRecord_FeedbackMessage__rosidl_typesupport_introspection_c__OdomRecord_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__OdomRecord_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__action__OdomRecord_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers OdomRecord_FeedbackMessage__rosidl_typesupport_introspection_c__OdomRecord_FeedbackMessage_message_members = {
  "custom_interfaces__action",  // message namespace
  "OdomRecord_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(custom_interfaces__action__OdomRecord_FeedbackMessage),
  OdomRecord_FeedbackMessage__rosidl_typesupport_introspection_c__OdomRecord_FeedbackMessage_message_member_array,  // message members
  OdomRecord_FeedbackMessage__rosidl_typesupport_introspection_c__OdomRecord_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  OdomRecord_FeedbackMessage__rosidl_typesupport_introspection_c__OdomRecord_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t OdomRecord_FeedbackMessage__rosidl_typesupport_introspection_c__OdomRecord_FeedbackMessage_message_type_support_handle = {
  0,
  &OdomRecord_FeedbackMessage__rosidl_typesupport_introspection_c__OdomRecord_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_FeedbackMessage)() {
  OdomRecord_FeedbackMessage__rosidl_typesupport_introspection_c__OdomRecord_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  OdomRecord_FeedbackMessage__rosidl_typesupport_introspection_c__OdomRecord_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, action, OdomRecord_Feedback)();
  if (!OdomRecord_FeedbackMessage__rosidl_typesupport_introspection_c__OdomRecord_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    OdomRecord_FeedbackMessage__rosidl_typesupport_introspection_c__OdomRecord_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &OdomRecord_FeedbackMessage__rosidl_typesupport_introspection_c__OdomRecord_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
