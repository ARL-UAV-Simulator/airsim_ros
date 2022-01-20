// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hrlsim_interfaces:msg/Multirotor.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hrlsim_interfaces/msg/detail/multirotor__rosidl_typesupport_introspection_c.h"
#include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hrlsim_interfaces/msg/detail/multirotor__functions.h"
#include "hrlsim_interfaces/msg/detail/multirotor__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `state`
#include "hrlsim_interfaces/msg/state.h"
// Member `state`
#include "hrlsim_interfaces/msg/detail/state__rosidl_typesupport_introspection_c.h"
// Member `odom`
#include "geometry_msgs/msg/pose.h"
// Member `odom`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `looptime`
#include "std_msgs/msg/float32.h"
// Member `looptime`
#include "std_msgs/msg/detail/float32__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Multirotor__rosidl_typesupport_introspection_c__Multirotor_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__msg__Multirotor__init(message_memory);
}

void Multirotor__rosidl_typesupport_introspection_c__Multirotor_fini_function(void * message_memory)
{
  hrlsim_interfaces__msg__Multirotor__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Multirotor__rosidl_typesupport_introspection_c__Multirotor_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__msg__Multirotor, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__msg__Multirotor, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "odom",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__msg__Multirotor, odom),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "looptime",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__msg__Multirotor, looptime),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Multirotor__rosidl_typesupport_introspection_c__Multirotor_message_members = {
  "hrlsim_interfaces__msg",  // message namespace
  "Multirotor",  // message name
  4,  // number of fields
  sizeof(hrlsim_interfaces__msg__Multirotor),
  Multirotor__rosidl_typesupport_introspection_c__Multirotor_message_member_array,  // message members
  Multirotor__rosidl_typesupport_introspection_c__Multirotor_init_function,  // function to initialize message memory (memory has to be allocated)
  Multirotor__rosidl_typesupport_introspection_c__Multirotor_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Multirotor__rosidl_typesupport_introspection_c__Multirotor_message_type_support_handle = {
  0,
  &Multirotor__rosidl_typesupport_introspection_c__Multirotor_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, msg, Multirotor)() {
  Multirotor__rosidl_typesupport_introspection_c__Multirotor_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  Multirotor__rosidl_typesupport_introspection_c__Multirotor_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, msg, State)();
  Multirotor__rosidl_typesupport_introspection_c__Multirotor_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  Multirotor__rosidl_typesupport_introspection_c__Multirotor_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float32)();
  if (!Multirotor__rosidl_typesupport_introspection_c__Multirotor_message_type_support_handle.typesupport_identifier) {
    Multirotor__rosidl_typesupport_introspection_c__Multirotor_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Multirotor__rosidl_typesupport_introspection_c__Multirotor_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
