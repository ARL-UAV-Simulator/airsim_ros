// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hrlsim_interfaces:action/MoveToLocation.idl
// generated code does not contain a copyright notice
#include "hrlsim_interfaces/action/detail/move_to_location__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
hrlsim_interfaces__action__MoveToLocation_Goal__init(hrlsim_interfaces__action__MoveToLocation_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // target
  // timeout
  // tolerance
  // speed
  // position_frame
  // fvel
  // facc
  // fjrk
  // yaw_frame
  // yaw
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_Goal__fini(hrlsim_interfaces__action__MoveToLocation_Goal * msg)
{
  if (!msg) {
    return;
  }
  // target
  // timeout
  // tolerance
  // speed
  // position_frame
  // fvel
  // facc
  // fjrk
  // yaw_frame
  // yaw
}

hrlsim_interfaces__action__MoveToLocation_Goal *
hrlsim_interfaces__action__MoveToLocation_Goal__create()
{
  hrlsim_interfaces__action__MoveToLocation_Goal * msg = (hrlsim_interfaces__action__MoveToLocation_Goal *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_Goal));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hrlsim_interfaces__action__MoveToLocation_Goal));
  bool success = hrlsim_interfaces__action__MoveToLocation_Goal__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hrlsim_interfaces__action__MoveToLocation_Goal__destroy(hrlsim_interfaces__action__MoveToLocation_Goal * msg)
{
  if (msg) {
    hrlsim_interfaces__action__MoveToLocation_Goal__fini(msg);
  }
  free(msg);
}


bool
hrlsim_interfaces__action__MoveToLocation_Goal__Sequence__init(hrlsim_interfaces__action__MoveToLocation_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hrlsim_interfaces__action__MoveToLocation_Goal * data = NULL;
  if (size) {
    data = (hrlsim_interfaces__action__MoveToLocation_Goal *)calloc(size, sizeof(hrlsim_interfaces__action__MoveToLocation_Goal));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hrlsim_interfaces__action__MoveToLocation_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hrlsim_interfaces__action__MoveToLocation_Goal__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_Goal__Sequence__fini(hrlsim_interfaces__action__MoveToLocation_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hrlsim_interfaces__action__MoveToLocation_Goal__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hrlsim_interfaces__action__MoveToLocation_Goal__Sequence *
hrlsim_interfaces__action__MoveToLocation_Goal__Sequence__create(size_t size)
{
  hrlsim_interfaces__action__MoveToLocation_Goal__Sequence * array = (hrlsim_interfaces__action__MoveToLocation_Goal__Sequence *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_Goal__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hrlsim_interfaces__action__MoveToLocation_Goal__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hrlsim_interfaces__action__MoveToLocation_Goal__Sequence__destroy(hrlsim_interfaces__action__MoveToLocation_Goal__Sequence * array)
{
  if (array) {
    hrlsim_interfaces__action__MoveToLocation_Goal__Sequence__fini(array);
  }
  free(array);
}


bool
hrlsim_interfaces__action__MoveToLocation_Result__init(hrlsim_interfaces__action__MoveToLocation_Result * msg)
{
  if (!msg) {
    return false;
  }
  // location
  // error
  // time_left
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_Result__fini(hrlsim_interfaces__action__MoveToLocation_Result * msg)
{
  if (!msg) {
    return;
  }
  // location
  // error
  // time_left
}

hrlsim_interfaces__action__MoveToLocation_Result *
hrlsim_interfaces__action__MoveToLocation_Result__create()
{
  hrlsim_interfaces__action__MoveToLocation_Result * msg = (hrlsim_interfaces__action__MoveToLocation_Result *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_Result));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hrlsim_interfaces__action__MoveToLocation_Result));
  bool success = hrlsim_interfaces__action__MoveToLocation_Result__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hrlsim_interfaces__action__MoveToLocation_Result__destroy(hrlsim_interfaces__action__MoveToLocation_Result * msg)
{
  if (msg) {
    hrlsim_interfaces__action__MoveToLocation_Result__fini(msg);
  }
  free(msg);
}


bool
hrlsim_interfaces__action__MoveToLocation_Result__Sequence__init(hrlsim_interfaces__action__MoveToLocation_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hrlsim_interfaces__action__MoveToLocation_Result * data = NULL;
  if (size) {
    data = (hrlsim_interfaces__action__MoveToLocation_Result *)calloc(size, sizeof(hrlsim_interfaces__action__MoveToLocation_Result));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hrlsim_interfaces__action__MoveToLocation_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hrlsim_interfaces__action__MoveToLocation_Result__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_Result__Sequence__fini(hrlsim_interfaces__action__MoveToLocation_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hrlsim_interfaces__action__MoveToLocation_Result__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hrlsim_interfaces__action__MoveToLocation_Result__Sequence *
hrlsim_interfaces__action__MoveToLocation_Result__Sequence__create(size_t size)
{
  hrlsim_interfaces__action__MoveToLocation_Result__Sequence * array = (hrlsim_interfaces__action__MoveToLocation_Result__Sequence *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_Result__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hrlsim_interfaces__action__MoveToLocation_Result__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hrlsim_interfaces__action__MoveToLocation_Result__Sequence__destroy(hrlsim_interfaces__action__MoveToLocation_Result__Sequence * array)
{
  if (array) {
    hrlsim_interfaces__action__MoveToLocation_Result__Sequence__fini(array);
  }
  free(array);
}


bool
hrlsim_interfaces__action__MoveToLocation_Feedback__init(hrlsim_interfaces__action__MoveToLocation_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // location
  // error
  // time_left
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_Feedback__fini(hrlsim_interfaces__action__MoveToLocation_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // location
  // error
  // time_left
}

hrlsim_interfaces__action__MoveToLocation_Feedback *
hrlsim_interfaces__action__MoveToLocation_Feedback__create()
{
  hrlsim_interfaces__action__MoveToLocation_Feedback * msg = (hrlsim_interfaces__action__MoveToLocation_Feedback *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_Feedback));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hrlsim_interfaces__action__MoveToLocation_Feedback));
  bool success = hrlsim_interfaces__action__MoveToLocation_Feedback__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hrlsim_interfaces__action__MoveToLocation_Feedback__destroy(hrlsim_interfaces__action__MoveToLocation_Feedback * msg)
{
  if (msg) {
    hrlsim_interfaces__action__MoveToLocation_Feedback__fini(msg);
  }
  free(msg);
}


bool
hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence__init(hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hrlsim_interfaces__action__MoveToLocation_Feedback * data = NULL;
  if (size) {
    data = (hrlsim_interfaces__action__MoveToLocation_Feedback *)calloc(size, sizeof(hrlsim_interfaces__action__MoveToLocation_Feedback));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hrlsim_interfaces__action__MoveToLocation_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hrlsim_interfaces__action__MoveToLocation_Feedback__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence__fini(hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hrlsim_interfaces__action__MoveToLocation_Feedback__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence *
hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence__create(size_t size)
{
  hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence * array = (hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence__destroy(hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence * array)
{
  if (array) {
    hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__functions.h"

bool
hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__init(hrlsim_interfaces__action__MoveToLocation_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!hrlsim_interfaces__action__MoveToLocation_Goal__init(&msg->goal)) {
    hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__fini(hrlsim_interfaces__action__MoveToLocation_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  hrlsim_interfaces__action__MoveToLocation_Goal__fini(&msg->goal);
}

hrlsim_interfaces__action__MoveToLocation_SendGoal_Request *
hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__create()
{
  hrlsim_interfaces__action__MoveToLocation_SendGoal_Request * msg = (hrlsim_interfaces__action__MoveToLocation_SendGoal_Request *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_SendGoal_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hrlsim_interfaces__action__MoveToLocation_SendGoal_Request));
  bool success = hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__destroy(hrlsim_interfaces__action__MoveToLocation_SendGoal_Request * msg)
{
  if (msg) {
    hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__fini(msg);
  }
  free(msg);
}


bool
hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence__init(hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hrlsim_interfaces__action__MoveToLocation_SendGoal_Request * data = NULL;
  if (size) {
    data = (hrlsim_interfaces__action__MoveToLocation_SendGoal_Request *)calloc(size, sizeof(hrlsim_interfaces__action__MoveToLocation_SendGoal_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence__fini(hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence *
hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence__create(size_t size)
{
  hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence * array = (hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence__destroy(hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence * array)
{
  if (array) {
    hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__init(hrlsim_interfaces__action__MoveToLocation_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__fini(hrlsim_interfaces__action__MoveToLocation_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

hrlsim_interfaces__action__MoveToLocation_SendGoal_Response *
hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__create()
{
  hrlsim_interfaces__action__MoveToLocation_SendGoal_Response * msg = (hrlsim_interfaces__action__MoveToLocation_SendGoal_Response *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_SendGoal_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hrlsim_interfaces__action__MoveToLocation_SendGoal_Response));
  bool success = hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__destroy(hrlsim_interfaces__action__MoveToLocation_SendGoal_Response * msg)
{
  if (msg) {
    hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__fini(msg);
  }
  free(msg);
}


bool
hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence__init(hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hrlsim_interfaces__action__MoveToLocation_SendGoal_Response * data = NULL;
  if (size) {
    data = (hrlsim_interfaces__action__MoveToLocation_SendGoal_Response *)calloc(size, sizeof(hrlsim_interfaces__action__MoveToLocation_SendGoal_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence__fini(hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence *
hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence__create(size_t size)
{
  hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence * array = (hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence__destroy(hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence * array)
{
  if (array) {
    hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
hrlsim_interfaces__action__MoveToLocation_GetResult_Request__init(hrlsim_interfaces__action__MoveToLocation_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    hrlsim_interfaces__action__MoveToLocation_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_GetResult_Request__fini(hrlsim_interfaces__action__MoveToLocation_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

hrlsim_interfaces__action__MoveToLocation_GetResult_Request *
hrlsim_interfaces__action__MoveToLocation_GetResult_Request__create()
{
  hrlsim_interfaces__action__MoveToLocation_GetResult_Request * msg = (hrlsim_interfaces__action__MoveToLocation_GetResult_Request *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_GetResult_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hrlsim_interfaces__action__MoveToLocation_GetResult_Request));
  bool success = hrlsim_interfaces__action__MoveToLocation_GetResult_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hrlsim_interfaces__action__MoveToLocation_GetResult_Request__destroy(hrlsim_interfaces__action__MoveToLocation_GetResult_Request * msg)
{
  if (msg) {
    hrlsim_interfaces__action__MoveToLocation_GetResult_Request__fini(msg);
  }
  free(msg);
}


bool
hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence__init(hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hrlsim_interfaces__action__MoveToLocation_GetResult_Request * data = NULL;
  if (size) {
    data = (hrlsim_interfaces__action__MoveToLocation_GetResult_Request *)calloc(size, sizeof(hrlsim_interfaces__action__MoveToLocation_GetResult_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hrlsim_interfaces__action__MoveToLocation_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hrlsim_interfaces__action__MoveToLocation_GetResult_Request__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence__fini(hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hrlsim_interfaces__action__MoveToLocation_GetResult_Request__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence *
hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence__create(size_t size)
{
  hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence * array = (hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence__destroy(hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence * array)
{
  if (array) {
    hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `result`
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__functions.h"

bool
hrlsim_interfaces__action__MoveToLocation_GetResult_Response__init(hrlsim_interfaces__action__MoveToLocation_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!hrlsim_interfaces__action__MoveToLocation_Result__init(&msg->result)) {
    hrlsim_interfaces__action__MoveToLocation_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_GetResult_Response__fini(hrlsim_interfaces__action__MoveToLocation_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  hrlsim_interfaces__action__MoveToLocation_Result__fini(&msg->result);
}

hrlsim_interfaces__action__MoveToLocation_GetResult_Response *
hrlsim_interfaces__action__MoveToLocation_GetResult_Response__create()
{
  hrlsim_interfaces__action__MoveToLocation_GetResult_Response * msg = (hrlsim_interfaces__action__MoveToLocation_GetResult_Response *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_GetResult_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hrlsim_interfaces__action__MoveToLocation_GetResult_Response));
  bool success = hrlsim_interfaces__action__MoveToLocation_GetResult_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hrlsim_interfaces__action__MoveToLocation_GetResult_Response__destroy(hrlsim_interfaces__action__MoveToLocation_GetResult_Response * msg)
{
  if (msg) {
    hrlsim_interfaces__action__MoveToLocation_GetResult_Response__fini(msg);
  }
  free(msg);
}


bool
hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence__init(hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hrlsim_interfaces__action__MoveToLocation_GetResult_Response * data = NULL;
  if (size) {
    data = (hrlsim_interfaces__action__MoveToLocation_GetResult_Response *)calloc(size, sizeof(hrlsim_interfaces__action__MoveToLocation_GetResult_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hrlsim_interfaces__action__MoveToLocation_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hrlsim_interfaces__action__MoveToLocation_GetResult_Response__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence__fini(hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hrlsim_interfaces__action__MoveToLocation_GetResult_Response__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence *
hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence__create(size_t size)
{
  hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence * array = (hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence__destroy(hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence * array)
{
  if (array) {
    hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__functions.h"

bool
hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__init(hrlsim_interfaces__action__MoveToLocation_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!hrlsim_interfaces__action__MoveToLocation_Feedback__init(&msg->feedback)) {
    hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__fini(hrlsim_interfaces__action__MoveToLocation_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  hrlsim_interfaces__action__MoveToLocation_Feedback__fini(&msg->feedback);
}

hrlsim_interfaces__action__MoveToLocation_FeedbackMessage *
hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__create()
{
  hrlsim_interfaces__action__MoveToLocation_FeedbackMessage * msg = (hrlsim_interfaces__action__MoveToLocation_FeedbackMessage *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_FeedbackMessage));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hrlsim_interfaces__action__MoveToLocation_FeedbackMessage));
  bool success = hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__destroy(hrlsim_interfaces__action__MoveToLocation_FeedbackMessage * msg)
{
  if (msg) {
    hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__fini(msg);
  }
  free(msg);
}


bool
hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence__init(hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hrlsim_interfaces__action__MoveToLocation_FeedbackMessage * data = NULL;
  if (size) {
    data = (hrlsim_interfaces__action__MoveToLocation_FeedbackMessage *)calloc(size, sizeof(hrlsim_interfaces__action__MoveToLocation_FeedbackMessage));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence__fini(hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence *
hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence__create(size_t size)
{
  hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence * array = (hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence *)malloc(sizeof(hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence__destroy(hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence * array)
{
  if (array) {
    hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence__fini(array);
  }
  free(array);
}
