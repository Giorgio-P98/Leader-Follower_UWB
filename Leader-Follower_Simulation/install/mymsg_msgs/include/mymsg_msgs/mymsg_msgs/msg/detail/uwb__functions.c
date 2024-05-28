// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mymsg_msgs:msg/Uwb.idl
// generated code does not contain a copyright notice
#include "mymsg_msgs/msg/detail/uwb__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
mymsg_msgs__msg__Uwb__init(mymsg_msgs__msg__Uwb * msg)
{
  if (!msg) {
    return false;
  }
  // range
  // aoa
  return true;
}

void
mymsg_msgs__msg__Uwb__fini(mymsg_msgs__msg__Uwb * msg)
{
  if (!msg) {
    return;
  }
  // range
  // aoa
}

bool
mymsg_msgs__msg__Uwb__are_equal(const mymsg_msgs__msg__Uwb * lhs, const mymsg_msgs__msg__Uwb * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // range
  if (lhs->range != rhs->range) {
    return false;
  }
  // aoa
  if (lhs->aoa != rhs->aoa) {
    return false;
  }
  return true;
}

bool
mymsg_msgs__msg__Uwb__copy(
  const mymsg_msgs__msg__Uwb * input,
  mymsg_msgs__msg__Uwb * output)
{
  if (!input || !output) {
    return false;
  }
  // range
  output->range = input->range;
  // aoa
  output->aoa = input->aoa;
  return true;
}

mymsg_msgs__msg__Uwb *
mymsg_msgs__msg__Uwb__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mymsg_msgs__msg__Uwb * msg = (mymsg_msgs__msg__Uwb *)allocator.allocate(sizeof(mymsg_msgs__msg__Uwb), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mymsg_msgs__msg__Uwb));
  bool success = mymsg_msgs__msg__Uwb__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mymsg_msgs__msg__Uwb__destroy(mymsg_msgs__msg__Uwb * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mymsg_msgs__msg__Uwb__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mymsg_msgs__msg__Uwb__Sequence__init(mymsg_msgs__msg__Uwb__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mymsg_msgs__msg__Uwb * data = NULL;

  if (size) {
    data = (mymsg_msgs__msg__Uwb *)allocator.zero_allocate(size, sizeof(mymsg_msgs__msg__Uwb), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mymsg_msgs__msg__Uwb__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mymsg_msgs__msg__Uwb__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mymsg_msgs__msg__Uwb__Sequence__fini(mymsg_msgs__msg__Uwb__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mymsg_msgs__msg__Uwb__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mymsg_msgs__msg__Uwb__Sequence *
mymsg_msgs__msg__Uwb__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mymsg_msgs__msg__Uwb__Sequence * array = (mymsg_msgs__msg__Uwb__Sequence *)allocator.allocate(sizeof(mymsg_msgs__msg__Uwb__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mymsg_msgs__msg__Uwb__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mymsg_msgs__msg__Uwb__Sequence__destroy(mymsg_msgs__msg__Uwb__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mymsg_msgs__msg__Uwb__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mymsg_msgs__msg__Uwb__Sequence__are_equal(const mymsg_msgs__msg__Uwb__Sequence * lhs, const mymsg_msgs__msg__Uwb__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mymsg_msgs__msg__Uwb__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mymsg_msgs__msg__Uwb__Sequence__copy(
  const mymsg_msgs__msg__Uwb__Sequence * input,
  mymsg_msgs__msg__Uwb__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mymsg_msgs__msg__Uwb);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mymsg_msgs__msg__Uwb * data =
      (mymsg_msgs__msg__Uwb *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mymsg_msgs__msg__Uwb__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mymsg_msgs__msg__Uwb__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mymsg_msgs__msg__Uwb__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
