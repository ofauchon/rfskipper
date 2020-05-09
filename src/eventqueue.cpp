/*
 * eventqueue.cpp
 *
 *  Created on: 22 Oct 2019
 *      Author: Pierre
 */

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "standard.hpp"
#include "eventqueue.hpp"

/*----------------------------------------------------------------------------*/

#define HEAP_PARENT(x) ((int) ((x) -1) / 2)
#define HEAP_LEFT(x) ((int) ((x) *2) + 1)
#define HEAP_RIGHT(x) ((int) ((x) *2) + 2)

/*----------------------------------------------------------------------------*/

uint8_t EventQueue::get() {
  uint8_t u8_swapEvent;
  uint8_t u8_event;
  int32_t i_right;
  int32_t i_left;
  int32_t i_node;
  int32_t i_swap;

  if (isEmpty()) {
    return -1;
  }

  ENTER_CRITICAL();

  u8_event = _pu8_heap[0];

  if (_u8_size > 1) {
    _pu8_heap[0] = _pu8_heap[--_u8_size];
    i_node = 0;

    for (;;) {
      i_left = HEAP_LEFT(i_node);
      if (i_left < _u8_size && _pu8_heap[i_left] > _pu8_heap[i_node]) {
        i_swap = i_left;
      } else {
        i_swap = i_node;
      }

      i_right = HEAP_RIGHT(i_node);
      if (i_right < _u8_size && _pu8_heap[i_right] > _pu8_heap[i_swap]) {
        i_swap = i_right;
      }

      if (i_swap == i_node) {
        break;
      }

      u8_swapEvent = _pu8_heap[i_swap];
      _pu8_heap[i_swap] = _pu8_heap[i_node];
      _pu8_heap[i_node] = u8_swapEvent;
      i_node = i_swap;
    }
  } else {
    _u8_size = 0;
  }

  EXIT_CRITICAL();

  return u8_event;
}

/*----------------------------------------------------------------------------*/

int EventQueue::put(uint8_t u8_event, uint8_t u8_priority) {
  int32_t i_parent;
  int32_t i_node;
  uint8_t u8_swap;

  ENTER_CRITICAL();

  i_node = _u8_size;
  i_parent = HEAP_PARENT(i_node);

  _pu8_heap[_u8_size++] = u8_event;

  while (i_node > 0 && _pu8_heap[i_parent] < _pu8_heap[i_node]) {
    u8_swap = _pu8_heap[i_parent];
    _pu8_heap[i_parent] = _pu8_heap[i_node];
    _pu8_heap[i_node] = u8_swap;

    i_node = i_parent;
    i_parent = HEAP_PARENT(i_node);
  }

  EXIT_CRITICAL();

  return 0;
}

/*----------------------------------------------------------------------------*/
