/*
 * ring.C
 *
 *  Created on: 22 Oct 2019
 *      Author: Pierre
 */

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "ring.hpp"

/*----------------------------------------------------------------------------*/

template <int SIZE> RING<SIZE>::RING() {
  _pu8_begin = _pu8_read = _pu8_write = _pu8_buffer;
  _pu8_end = _pu8_buffer + SIZE;
}

/*----------------------------------------------------------------------------*/

template <int SIZE> int RING<SIZE>::writeByte(uint8_t c) {
  uint8_t *pu8_next;

  pu8_next = _pu8_write;
  *pu8_next++ = c;
  if (pu8_next == _pu8_end) {
    pu8_next = _pu8_begin;
  }

  if (pu8_next == _pu8_read) {
    return -1;
  }

  _pu8_write = pu8_next;

  return c;
}

/*----------------------------------------------------------------------------*/

template <int SIZE> int RING<SIZE>::write(uint8_t *pu8_data, int i_size) {
  int32_t i;

  for (i = 0; i < i_size; i++) {
    if (writeByte(pu8_data[i]) < 0) {
      return -i;
    }
  }

  return i;
}

/*----------------------------------------------------------------------------*/

template <int SIZE> int RING<SIZE>::readByte() {
  int c;

  if (_pu8_write == _pu8_read) {
    return -1;
  }

  c = *_pu8_read++;

  if (_pu8_read == _pu8_end) {
    _pu8_read = _pu8_begin;
  }

  return c;
}

/*----------------------------------------------------------------------------*/

template <int SIZE> int RING<SIZE>::read(uint8_t *pu8_data, int i_size) {
  int32_t i;

  for (i = 0; i < i_size; i++) {
    if ((pu8_data[i] = readByte()) < 0) {
      return i;
    }
  }

  return -i;
}

/*----------------------------------------------------------------------------*/

template class RING<60>;
template class RING<128>;
template class RING<256>;

/*----------------------------------------------------------------------------*/
