#ifndef RING_H
#define RING_H 100

#include <stdint.h>

/*----------------------------------------------------------------------------*/

template<int SIZE>
class RING {
private:
   uint8_t _pu8_buffer[SIZE];
   uint8_t *_pu8_begin=0;
   uint8_t *_pu8_end=0;
   uint8_t *_pu8_read=0;
   uint8_t *_pu8_write=0;

public:
//   void init(uint8_t *pu8_buffer, int i_size);
   RING();
   int writeByte(uint8_t c);
   int write(uint8_t *pu8_data, int i_size);
   int readByte();
   int read(uint8_t *pu8_data, int i_size);
   inline int hasNext() {
      return _pu8_write != _pu8_read;
   }
};

/*----------------------------------------------------------------------------*/

#endif
