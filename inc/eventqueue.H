#ifndef EVENTQUEUE_H
#define EVENTQUEUE_H 100

#include <stdint.h>

/*----------------------------------------------------------------------------*/

typedef struct _Event {

} Event;

/*----------------------------------------------------------------------------*/

class EventQueue {
private:
   uint8_t _u8_size;
   uint8_t _u8_maxSize;
   uint8_t _pu8_heap[32];

public:
   inline void init(uint8_t u8_maxSize = 32) {
      _u8_maxSize = u8_maxSize;
      _u8_size = 0;
   }
   inline bool isEmpty() {
      return _u8_size == 0;
   }
   int put(uint8_t u8_event, uint8_t u8_priority);
   uint8_t get();
};

/*----------------------------------------------------------------------------*/

#endif
