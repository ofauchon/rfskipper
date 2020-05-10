#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stddef.h>

#include <libopencm3/stm32/flash.h>

/* Exported constants --------------------------------------------------------*/
/* Define the STM32F10Xxx Flash page size depending on the used STM32 device */
/* EEPROM start address in Flash after 64KByte of used Flash memory */
#define EEPROM_START_ADDRESS ((uint32_t) 0x08010000)

/* No valid page define */
#define NO_VALID_PAGE ((uint16_t) 0x00AB)

/*
 * Page status definitions;
 * ERASED: empty bucket
 * VALID_BUCKET: bucket contains valid data
 * RECEIVE_DATA: bucket is marked to receive data
 */
#define ERASED ((uint16_t) 0xFFFF)
#define RECEIVE_DATA ((uint16_t) 0xEEEE)
#define VALID_BUCKET ((uint16_t) 0x0000)

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE ((uint8_t) 0x00)
#define WRITE_IN_VALID_PAGE ((uint8_t) 0x01)

/* Page full define */
#define PAGE_FULL ((uint8_t) 0x80)

#define FLASH_COMPLETE 0
#define FLASH_BUSY 1
#define FLASH_WRITE_PROTECTION_ERROR 2
#define FLASH_PROGRAMMING_ERROR 3
#define FLASH_UNKNOWN_ERROR 4

#define EEPROM_HEADER_SIZE 2

/* Exported types ------------------------------------------------------------*/

typedef uint16_t FlashStatus;

/* Exported macro ------------------------------------------------------------*/

#define ROUND_WORD(x) ((x + 1) & ~0x01)

/* Exported functions ------------------------------------------------------- */

#define FLASH_START_ADDRESS                                                    \
  ((uint32_t) 0x08010000) /* EEPROM emulation start address:                   \
                         after 64KByte of used Flash memory */
/* Define the STM32F10Xxx Flash page size depending on the used STM32 device */
#define STM32F10X_MD
#if defined(STM32F10X_LD) || defined(STM32F10X_MD)
#define FLASH_PAGE_SIZE (uint16_t) 0x400 /* Page size = 1KByte */
#elif defined(STM32F10X_HD) || defined(STM32F10X_CL)
#define FLASH_PAGE_SIZE (uint16_t) 0x800 /* Page size = 2KByte */
#endif

/*----------------------------------------------------------------------------*/

class EEPROM {
private:
  void *_pv_activeBucket;
  void *_pv_base;
  uint8_t *_pu8_stack;
  uint8_t _u8_buckets;
  uint8_t _u8_bucketPages;

private:
  uint16_t format(void);
  uint16_t findValidPage(uint8_t Operation);
  uint16_t verifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data);
  uint16_t bucketTransfer(void *pv_key, uint8_t u8_keySize);
  uint16_t bucketErase(void *pv_bucket);
  uint16_t getStatusCode();
  void copy(void *pv_dst, const void *pv_src, uint8_t u8_size);

public:
  EEPROM(void *pv_base, uint8_t u8_buckets, uint8_t u8_bucketPages) {
    init(pv_base, u8_buckets, u8_bucketPages);
  }
  EEPROM() { _pv_base = NULL; }

  uint16_t init(void *pv_base, uint8_t u8_buckets, uint8_t u8_bucketPages);
  bool isInit() { return _pv_base != NULL; }
  uint16_t clear();

  bool readVariable(void *pv_key, uint8_t u8_keySize, void *pv_data,
                    uint8_t *pu8_dataSize);
  uint16_t writeVariable(void *pv_key, uint8_t u8_keySize, void *pv_data,
                         uint8_t u8_dataSize);
  void getNext(void **ppv_handle, void **ppv_key, uint8_t *pu8_keySize,
               void **ppv_data, uint8_t *pu8_dataSize);
};

class FlashLock {
private:
  uint8_t _u8_nested;

public:
  FlashLock() : _u8_nested(0) {}
  ~FlashLock() {
    if (_u8_nested > 0) {
      flash_lock();
    }
  }

  void unlock() {
    if (_u8_nested++ == 0) {
      flash_unlock();
    }
  }

  void lock() {
    if (_u8_nested > 0 && --_u8_nested == 0) {
      flash_lock();
    }
  }
};

#endif /* __EEPROM_H */
