/* Includes ------------------------------------------------------------------*/

#include <stddef.h>
#include <string.h>

#include "eeprom.hpp"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

uint16_t EEPROM::getStatusCode() {
  uint32_t u32_flashStatus;

  u32_flashStatus = flash_get_status_flags();
  if (u32_flashStatus == FLASH_SR_EOP) {
    return FLASH_COMPLETE;
  }

  if (u32_flashStatus & FLASH_SR_WRPRTERR) {
    return FLASH_WRITE_PROTECTION_ERROR;
  }
  if (u32_flashStatus & FLASH_SR_PGERR) {
    return FLASH_PROGRAMMING_ERROR;
  }
  if (u32_flashStatus & FLASH_SR_BSY) {
    return FLASH_BUSY;
  }

  return FLASH_UNKNOWN_ERROR;
}

/*----------------------------------------------------------------------------*/
/*
 * Reformat a bucket by erasing all its flash pages. The flash should be
 * unlocked before calling this method.
 */

uint16_t EEPROM::bucketErase(void *pv_bucket) {
  uint16_t u16_status;
  uint8_t *pu8_page;
  uint8_t u8_page;

  u16_status = FLASH_COMPLETE;
  pu8_page = (uint8_t *) pv_bucket;
  for (u8_page = 0; u8_page < _u8_bucketPages; u8_page++) {
    flash_erase_page((uint32_t) pu8_page);
    if ((u16_status = getStatusCode()) != FLASH_COMPLETE) {
      break;
    }
    pu8_page += FLASH_PAGE_SIZE;
  }

  return u16_status;
}

/*----------------------------------------------------------------------------*/
/**
 * @brief  Restore the pages to a known good state in case of page's status
 *   corruption after a power loss.
 * @param  None.
 * @retval - Flash error code: on write Flash error
 *         - FLASH_COMPLETE: on success
 */
uint16_t EEPROM::init(void *pv_base, uint8_t u8_buckets,
                      uint8_t u8_bucketPages) {
  uint32_t u32_bucketSize;
  uint16_t u16_bucketStatus;
  uint16_t u16;
  uint8_t u8;
  void *pv_receiveData;
  void *pv_bucketEnd;
  void *pv_bucket;
  FlashLock o_lock;

  _pv_base = pv_base;
  _u8_buckets = u8_buckets;
  _u8_bucketPages = u8_bucketPages;
  _pv_activeBucket = NULL;

  u32_bucketSize = FLASH_PAGE_SIZE * _u8_bucketPages;

  o_lock.unlock();

  /*
   * Verify status of all buckets and determine which bucket is VALID.
   * If illegal bucket status is found or if several buckets are in VALID state,
   * re-format the flash.
   */
  pv_bucket = pv_base;
  pv_receiveData = NULL;
  for (u8 = 0; u8 < u8_buckets; u8++) {
    u16_bucketStatus = *((uint16_t *) pv_bucket);
    if (u16_bucketStatus == VALID_BUCKET) {
      if (_pv_activeBucket != NULL) {
        return format();
      }
      _pv_activeBucket = pv_bucket;
    } else if (u16_bucketStatus == RECEIVE_DATA) {
      pv_receiveData = pv_bucket;
    } else if (u16_bucketStatus != ERASED) {
      return format();
    }
    pv_bucket = ((uint8_t *) pv_bucket) + u32_bucketSize;
  }

  // if no valid bucket found, re-format the flash
  if (_pv_activeBucket == NULL) {
    if (pv_receiveData != NULL) {
      // a bucket transfer was initiated but was interrupted before changing
      // the bucket status.
      _pv_activeBucket = pv_receiveData;
      flash_program_half_word((uint32_t) _pv_activeBucket, VALID_BUCKET);
    } else {
      return format();
    }
  } else if (pv_receiveData != NULL) {
    // transfer was in progress when poweroff, so reformat the bucket
    u16 = bucketErase(pv_receiveData);
    if (u16 != FLASH_COMPLETE) {
      return u16;
    }
  }

  // search for the last used word and set _pu8_stack
  _pu8_stack = ((uint8_t *) _pv_activeBucket) + EEPROM_HEADER_SIZE;
  pv_bucketEnd = ((uint8_t *) _pv_activeBucket) + u32_bucketSize;
  while (_pu8_stack < pv_bucketEnd) {
    u16 = *((uint16_t *) _pu8_stack);
    if (u16 != 0xFFFF) {
      break;
    }
    _pu8_stack += 2;
  }

  o_lock.lock();

  return FLASH_COMPLETE;
}

/*----------------------------------------------------------------------------*/
/**
 * @brief  Clear all values in the EEPROM by formatting it.
 * @param  None.
 * @retval - Flash error code: on write Flash error
 *         - FLASH_COMPLETE: on success
 */
uint16_t EEPROM::clear() {
  uint16_t u16_status;
  FlashLock o_lock;

  o_lock.unlock();
  u16_status = format();
  o_lock.lock();

  return u16_status;
}

/*----------------------------------------------------------------------------*/
/**
 * @brief  Returns the last stored variable data, if found, which correspond to
 *   the passed key
 * @param  pv_key: pointer to key value
 * @param  u8_keySize: length of the key
 * @param  pv_data: pointer to area to store the value associate to the key
 * @param  pu8_dataSize: pointer to data area size
 * @retval Success true else false if not found
 */
bool EEPROM::readVariable(void *pv_key, uint8_t u8_keySize, void *pv_data,
                          uint8_t *pu8_dataSize) {
  void *pv_bucketEnd;
  uint8_t *pu8;
  uint8_t u8_kSize;
  uint8_t u8_dSize;

  pv_bucketEnd =
      ((uint8_t *) _pv_activeBucket) + FLASH_PAGE_SIZE * _u8_bucketPages;
  pu8 = _pu8_stack;
  while (pu8 < pv_bucketEnd) {
    u8_kSize = *pu8++;
    u8_dSize = *pu8++;
    if (u8_kSize == u8_keySize && memcmp(pu8, pv_key, u8_kSize) == 0) {
      if (*pu8_dataSize < u8_dSize) {
        return false;
      }
      *pu8_dataSize = u8_dSize;
      memcpy(pv_data, pu8 + ROUND_WORD(u8_kSize), u8_dSize);
      return true;
    }
    pu8 += ROUND_WORD(u8_kSize) + ROUND_WORD(u8_dSize);
  }

  return false;
}

/*----------------------------------------------------------------------------*/
/**
 * @brief  Writes/updates variable data in EEPROM.
 * @param  VirtAddress: Variable virtual address
 * @param  Data: 16 bit data to be written
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
uint16_t EEPROM::writeVariable(void *pv_key, uint8_t u8_keySize, void *pv_data,
                               uint8_t u8_dataSize) {
  void *pv_bucketBody;
  uint8_t *pu8_dst;
  uint16_t u16_status;
  uint16_t u16;
  FlashLock o_lock;

  o_lock.unlock();

  pv_bucketBody = ((uint8_t *) _pv_activeBucket) + EEPROM_HEADER_SIZE;
  pu8_dst = _pu8_stack - (2 + ROUND_WORD(u8_keySize) + ROUND_WORD(u8_dataSize));
  if (pu8_dst < pv_bucketBody) {
    u16_status = bucketTransfer(pv_key, u8_keySize);
    if (u16_status != FLASH_COMPLETE) {
      return u16_status;
    }
    pu8_dst =
        _pu8_stack - (2 + ROUND_WORD(u8_keySize) + ROUND_WORD(u8_dataSize));
  }

  _pu8_stack = pu8_dst;
  u16 = (u8_dataSize << 8) + u8_keySize;

  flash_program_half_word((uint32_t) pu8_dst, u16);
  u16_status = getStatusCode();
  if (u16_status == FLASH_COMPLETE) {
    pu8_dst += 2;

    // transfer key
    copy(pu8_dst, pv_key, u8_keySize);
    pu8_dst += ROUND_WORD(u8_keySize);

    // transfer data
    copy(pu8_dst, pv_data, u8_dataSize);
  }

  o_lock.lock();

  return u16_status;
}

/*----------------------------------------------------------------------------*/
/*
 * Used internally to transfer a memory zone to EEPROM.
 */
void EEPROM::copy(void *pv_dst, const void *pv_src, uint8_t u8_size) {
  uint16_t *pu16_dst;
  uint16_t *pu16_src;

  pu16_dst = (uint16_t *) pv_dst;
  pu16_src = (uint16_t *) pv_src;
  for (;;) {
    flash_program_half_word((uint32_t) pu16_dst, *pu16_src);
    if (u8_size <= 2) {
      break;
    }
    pu16_dst++;
    pu16_src++;
    u8_size -= 2;
  }
}

/*----------------------------------------------------------------------------*/
/**
 * @brief  Erases all pages and writes VALID_PAGE header to first page
 * @param  None
 * @retval Status of the last operation (Flash write or erase) done during
 *         EEPROM formating
 */
uint16_t EEPROM::format(void) {
  uint16_t u16_status;
  uint8_t *pu8_page;
  uint8_t u8_bucket;
  uint8_t u8_page;

  _pu8_stack = ((uint8_t *) _pv_base) + _u8_bucketPages * FLASH_PAGE_SIZE;
  _pv_activeBucket = _pv_base;

  pu8_page = (uint8_t *) _pv_base;
  for (u8_bucket = 0; u8_bucket < _u8_buckets; u8_bucket++) {
    for (u8_page = 0; u8_page < _u8_bucketPages; u8_page++) {
      flash_erase_page((uint32_t) pu8_page);
      u16_status = getStatusCode();
      if (u16_status != FLASH_COMPLETE) {
        return u16_status;
      }
      pu8_page += FLASH_PAGE_SIZE;
    }
  }

  flash_program_half_word((uint32_t) _pv_activeBucket, VALID_BUCKET);
  u16_status = getStatusCode();

  return u16_status;
}

/*----------------------------------------------------------------------------*/
/**
 * @brief  Transfers last updated variables data from the full Page to
 *   an empty one.
 * @param  pv_exceptKey: pointer to the key value which shouldn't be transfered
 * @param  u8_keySize: key size
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
uint16_t EEPROM::bucketTransfer(void *pv_exceptKey, uint8_t u8_keySize) {
  uint8_t *pu8_loopStack;
  uint8_t *pu8_src;
  uint16_t u16_status;
  uint16_t u16_itemSize;
  uint32_t u32_bucketSize;
  void *pv_nextBucketEnd;
  void *pv_nextBucket;
  void *pv_bucketEnd;
  uint8_t u8_kSizeTmp;
  uint8_t u8_dSizeTmp;
  uint8_t u8_kSize;
  uint8_t u8_dSize;

  u32_bucketSize = FLASH_PAGE_SIZE * _u8_bucketPages;

  pv_nextBucket = ((uint8_t *) _pv_activeBucket) + u32_bucketSize;
  if (pv_nextBucket >= ((uint8_t *) _pv_base) + u32_bucketSize * _u8_buckets) {
    pv_nextBucket = _pv_base;
  }

  flash_program_half_word((uint32_t) pv_nextBucket, RECEIVE_DATA);
  u16_status = getStatusCode();
  if (u16_status != FLASH_COMPLETE) {
    return u16_status;
  }

  pv_nextBucketEnd = ((uint8_t *) pv_nextBucket) + u32_bucketSize;
  pv_bucketEnd = ((uint8_t *) _pv_activeBucket) + u32_bucketSize;

  pu8_src = _pu8_stack;
  _pu8_stack = (uint8_t *) pv_nextBucketEnd;

  while (pu8_src < pv_bucketEnd) {
    u8_kSize = *pu8_src++;
    u8_dSize = *pu8_src++;
    u16_itemSize = ROUND_WORD(u8_kSize) + ROUND_WORD(u8_dSize);

    if (u8_keySize != u8_kSize ||
        memcmp(pu8_src, pv_exceptKey, u8_keySize) != 0) {
      pu8_loopStack = _pu8_stack;
      while (pu8_loopStack < pv_nextBucketEnd) {
        u8_kSizeTmp = *pu8_loopStack++;
        u8_dSizeTmp = *pu8_loopStack++;
        if (u8_kSizeTmp == u8_kSize &&
            memcmp(pu8_loopStack, pu8_src, u8_kSize) == 0) {
          goto nextVariable;
        }
        pu8_loopStack += ROUND_WORD(u8_kSizeTmp) + ROUND_WORD(u8_dSizeTmp);
      }

      _pu8_stack -= u16_itemSize + 2;
      copy(_pu8_stack, pu8_src - 2, u16_itemSize + 2);
    }
  nextVariable:
    pu8_src += u16_itemSize;
  }

  u16_status = bucketErase(_pv_activeBucket);
  if (u16_status != FLASH_COMPLETE) {
    return u16_status;
  }

  _pv_activeBucket = pv_nextBucket;

  flash_program_half_word((uint32_t) _pv_activeBucket, VALID_BUCKET);
  u16_status = getStatusCode();

  return u16_status;
}

/*----------------------------------------------------------------------------*/
/**
 * @brief  Transfers last updated variables data from the full Page to
 *   an empty one.
 * @param  pv_exceptKey: pointer to the key value which shouldn't be transfered
 * @param  u8_keySize: key size
 * @retval Success or error status:
 *           - FLASH_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
void EEPROM::getNext(void **ppv_handle, void **ppv_key, uint8_t *pu8_keySize,
                     void **ppv_data, uint8_t *pu8_dataSize) {
  uint8_t *pu8_loopStack;
  uint8_t *pu8_handle;
  void *pv_bucketEnd;
  void *pv_handle;
  uint8_t u8_kSizeTmp;
  uint8_t u8_dSizeTmp;
  uint8_t u8_kSize;
  uint8_t u8_dSize;
  bool b_duplicate;

  pv_bucketEnd =
      ((uint8_t *) _pv_activeBucket) + FLASH_PAGE_SIZE * _u8_bucketPages;

  pu8_handle = (uint8_t *) *ppv_handle;
  if (pu8_handle == NULL) {
    // starts from top of stack
    *ppv_handle = pu8_handle = _pu8_stack;
  } else {
    // skips current item
    u8_kSize = *pu8_handle++;
    u8_dSize = *pu8_handle++;
    pu8_handle += ROUND_WORD(u8_kSize) + ROUND_WORD(u8_dSize);
  }

  do {
    if (pu8_handle >= pv_bucketEnd) {
      *ppv_handle = NULL;
      return;
    }

    pv_handle = pu8_handle;
    u8_kSize = *pu8_handle++;
    u8_dSize = *pu8_handle++;

    b_duplicate = false;
    pu8_loopStack = _pu8_stack;
    while (pu8_loopStack < pv_handle) {
      u8_kSizeTmp = *pu8_loopStack++;
      u8_dSizeTmp = *pu8_loopStack++;
      if (u8_kSizeTmp == u8_kSize &&
          memcmp(pu8_loopStack, pu8_handle, u8_kSize) == 0) {
        pu8_handle += ROUND_WORD(u8_kSize) + ROUND_WORD(u8_dSize);
        b_duplicate = true;
        break;
      }
      pu8_loopStack += ROUND_WORD(u8_kSizeTmp) + ROUND_WORD(u8_dSizeTmp);
    }
  } while (b_duplicate);

  *pu8_keySize = u8_kSize;
  *pu8_dataSize = u8_dSize;
  *ppv_key = pu8_handle;
  *ppv_data = pu8_handle + ROUND_WORD(u8_kSize);
  *ppv_handle = pv_handle;
}

/***********************************END OF FILE********************************/
