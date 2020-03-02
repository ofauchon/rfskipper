#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

#include <libopencm3/stm32/flash.h>

/* Exported constants --------------------------------------------------------*/
/* Define the STM32F10Xxx Flash page size depending on the used STM32 device */
#define STM32F10X_MD
#if defined (STM32F10X_LD) || defined (STM32F10X_MD)
  #define PAGE_SIZE  (uint16_t)0x400  /* Page size = 1KByte */
#elif defined (STM32F10X_HD) || defined (STM32F10X_CL)
  #define PAGE_SIZE  (uint16_t)0x800  /* Page size = 2KByte */
#endif

/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS    ((uint32_t)0x08010000) /* EEPROM emulation start address:
                                                  after 64KByte of used Flash memory */

/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS      ((uint32_t)(EEPROM_START_ADDRESS + 0x000))
#define PAGE0_END_ADDRESS       ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))

#define PAGE1_BASE_ADDRESS      ((uint32_t)(EEPROM_START_ADDRESS + PAGE_SIZE))
#define PAGE1_END_ADDRESS       ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))

/* Used Flash pages for EEPROM emulation */
#define PAGE0                   ((uint16_t)0x0000)
#define PAGE1                   ((uint16_t)0x0001)

/* No valid page define */
#define NO_VALID_PAGE           ((uint16_t)0x00AB)

/* Page status definitions */
#define ERASED                  ((uint16_t)0xFFFF)     /* PAGE is empty */
#define RECEIVE_DATA            ((uint16_t)0xEEEE)     /* PAGE is marked to receive data */
#define VALID_PAGE              ((uint16_t)0x0000)     /* PAGE containing valid data */

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE    ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE     ((uint8_t)0x01)

/* Page full define */
#define PAGE_FULL               ((uint8_t)0x80)

/* Variables' number */
#define NumbOfVar               ((uint8_t)0x03)

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#define FLASH_START_ADDRESS    ((uint32_t)0x08010000) /* EEPROM emulation start address:
                                                     after 64KByte of used Flash memory */
/* Define the STM32F10Xxx Flash page size depending on the used STM32 device */
#define STM32F10X_MD
#if defined (STM32F10X_LD) || defined (STM32F10X_MD)
  #define FLASH_PAGE_SIZE  (uint16_t)0x400  /* Page size = 1KByte */
#elif defined (STM32F10X_HD) || defined (STM32F10X_CL)
  #define FLASH_PAGE_SIZE  (uint16_t)0x800  /* Page size = 2KByte */
#endif

/*----------------------------------------------------------------------------*/

//template <typename T, typename S>
class EEPROM
{
private:
   uint16_t *_pu16_activePage;
   uint16_t *_pu16_base;
   uint8_t _u8_pages;


public:
   uint16_t init(uint8_t u8_pages);

   uint16_t readVariable(uint16_t u16_key, uint16_t *pu16_value);
   uint16_t writeVariable();
};

#endif /* __EEPROM_H */
