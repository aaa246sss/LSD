#ifndef __FLASH_H
#define __FLASH_H 			   

#define uint8_t  unsigned char
#define uint16_t unsigned short
#define uint8  unsigned char
#define uint16 unsigned short
#define uint32  unsigned long int
#define uint32_t unsigned long int

#define FLASH_ADDR_START	0x08000000		//FLASH????
#define FLASH_PAGE_SIZE		1024			//FLASH???
#define FLASH_PAGE_COUNT	64			//FLASH???

/*******************  Bit definition for FLASH_CR register  *******************/
#define  FLASH_CR_PG                         ((uint16_t)0x0001)            /*!< Programming */
#define  FLASH_CR_PER                        ((uint16_t)0x0002)            /*!< Page Erase */
#define  FLASH_CR_MER                        ((uint16_t)0x0004)            /*!< Mass Erase */
#define  FLASH_CR_OPTPG                      ((uint16_t)0x0010)            /*!< Option Byte Programming */
#define  FLASH_CR_OPTER                      ((uint16_t)0x0020)            /*!< Option Byte Erase */
#define  FLASH_CR_STRT                       ((uint16_t)0x0040)            /*!< Start */
#define  FLASH_CR_LOCK                       ((uint16_t)0x0080)            /*!< Lock */
#define  FLASH_CR_OPTWRE                     ((uint16_t)0x0200)            /*!< Option Bytes Write Enable */
#define  FLASH_CR_ERRIE                      ((uint16_t)0x0400)            /*!< Error Interrupt Enable */
#define  FLASH_CR_EOPIE                      ((uint16_t)0x1000)            /*!< End of operation interrupt enable */

/******************  Bit definition for FLASH_SR register  *******************/
#define  FLASH_SR_BSY                        ((uint8_t)0x01)               /*!< Busy */
#define  FLASH_SR_PGERR                      ((uint8_t)0x04)               /*!< Programming Error */
#define  FLASH_SR_WRPRTERR                   ((uint8_t)0x10)               /*!< Write Protection Error */
#define  FLASH_SR_EOP                        ((uint8_t)0x20)
/*-------------------------------------------------------------------------------
 Func: FLASH²Ù×÷Ã¦ÅÐ¶Ï
 Note: return 0/OK   >0/timeout
------------------------------------------------------------------------------*/

uint8 Flash_EreasePage(uint16 PageIndex,uint16 PageCount);
uint8 Flash_WriteDatas(uint32 Addr,uint16 *Buffer,uint16 Length);
void  FLASH_ReadDatas(uint32 Addr,uint16 *Buffer,uint16 Length);
void FLASH_save(void);
#endif
