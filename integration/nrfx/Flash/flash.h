/*
 ******************************************************************************
 * @file    flash.h
 * @author  SNode Firmware Team
 * @version V1.0.0
 * @date    16-Sep-2021
 * @brief   This file contains all the functions prototypes for interfacing 
            external QSPI based Flash memory. 
 *******************************************************************************
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef QSPI_H
#define QSPI_H

#ifdef __cplusplus
 extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "SNode_bus.h"
#include "snode_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"




/** @defgroup ExternalFlash_Exported_Constants 
 * @{
 */

/* Macros Defintions ---------------------------------------------------------*/
#define FLASH_OK             0
#define FLASH_ERROR         -1

#define QSPI_STD_CMD_WRSR   0x01
#define QSPI_STD_CMD_RSTEN  0x66
#define QSPI_STD_CMD_RST    0x99

/* Flash Variables --------------------------------------------------------- */
/* Flash Trigger Status */
typedef enum
{
  DISABLE,
  ENABLE,  
}flash_status_t;




/** @addtogroup ExternalFlash_Exported_Functions 
 * @{
 */
uint32_t SNode_Configure_Flash_Memory(void);
uint32_t SNode_Flash_Init(void);
uint32_t SNode_Flash_DeInit(void);
uint32_t SNode_Flash_Erase(nrf_qspi_erase_len_t len,uint32_t start_address);
uint32_t SNode_Flash_Write(void const * p_tx_buffer,size_t tx_buffer_length,uint32_t dst_address);
uint32_t SNode_Flash_Read(void * p_rx_buffer,size_t rx_buffer_length,uint32_t src_address);
void Trigger_Flash_State(bool status);



/** @addtogroup ExternalFlash_Exported_Variables 
 * @{
 */



#ifdef __cplusplus
}
#endif

#endif /*QSPI_H*/




/***************** (C) COPYRIGHT DotCom IoT *****END OF FILE*************************/
