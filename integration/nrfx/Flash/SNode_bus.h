/**
******************************************************************************
* @file    SNode_bus.h
* @author  SNode Firmware Team
* @version V1.0.0
* @date    11-Sep-2021
* @brief   header file for the BSP BUS IO driver
******************************************************************************
*/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SNODE_BUS_H
#define __SNODE_BUS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "snode_gpio.h"
#include "SNodeConfig.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "SNode.errno.h"

#if (SNODE_I2C_ENABLE == 1)
#include "nrf_drv_twi.h"
#endif

#if (SNODE_QSPI_ENABLE == 1)
#include "nrf_drv_qspi.h"
#endif


/* Macros Defintions ---------------------------------------------------------*/
#if (SNODE_I2C_ENABLE == 1)

#define TWI_ADDRESSES      127

#if TWI0_ENABLED
#define TWI0_INSTANCE_ID     0
#endif

#if TWI1_ENABLED
#define TWI1_INSTANCE_ID     1
#endif

#endif // I2C_ENABLE


#if (SNODE_QSPI_ENABLE == 1)
#define QSPI_OK               0
#define QSPI_ERROR           -1

#define WAIT_FOR_PERIPH() do { \
        while (!m_finished) {} \
        m_finished = false;    \
    } while (0)

#endif  // SNODE_QSPI_ENABLE


/* Function Declaration ---------------------------------------------------------*/

#if (SNODE_I2C_ENABLE == 1)

int32_t BSP_I2C0_Init (void);
int32_t BSP_I2C0_DeInit(void);;
int32_t BSP_I2C1_Init (void);
int32_t BSP_I2C1_DeInit(void);
int32_t BSP_I2C_Disable(nrf_drv_twi_t const *handler);
int32_t BSP_I2C_Enable(nrf_drv_twi_t const *handler);

int32_t BSP_I2C0_WriteReg(uint8_t slaveaddr, uint8_t regaddr, const uint8_t *bufp,uint16_t len);
int32_t BSP_I2C0_ReadReg(uint8_t slaveaddr, uint8_t regaddr, uint8_t *bufp, uint16_t len);
int32_t BSP_I2C1_ReadReg(uint16_t SlaveAddr, uint16_t RegAddr, uint8_t *bufp, uint16_t len);
int32_t BSP_I2C1_WriteReg(uint16_t SlaveAddr, uint16_t Reg, uint8_t *pData, uint16_t len);
int32_t BSP_I2C1_WriteReg_16(uint8_t slaveAddr, uint16_t registerAddr, uint8_t *pdata, uint32_t len);
int32_t BSP_I2C1_ReadReg_16(uint8_t slaveAddr, uint16_t registerAddr, uint8_t *pdata, uint32_t len);
int32_t BSP_I2C1_WriteReg_32(uint8_t slaveAddr, uint16_t registerAddr, uint8_t *pdata, uint32_t len);
int32_t BSP_I2C1_ReadReg_32(uint8_t slaveAddr, uint16_t registerAddr, uint8_t *pdata, uint32_t len);
void BSP_I2C_Scanner(nrf_drv_twi_t const *handler);

#endif // I2C_ENABLE


#if (SNODE_QSPI_ENABLE == 1)

uint32_t BSP_QSPI_Init(void);
uint32_t BSP_QSPI_DeInit(void);
uint32_t BSP_QSPI_Erase(nrf_qspi_erase_len_t len,uint32_t start_address);
uint32_t BSP_QSPI_Write(void const * p_tx_buffer,size_t tx_buffer_length,uint32_t dst_address);
uint32_t BSP_QSPI_Read(void * p_rx_buffer,size_t rx_buffer_length,uint32_t src_address);
uint32_t BSP_QSPI_Configure_Memory(nrf_qspi_cinstr_conf_t const *p_config,void const * p_tx_buffer, void * p_rx_buffer);

#endif // SNODE_QSPI_ENABLE 


/** @addtogroup SNodeBus_Exported_Variables I2C Exported Variables
 * @{
 */

#if (SNODE_I2C_ENABLE == 1)
extern const nrf_drv_twi_t m_twi0;
extern const nrf_drv_twi_t m_twi1;
#endif 

#if (SNODE_QSPI_ENABLE == 1)
extern volatile bool m_finished;
#endif 


#ifdef __cplusplus
}
#endif

#endif  // __SNODE_BUS_H

/***************** (C) COPYRIGHT DotCom IoT ***** END OF FILE *************************/