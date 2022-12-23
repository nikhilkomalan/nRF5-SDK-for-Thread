/*
 ******************************************************************************
 * @file    flash.c
 * @author  SNode Firmware Team
 * @version V1.0.0
 * @date    16-Sep-2021
 * @brief   This file provides a set of functions needed to interface the
  *         lsm6dsox sensor with SNode
 *******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include "SNodeConfig.h"
#if (SNODE_QSPI_ENABLE == 1U)

#include "flash.h"



/* Macros Defintions ---------------------------------------------------------*/


/* QSPI Flash Variables ---------------------------------------------------------*/




/* Function Definitions ---------------------------------------------------------*/

/*
 * @brief  Function to configure Flash in QSPI mode
 * @param  None      
 * @retval BSP status
*/
uint32_t SNode_Configure_Flash_Memory(void)
{
    uint8_t temporary = 0x40;
    uint32_t err_code;
    nrf_qspi_cinstr_conf_t cinstr_cfg = {
        .opcode    = QSPI_STD_CMD_RSTEN,
        .length    = NRF_QSPI_CINSTR_LEN_1B,
        .io2_level = true,
        .io3_level = true,
        .wipwait   = true,
        .wren      = true
    };

    /* Send reset enable */
    err_code = BSP_QSPI_Configure_Memory(&cinstr_cfg, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    /* Send reset command */
    cinstr_cfg.opcode = QSPI_STD_CMD_RST;
    err_code = BSP_QSPI_Configure_Memory(&cinstr_cfg, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    /* Switch to qspi mode */
    cinstr_cfg.opcode = QSPI_STD_CMD_WRSR;
    cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_2B;
    err_code = BSP_QSPI_Configure_Memory(&cinstr_cfg, &temporary, NULL);
    APP_ERROR_CHECK(err_code);

    return  err_code;
}



/*
 * @brief  Function to Initilaize External Flash Memory
 * @param  None      
 * @retval BSP status
*/
uint32_t SNode_Flash_Init(void)
{
    int32_t err_code = FLASH_OK;

    err_code = BSP_QSPI_Init();
    if(err_code != FLASH_OK)
    {
      NRF_LOG_ERROR("QSPI Flash Initilization failed\r\n");
    }

    return err_code;
}


/*
 * @brief  Function to De-Initilaize External Flash Memory
 * @param  None      
 * @retval BSP status
*/
uint32_t SNode_Flash_DeInit(void)
{
  int32_t err_code = FLASH_OK;

  BSP_QSPI_DeInit();

  return err_code;
}


/*
 * @brief  Function to erase blocks of External Flash Memory
* @param  len          Len. of Blocks: NRF_QSPI_ERASE_LEN_4KB,
                                       NRF_QSPI_ERASE_LEN_64KB,
                                       NRF_QSPI_ERASE_LEN_ALL      
 * @param  start_address  Start Address to Erase data
 * @retval BSP status
*/
uint32_t SNode_Flash_Erase(nrf_qspi_erase_len_t len,uint32_t start_address)
{
    int32_t err_code = FLASH_OK;

    err_code = BSP_QSPI_Erase(len,start_address);
    if(err_code != FLASH_OK)
    {
      NRF_LOG_ERROR("FLASH : Erase failed\r\n");
    }

    return err_code;
}


/*
 * @brief  Function to write data to External Flash Memory
 * @param  p_tx_buffer        pointer to the data to be written 
 * @param  tx_buffer_length   length of data to be written
 * @param  dst_address        Destination address to write data
 * @retval BSP status
*/
uint32_t SNode_Flash_Write(void const * p_tx_buffer,size_t tx_buffer_length,uint32_t dst_address)
{
    int32_t err_code = FLASH_OK;

    err_code = BSP_QSPI_Write(p_tx_buffer,tx_buffer_length,dst_address);
    if(err_code != FLASH_OK)
    {
       NRF_LOG_ERROR("FLASH : Flash write operation failed\r\n");
    }

    return err_code;
}



/*
 * @brief  Function to Read data from External Flash Memory
 * @param  p_rx_buffer        pointer to buffer that store the read data
 * @param  rx_buffer_length   length of data to be read
 * @param  src_address        Source address to read data from 
 * @retval BSP status
*/
uint32_t SNode_Flash_Read(void * p_rx_buffer,size_t rx_buffer_length,uint32_t src_address)
{
    int32_t err_code = FLASH_OK;

    err_code = BSP_QSPI_Read(p_rx_buffer, rx_buffer_length, src_address);
    if(err_code != FLASH_OK)
    {
       NRF_LOG_ERROR("FLASH : Flash read operation failed\r\n");
    }
        
    return err_code;
}




#endif // QSPI_ENABLE //



/***************** (C) COPYRIGHT DotCom IoT *****  END OF FILE*************************/
