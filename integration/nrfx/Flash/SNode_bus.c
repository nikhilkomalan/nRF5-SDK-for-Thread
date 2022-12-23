/**
******************************************************************************
* @file    SNode_bus.h
* @author  SNode Firmware Team 
* @version V1.0.0
* @date    04-Feb-2020
* @brief   source file for the BSP BUS IO driver
******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include "SNode_bus.h"



/** @defgroup SNode_Private_Variables BUS Private Variables
  * @{
  */

#if (SNODE_I2C_ENABLE == 1U)
const nrf_drv_twi_t m_twi0 = NRF_DRV_TWI_INSTANCE(TWI0_INSTANCE_ID);
const nrf_drv_twi_t m_twi1 = NRF_DRV_TWI_INSTANCE(TWI1_INSTANCE_ID);

static volatile bool is_twi0_intialized;
static volatile bool is_twi1_intialized;

#endif 



#if (SNODE_QSPI_ENABLE == 1U)
volatile bool m_finished = false;
static volatile bool is_qspi_intialized;
#endif 




/*******************************************************************************
                            BUS OPERATIONS OVER QSPI
*******************************************************************************/
#if (SNODE_QSPI_ENABLE == 1U)

/*
 * @brief  QSPI Interrupt Handler function
 * @param  event      QSPI event
 * Qparam  p_context  default param
 * @retval None
*/
static void qspi_handler(nrf_drv_qspi_evt_t event, void * p_context)
{
    UNUSED_PARAMETER(event);
    UNUSED_PARAMETER(p_context);
    m_finished = true;
}


/**
  * @brief  Function for initializing the QSPI driver instance.
            This function configures the peripheral and its interrupts,
            and activates it.
  * @param  None
  * @retval BSP status
*/
uint32_t BSP_QSPI_Init(void)
{
    nrfx_err_t err_code = NRF_SUCCESS;

    if(!is_qspi_intialized)
    {
      /* Initilaize with default configuration */
      nrf_drv_qspi_config_t config = NRF_DRV_QSPI_DEFAULT_CONFIG;    
      err_code = nrf_drv_qspi_init(&config, qspi_handler, NULL);
      APP_ERROR_CHECK(err_code);
      
      if(err_code == NRF_SUCCESS)
      is_qspi_intialized = true;
    
    }

    return  err_code;
}

/**
  * @brief  Function for uninitializing the QSPI driver instance. 
  * @param  None
  * @retval BSP status
*/
uint32_t BSP_QSPI_DeInit(void)
{
    nrfx_err_t err_code = NRFX_SUCCESS;
    
    if(is_qspi_intialized)
    {
      nrf_drv_qspi_uninit();
      is_qspi_intialized = false;
    }
    
    return  err_code;
}


/*
 * @brief  Function for start erasing of one memory block
           - 4KB, 64KB, or the whole chip.
 * @param  Length of memory 
 * @param  Start Address
 * @retval 0 - Success
 *         else - Error
*/
uint32_t BSP_QSPI_Erase(nrf_qspi_erase_len_t len,uint32_t start_address)
{
    nrfx_err_t err_code;

    err_code = nrf_drv_qspi_erase(len, start_address);
    APP_ERROR_CHECK(err_code);
    WAIT_FOR_PERIPH();
    return err_code;
}



/*
 * @brief  Function for writing data to QSPI memory.
 * @param  p_tx_buffer      data buffer to be written 
 * @param  tx_buffer_length Length of data to be written
 * @param  dst_address      Destination Address
 * @retval 0 - Success
 *         else - Error
*/
uint32_t BSP_QSPI_Write(void const * p_tx_buffer,size_t tx_buffer_length,uint32_t dst_address)
{
    nrfx_err_t err_code;

    err_code = nrf_drv_qspi_write(p_tx_buffer, tx_buffer_length,dst_address);
    APP_ERROR_CHECK(err_code);
    WAIT_FOR_PERIPH();
    return err_code;
}


/*
 * @brief  Function for read data from QSPI memory.
 * @param  p_rx_buffer      data buffer to read data into
 * @param  rx_buffer_length Length of data to be read
 * @param  src_address      Source Address
 * @retval 0 - Success
 *         else - Error
*/
uint32_t BSP_QSPI_Read(void * p_rx_buffer,size_t rx_buffer_length,uint32_t src_address)
{
    nrfx_err_t err_code;

    err_code = nrf_drv_qspi_read(p_rx_buffer, rx_buffer_length, src_address);
    WAIT_FOR_PERIPH();
    return err_code;
}


/*
 * @brief  Function for sending operation code,sending data, 
           and receiving data from the memory device.
 * @param  p_config      Pointer to the structure with opcode
                         and transfer configuration. 
 * @param  p_tx_buffer   Pointer to the array with data to send. 
                         Can be NULL if only opcode is transmitted
 * @param  p_rx_buffer   Pointer to the array for data to receive. 
                         Can be NULL if there is nothing to receive.
 * @retval 0 - Success
 *         else - Error
*/
uint32_t BSP_QSPI_Configure_Memory(nrf_qspi_cinstr_conf_t const *p_config,void const * p_tx_buffer, void * p_rx_buffer)
{
    nrfx_err_t err_code;

    err_code = nrf_drv_qspi_cinstr_xfer(p_config, p_tx_buffer, p_rx_buffer);
    APP_ERROR_CHECK(err_code);

    return err_code;
}


#endif // QSPI_ENABLE 


/***************** (C) COPYRIGHT DotCom IoT *****  END OF FILE *************************/
