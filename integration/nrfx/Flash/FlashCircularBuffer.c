/**
******************************************************************************
* @file    FlashCircularBuffer.c
* @author  SNode Firmware Team 
* @version V1.1.0
* @date    02-Nov-2021
* @brief   Source file for storing data in circular buffer in QSPI Flash
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "SNodeConfig.h"
#if (DATA_COLLECTION_USING_EXTFLASH == 1U)


#include "FlashCircularBuffer.h"



/* Macros Defintions ---------------------------------------------------------*/


/* Flash Circular Buffer Variables ---------------------------------------------------------*/
flash_circular_buffer fcb;




/* Function Definitions ---------------------------------------------------------*/

/*
 * @brief  Function to check whether data is available to be read
 * @param  buffer    Pointer to flash circular buffer
 * @retval bool 
*/
static bool isDataAvailable(flash_circular_buffer* buffer)
{
  return buffer->data_available;
}

/*
 * @brief  Function to check whether memory is available
 * @param  buffer    Pointer to flash circular buffer
 * @retval bool 
*/
static bool isMemoryAvailable(flash_circular_buffer* buffer)
{
  return buffer->memory_available;
}



/*
 * @brief  Function to Initialize Circular Buffer
 * @param  buffer    Pointer to flash circular buffer
 * @retval None
*/
void FlashCircularBufferInit(flash_circular_buffer* buffer)
{
  buffer->tx_data = NULL;
  buffer->rx_data = NULL;
  buffer->head = FLASH_BUFFER_START_ADDR;
  buffer->tail = FLASH_BUFFER_START_ADDR;
  buffer->data_available = false;
  buffer->memory_available = true;
}



/*
 * @brief  Function to Store data in Flash Ring Buffer
 * @param  buffer    Pointer to flash circular buffer
 * @param  dataToBuffer    Pointer to dataset to store in ring buffer
 * @retval FCB Status
*/
fcb_status WriteDatasetToFlash(flash_circular_buffer* buffer, void* dataToBuffer, uint16_t length)
{
  uint32_t dst_addr,next_addr,err_code,data_len;
  uint32_t current_addr; 
  uint32_t head_sector, tail_sector;

  #if(MP34DT05_ENABLE == 0U)
  buffer->tx_data = (DATASET*)dataToBuffer;
  dst_addr = buffer->head;
  data_len = sizeof(DATASET);
  #else
  buffer->tx_data = (int16_t*)dataToBuffer;
  dst_addr = buffer->head;
  data_len = length;
  #endif

  /* Calculate next address */  
  if((buffer->head + data_len) >= FLASH_BUFFER_SIZE)
  {
    next_addr = FLASH_BUFFER_START_ADDR;
  }
  else
  {
    next_addr = (unsigned int)(buffer->head + data_len);
  }

  
  /*Check whether the buffer head and tail sectors are going to overlap*/
  if(buffer->tail > buffer->head)
  {
    // if the difference is less than 4096
    if((buffer->tail - buffer->head) < QSPI_BLOCK_SIZE_64K)
    {
      //Calculate current sector number from the address
      head_sector = next_addr / QSPI_BLOCK_SIZE_64K;
      tail_sector = buffer->tail / QSPI_BLOCK_SIZE_64K;

      /*Check whether head and tail sector are same */
      if(head_sector == tail_sector)
      {
        NRF_LOG_INFO("FCB : No empty locations to write\r\n. Next Addr: %d  Tail: %d",next_addr,buffer->tail);
        return FCB_ADDR_ON_SAME_SECTOR;
      }
    }
  }
  

  /* Check availablity of free memory locations */
 /* Not storing the data into the location just before the circular buffer tail 
 (meaning that the head would advance to the current location of the tail),
  we're about to overflow the buffer and so we don't write the character or
  advance the head. */
  if(next_addr != buffer->tail)
  {
    /* Write data to flash and increment the head position */
    err_code = nrf_drv_qspi_write(buffer->tx_data, data_len, dst_addr);
    APP_ERROR_CHECK(err_code);
    WAIT_FOR_PERIPH();
    if(err_code == FCB_OK)
    {
      buffer->head = buffer->head + data_len;
      buffer->data_available = true;
      NRF_LOG_INFO("Flash Buffer Current Head: %d, Tail: %d ",fcb.head,fcb.tail);
      err_code = FCB_OK;
    }
    else
    {
      NRF_LOG_INFO("FCB: Write Operation Failed");
      err_code = FCB_WRITE_ERROR;
    }

    
    /*Reset the circular buffer head to start position,
     if the head has reached the end address of memory */  
    if(buffer->head == FLASH_BUFFER_END_ADDR)
    {
      buffer->head = FLASH_BUFFER_START_ADDR;
    }
  }
  else
  {
    buffer->memory_available = false;
    err_code = FCB_NO_EMPTY_LOCATIONS;
    NRF_LOG_INFO("FCB Status: No empty locations");
  }
  
  return err_code;

}


/*
 * @brief  Function to Read data from Flash Circular Buffer
 * @param  buffer    Pointer to flash circular buffer
 * @param  dataToBuffer    Pointer to dataset to store in ring buffer
 * @retval FCB Status
*/
fcb_status ReadDatasetFromFlash(flash_circular_buffer* buffer,void* rcvBuffer,uint32_t length)
{
  uint32_t src_addr,data_len,err_code;
  
  #if MP34DT05_ENABLE
  buffer->rx_data = (int16_t*)rcvBuffer;
  #else
  buffer->rx_data = (DATASET*)rcvBuffer;
  #endif

  data_len = length;
  src_addr = buffer->tail;

  /*Check wether data is available to read from flash */  
  if(src_addr != buffer->head)
  {
      err_code = nrf_drv_qspi_read(buffer->rx_data, data_len,src_addr);
      WAIT_FOR_PERIPH();
      if(err_code == FCB_OK)
      {
        buffer->tail = buffer->tail + data_len;
        NRF_LOG_INFO("Flash Buffer Current Head: %d, Tail: %d ",fcb.head,fcb.tail);
      }
      else
      {
        err_code = FCB_READ_ERROR;
        NRF_LOG_INFO("FCB: Read Operation failed"); 
      }   

    /*Reset the circular buffer tail to start position,
     if the tail has reached the end address of memory */  
    if(buffer->tail == FLASH_BUFFER_END_ADDR)
    {
      /* Reset the tail position */
      buffer->tail = FLASH_BUFFER_START_ADDR;
    }
    
    /* Erase the read sector if the buffer head isn't locating memory
     in that sector */
      EraseFlashSector(buffer,src_addr);
      
  }
  else
  {
    buffer->data_available = false;
    NRF_LOG_INFO("FCB Status: No data available to read");
    err_code = FCB_NO_DATA_AVAILABLE;
  }

  return err_code;
}




fcb_status EraseFlashSector(flash_circular_buffer* buffer,uint32_t sector_addr)
{
  uint32_t err_code = FCB_OK;

#if(MP34DT05_ENABLE)

 /* Erase the previosuly read sector as soon as the current flash buffer tail
  position points to adddress on the next */
 switch(buffer->tail)
 {
   case ((BLOCK1_ADDR - ((QSPI_BLOCK_SIZE_64K * 1) % 32000) + 32000)):   
   case ((BLOCK2_ADDR - ((QSPI_BLOCK_SIZE_64K * 2) % 32000) + 32000)):      
   case ((BLOCK3_ADDR - ((QSPI_BLOCK_SIZE_64K * 3) % 32000) + 32000)):      
   case ((BLOCK4_ADDR - ((QSPI_BLOCK_SIZE_64K * 4) % 32000) + 32000)):      
   case ((BLOCK5_ADDR - ((QSPI_BLOCK_SIZE_64K * 5) % 32000) + 32000)):   
   case ((BLOCK6_ADDR - ((QSPI_BLOCK_SIZE_64K * 6) % 32000) + 32000)):      
   case ((BLOCK7_ADDR - ((QSPI_BLOCK_SIZE_64K * 7) % 32000) + 32000)):            
   case ((BLOCK8_ADDR - ((QSPI_BLOCK_SIZE_64K * 8) % 32000) + 32000)):          
   case ((BLOCK9_ADDR - ((QSPI_BLOCK_SIZE_64K * 9) % 32000) + 32000)):          
   case ((BLOCK10_ADDR - ((QSPI_BLOCK_SIZE_64K * 10) % 32000) + 32000)):          
   case ((BLOCK11_ADDR - ((QSPI_BLOCK_SIZE_64K * 11) % 32000) + 32000)):          
   case ((BLOCK12_ADDR - ((QSPI_BLOCK_SIZE_64K * 12) % 32000) + 32000)):          
   case ((BLOCK13_ADDR - ((QSPI_BLOCK_SIZE_64K * 13) % 32000) + 32000)):          
   case ((BLOCK14_ADDR - ((QSPI_BLOCK_SIZE_64K * 14) % 32000) + 32000)):          
   case ((BLOCK15_ADDR - ((QSPI_BLOCK_SIZE_64K * 15) % 32000) + 32000)):          

       /* Check if buffer head position is ahead or behind
        the tail position. If head position is greate than 
        tail erase the previos sector */
       if(buffer->head > buffer->tail)
       {
         err_code = nrf_drv_qspi_erase(NRF_QSPI_ERASE_LEN_64KB,sector_addr - 16000);
         APP_ERROR_CHECK(err_code);
         WAIT_FOR_PERIPH();
         if(err_code == NRF_SUCCESS)
         {
           NRF_LOG_INFO("FCB : Erased Sector Address: %d",sector_addr- 16000);
           nrf_delay_ms(1);
           err_code = FCB_OK;
         }
         else
         {
           err_code = FCB_ERASE_ERROR;
         }
       }
       else
       {
         /* Check whether the current buffer header position sector and
          previous buffer tail posoition sector macthes or not. If it odesn't macth 
          erase the sector */
         if((int)(buffer->head/QSPI_BLOCK_SIZE_64K) != (int)((buffer->tail - 32000)/QSPI_BLOCK_SIZE_64K))
         {            
           err_code = nrf_drv_qspi_erase(NRF_QSPI_ERASE_LEN_64KB,sector_addr - 16000);
           APP_ERROR_CHECK(err_code);
           WAIT_FOR_PERIPH();
           if(err_code == NRF_SUCCESS)
           {
             NRF_LOG_INFO("FCB : Erased Sector Address: %d",sector_addr- 16000);
             nrf_delay_ms(1);
             err_code = FCB_OK;
           }
           else
           {
             err_code = FCB_ERASE_ERROR;
           }
         }
         else
         {
           NRF_LOG_INFO("FCB: Header sector and Tail sector matches.\r\nCant erase memory");
           err_code = FCB_ADDR_ON_SAME_SECTOR;
         }
       }
     
      break;
     
      /* Event generated when tail rollsover the buffer */
     case FLASH_BUFFER_START_ADDR : 
       /* Check if the buffer header position is not overlapping 
       the last sector in the flash buffer */
       if((int)(buffer->head/QSPI_BLOCK_SIZE_64K) != ((FLASH_BUFFER_END_ADDR - 32000)/QSPI_BLOCK_SIZE_64K))
       {
         err_code = nrf_drv_qspi_erase(NRF_QSPI_ERASE_LEN_64KB,sector_addr - 16000);
         APP_ERROR_CHECK(err_code);
         WAIT_FOR_PERIPH();
         if(err_code == NRF_SUCCESS)
         {
           NRF_LOG_INFO("FCB : Erased Sector Address: %d",sector_addr- 16000);
           nrf_delay_ms(1);
           err_code = FCB_OK;
         }  
       }  
      break;

 }

#else
   
 /* Erase the previosuly read sector as soon as the current flash buffer tail
  position points to adddress on the next */
 switch(buffer->tail)
 {
   case ((BLOCK1_ADDR - ((QSPI_BLOCK_SIZE_64K * 1) % sizeof(DATASET)) + sizeof(DATASET))):   
   case ((BLOCK2_ADDR - ((QSPI_BLOCK_SIZE_64K * 2) % sizeof(DATASET)) + sizeof(DATASET))):      
   case ((BLOCK3_ADDR - ((QSPI_BLOCK_SIZE_64K * 3) % sizeof(DATASET)) + sizeof(DATASET))):      
   case ((BLOCK4_ADDR - ((QSPI_BLOCK_SIZE_64K * 4) % sizeof(DATASET)) + sizeof(DATASET))):      
   case ((BLOCK5_ADDR - ((QSPI_BLOCK_SIZE_64K * 5) % sizeof(DATASET)) + sizeof(DATASET))):   
   case ((BLOCK6_ADDR - ((QSPI_BLOCK_SIZE_64K * 6) % sizeof(DATASET)) + sizeof(DATASET))):      
   case ((BLOCK7_ADDR - ((QSPI_BLOCK_SIZE_64K * 7) % sizeof(DATASET)) + sizeof(DATASET))):          
  
   case ((BLOCK8_ADDR - ((QSPI_BLOCK_SIZE_64K * 8) % sizeof(DATASET)) + sizeof(DATASET))):          
   case ((BLOCK9_ADDR - ((QSPI_BLOCK_SIZE_64K * 9) % sizeof(DATASET)) + sizeof(DATASET))):          
   case ((BLOCK10_ADDR - ((QSPI_BLOCK_SIZE_64K * 10) % sizeof(DATASET)) + sizeof(DATASET))):          
   case ((BLOCK11_ADDR - ((QSPI_BLOCK_SIZE_64K * 11) % sizeof(DATASET)) + sizeof(DATASET))):          
   case ((BLOCK12_ADDR - ((QSPI_BLOCK_SIZE_64K * 12) % sizeof(DATASET)) + sizeof(DATASET))):          
   case ((BLOCK13_ADDR - ((QSPI_BLOCK_SIZE_64K * 13) % sizeof(DATASET)) + sizeof(DATASET))):          
   case ((BLOCK14_ADDR - ((QSPI_BLOCK_SIZE_64K * 14) % sizeof(DATASET)) + sizeof(DATASET))):          

       /* Check if buffer head position is ahead or behind
        the tail position. If head position is greate than 
        tail erase the previos sector */
       if(buffer->head > buffer->tail)
       {
         err_code = nrf_drv_qspi_erase(NRF_QSPI_ERASE_LEN_64KB,sector_addr - sizeof(DATASET));
         APP_ERROR_CHECK(err_code);
         WAIT_FOR_PERIPH();
         if(err_code == NRF_SUCCESS)
         {
           NRF_LOG_INFO("FCB : Erased Sector Address: %d",sector_addr- sizeof(DATASET));
           nrf_delay_ms(1);
           err_code = FCB_OK;
         }
         else
         {
           err_code = FCB_ERASE_ERROR;
         }
       }
       else
       {
         /* Check whether the current buffer header position sector and
          previous buffer tail posoition sector macthes or not. If it odesn't macth 
          erase the sector */
         if((int)(buffer->head/QSPI_BLOCK_SIZE_64K) != (int)((buffer->tail - sizeof(DATASET))/QSPI_BLOCK_SIZE_64K))
         {            
           err_code = nrf_drv_qspi_erase(NRF_QSPI_ERASE_LEN_64KB,sector_addr - sizeof(DATASET));
           APP_ERROR_CHECK(err_code);
           WAIT_FOR_PERIPH();
           if(err_code == NRF_SUCCESS)
           {
             NRF_LOG_INFO("FCB : Erased Sector Address: %d",sector_addr- sizeof(DATASET));
             nrf_delay_ms(1);
             err_code = FCB_OK;
           }
           else
           {
             err_code = FCB_ERASE_ERROR;
           }
         }
         else
         {
           NRF_LOG_INFO("FCB: Header sector and Tail sector matches.\r\nCant erase memory");
           err_code = FCB_ADDR_ON_SAME_SECTOR;
         }
       }
     
      break;
     
      /* Event generated when tail rollsover the buffer */
     case FLASH_BUFFER_START_ADDR : 
       /* Check if the buffer header position is not overlapping 
       the last sector in the flash buffer */
       if((int)(buffer->head/QSPI_BLOCK_SIZE_64K) != ((FLASH_BUFFER_END_ADDR - sizeof(DATASET))/QSPI_BLOCK_SIZE_64K))
       {
         err_code = nrf_drv_qspi_erase(NRF_QSPI_ERASE_LEN_64KB,sector_addr - sizeof(DATASET));
         APP_ERROR_CHECK(err_code);
         WAIT_FOR_PERIPH();
         if(err_code == NRF_SUCCESS)
         {
           NRF_LOG_INFO("FCB : Erased Sector Address: %d",sector_addr- sizeof(DATASET));
           nrf_delay_ms(1);
           err_code = FCB_OK;
         }  
       }  
      break;

 }
 
#endif 

 return  err_code;

}




//
//fcb_status EraseFlashSector(flash_circular_buffer* buffer,uint32_t sector_addr)
//{
//   uint32_t err_code = FCB_OK;
//    
//  /* Erase the previosuly read sector as soon as the current flash buffer tail
//   position points to adddress on the next */
//  switch(buffer->tail)
//  {
//    case ((BLOCK1_ADDR - ((QSPI_BLOCK_SIZE_64K * 1) % sizeof(DATASET)) + sizeof(DATASET))):   
//    case ((BLOCK2_ADDR - ((QSPI_BLOCK_SIZE_64K * 2) % sizeof(DATASET)) + sizeof(DATASET))):      
//    case ((BLOCK3_ADDR - ((QSPI_BLOCK_SIZE_64K * 3) % sizeof(DATASET)) + sizeof(DATASET))):      
//    case ((BLOCK4_ADDR - ((QSPI_BLOCK_SIZE_64K * 4) % sizeof(DATASET)) + sizeof(DATASET))):      
//    case ((BLOCK5_ADDR - ((QSPI_BLOCK_SIZE_64K * 5) % sizeof(DATASET)) + sizeof(DATASET))):   
//    case ((BLOCK6_ADDR - ((QSPI_BLOCK_SIZE_64K * 6) % sizeof(DATASET)) + sizeof(DATASET))):      
//    case ((BLOCK7_ADDR - ((QSPI_BLOCK_SIZE_64K * 7) % sizeof(DATASET)) + sizeof(DATASET))):          
//   
//    case ((BLOCK8_ADDR - ((QSPI_BLOCK_SIZE_64K * 8) % sizeof(DATASET)) + sizeof(DATASET))):          
//    case ((BLOCK9_ADDR - ((QSPI_BLOCK_SIZE_64K * 9) % sizeof(DATASET)) + sizeof(DATASET))):          
//    case ((BLOCK10_ADDR - ((QSPI_BLOCK_SIZE_64K * 10) % sizeof(DATASET)) + sizeof(DATASET))):          
//    case ((BLOCK11_ADDR - ((QSPI_BLOCK_SIZE_64K * 11) % sizeof(DATASET)) + sizeof(DATASET))):          
//    case ((BLOCK12_ADDR - ((QSPI_BLOCK_SIZE_64K * 12) % sizeof(DATASET)) + sizeof(DATASET))):          
//    case ((BLOCK13_ADDR - ((QSPI_BLOCK_SIZE_64K * 13) % sizeof(DATASET)) + sizeof(DATASET))):          
//    case ((BLOCK14_ADDR - ((QSPI_BLOCK_SIZE_64K * 14) % sizeof(DATASET)) + sizeof(DATASET))):          
//
//        /* Check if buffer head position is ahead or behind
//         the tail position. If head position is greate than 
//         tail erase the previos sector */
//        if(buffer->head > buffer->tail)
//        {
//          err_code = nrf_drv_qspi_erase(NRF_QSPI_ERASE_LEN_64KB,sector_addr - sizeof(DATASET));
//          APP_ERROR_CHECK(err_code);
//          WAIT_FOR_PERIPH();
//          if(err_code == NRF_SUCCESS)
//          {
//            NRF_LOG_INFO("FCB : Erased Sector Address: %d",sector_addr- sizeof(DATASET));
//            nrf_delay_ms(1);
//            err_code = FCB_OK;
//          }
//          else
//          {
//            err_code = FCB_ERASE_ERROR;
//          }
//        }
//        else
//        {
//          /* Check whether the current buffer header position sector and
//           previous buffer tail posoition sector macthes or not. If it odesn't macth 
//           erase the sector */
//          if((int)(buffer->head/QSPI_BLOCK_SIZE_64K) != (int)((buffer->tail - sizeof(DATASET))/QSPI_BLOCK_SIZE_64K))
//          {            
//            err_code = nrf_drv_qspi_erase(NRF_QSPI_ERASE_LEN_64KB,sector_addr - sizeof(DATASET));
//            APP_ERROR_CHECK(err_code);
//            WAIT_FOR_PERIPH();
//            if(err_code == NRF_SUCCESS)
//            {
//              NRF_LOG_INFO("FCB : Erased Sector Address: %d",sector_addr- sizeof(DATASET));
//              nrf_delay_ms(1);
//              err_code = FCB_OK;
//            }
//            else
//            {
//              err_code = FCB_ERASE_ERROR;
//            }
//          }
//          else
//          {
//            NRF_LOG_INFO("FCB: Header sector and Tail sector matches.\r\nCant erase memory");
//            err_code = FCB_ADDR_ON_SAME_SECTOR;
//          }
//        }
//      
//       break;
//      
//       /* Event generated when tail rollsover the buffer */
//      case FLASH_BUFFER_START_ADDR : 
//        /* Check if the buffer header position is not overlapping 
//        the last sector in the flash buffer */
//        if((int)(buffer->head/QSPI_BLOCK_SIZE_64K) != ((FLASH_BUFFER_END_ADDR - sizeof(DATASET))/QSPI_BLOCK_SIZE_64K))
//        {
//          err_code = nrf_drv_qspi_erase(NRF_QSPI_ERASE_LEN_64KB,sector_addr - sizeof(DATASET));
//          APP_ERROR_CHECK(err_code);
//          WAIT_FOR_PERIPH();
//          if(err_code == NRF_SUCCESS)
//          {
//            NRF_LOG_INFO("FCB : Erased Sector Address: %d",sector_addr- sizeof(DATASET));
//            nrf_delay_ms(1);
//            err_code = FCB_OK;
//          }  
//        }  
//       break;
//
//  }
//  
//  return  err_code;
//
//}
//
//



#endif //USE_EXT_FLASH_STORE_DATA



/***************** (C) COPYRIGHT DotCom IoT *****  END OF FILE *************************/
  