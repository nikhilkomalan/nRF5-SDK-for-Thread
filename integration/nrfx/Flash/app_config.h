/**
  ******************************************************************************
  * @file    app_config.h 
  * @author  SNode Firmware Team
  * @version V1.0.0
  * @date    02-Oct-2021
  * @brief   Application Configuration file to configure SDK 
             as per the application need. 
 ******************************************************************************
 */
  
#ifndef APP_CONFIG_H
#define APP_CONFIG_H


#include "SNodeConfig.h"

#define NRFX_PDM_ENABLED 1U

/*######################################################################
   OpenThread Stack Parameters
#######################################################################*/

//#define THREAD_PANID    43981   // Pan id <0-65535> 
//#define THREAD_CHANNEL  26     // Channel <11-26> 

//#define THREAD_CHANNEL 18
//#define THREAD_CHANNEL 26
#define THREAD_PANID    30583   // Pan id <0-65535> 
#define THREAD_CHANNEL 17


/*#####################################################################
        Configuration to enbale APP Timer 
#######################################################################*/

#define APP_TIMER_ENABLED 1

// <0=> 32768 Hz 
// <1=> 16384 Hz 
// <3=> 8192 Hz 
// <7=> 4096 Hz 
// <15=> 2048 Hz 
// <31=> 1024 Hz 
#define APP_TIMER_CONFIG_RTC_FREQUENCY 1

//#define APP_TIMER_CONFIG_IRQ_PRIORITY 6
#define APP_TIMER_CONFIG_IRQ_PRIORITY 3

// <o> APP_TIMER_CONFIG_OP_QUEUE_SIZE - Capacity of timer requests queue. 
// <i> Size of the queue depends on how many timers are used
// <i> in the system, how often timers are started and overall
// <i> system latency. If queue size is too small app_timer calls
// <i> will fail.
#define APP_TIMER_CONFIG_OP_QUEUE_SIZE 64

// <q> APP_TIMER_CONFIG_USE_SCHEDULER  - Enable scheduling app_timer events to app_scheduler
#define APP_TIMER_CONFIG_USE_SCHEDULER 1

// <q> APP_TIMER_KEEPS_RTC_ACTIVE  - Enable RTC always on
#define APP_TIMER_KEEPS_RTC_ACTIVE 0

// <o> APP_TIMER_SAFE_WINDOW_MS - Maximum possible latency (in milliseconds) of handling app_timer event. 
// <i> Maximum possible timeout that can be set is reduced by safe window.
// <i> Example: RTC frequency 16384 Hz, maximum possible timeout 1024 seconds - APP_TIMER_SAFE_WINDOW_MS.
// <i> Since RTC is not stopped when processor is halted in debugging session, this value
// <i> must cover it if debugging is needed. It is possible to halt processor for APP_TIMER_SAFE_WINDOW_MS
// <i> without corrupting app_timer behavior.
#define APP_TIMER_SAFE_WINDOW_MS 300000

// <q> APP_TIMER_WITH_PROFILER  - Enable app_timer profiling
#define APP_TIMER_WITH_PROFILER 1

// <q> APP_TIMER_CONFIG_SWI_NUMBER  - Configure SWI instance used.
#define APP_TIMER_CONFIG_SWI_NUMBER 0


/*######################################################################
   Configuration for enabling NRFX TWI/TWIM peripheral
#######################################################################*/

#define TWI_ENABLED         1       // enable TWI peripherl driver (legacy layer) 
#define NRFX_TWI_ENABLED    1       // enable NRFX TWI peripherl driver
#define NRFX_TWIM_ENABLED   1       // enable TWIM peripheral driver
#define TWI0_ENABLED        1       // Enable TWI0 Instance
#define TWI1_ENABLED        1       // Enable TWI1 Instance

/* Use EasyDMA (if present). Determines if you use the
 TWI peripheral (without DMA) or the TWIM peripheral (with DMA).*/
#define TWI0_USE_EASY_DMA   1
#define TWI1_USE_EASY_DMA   1

/* (TWI/NRFX_TWI/NRFX_TWIM)_DEFAULT_CONFIG_FREQUENCY  - Frequency 
<26738688=> 100k ,67108864=> 250k, <104857600=> 400k */
#define TWI_DEFAULT_CONFIG_FREQUENCY        26738688
#define NRFX_TWI_DEFAULT_CONFIG_FREQUENCY   26738688
#define NRFX_TWIM_DEFAULT_CONFIG_FREQUENCY  26738688

/* <o> (TWI/NRFX_TWI/NRFX_TWIM)_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority 
 Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
  <0=> 0 (highest) 
  <1=> 1 
  <2=> 2 
  <3=> 3 
  <4=> 4 
  <5=> 5 
  <6=> 6 
  <7=> 7 
*/
#define TWI_DEFAULT_CONFIG_IRQ_PRIORITY         6
#define NRFX_TWI_DEFAULT_CONFIG_IRQ_PRIORITY    6
#define NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY   6



/*########################################################################
              Configuration for Watchdog Timer 
#########################################################################*/
#define NRFX_WDT_ENABLED 1

// <1=> Run in SLEEP, Pause in HALT 
// <8=> Pause in SLEEP, Run in HALT 
// <9=> Run in SLEEP and HALT 
// <0=> Pause in SLEEP and HALT 
#define NRFX_WDT_CONFIG_BEHAVIOUR   1    /* Run in sleep and stop in halt */

#define NRFX_WDT_CONFIG_RELOAD_VALUE 120000 /* Reload value (ms)  <15-4294967295> */
#define NRFX_WDT_CONFIG_IRQ_PRIORITY 6



/*########################################################################
              Configuration for ADC
#########################################################################*/
#define SAADC_ENABLED       1
#define NRFX_SAADC_ENABLED  1
// <o> NRFX_SAADC_CONFIG_RESOLUTION  - Resolution
 
// <0=> 8 bit 
// <1=> 10 bit 
// <2=> 12 bit 
// <3=> 14 bit 
#define NRFX_SAADC_CONFIG_RESOLUTION 2
#define SAADC_CONFIG_RESOLUTION      2

#define SAADC_CONFIG_OVERSAMPLE       0
#define NRFX_SAADC_CONFIG_OVERSAMPLE  0

#define SAADC_CONFIG_LP_MODE          1
#define NRFX_SAADC_CONFIG_LP_MODE     1

#define SAADC_CONFIG_IRQ_PRIORITY      6
#define NRFX_SAADC_CONFIG_IRQ_PRIORITY 6



/*###########################################################################
                  Configuration for enabling QSPI peripheral
#############################################################################*/

#define QSPI_ENABLED                  1       // Enable QSPI peripheral driver (legacy)
#define NRFX_QSPI_ENABLED             1       // Enable QSPI peripheral driver

/* QSPI Pin Defintions */
#define QSPI_PIN_SCK                  19      // SCK  pin
#define QSPI_PIN_CSN                  17      // CS pin
#define QSPI_PIN_IO0                  20      // IO0 
#define QSPI_PIN_IO1                  21      // IO1 
#define QSPI_PIN_IO2                  22      // IO2 
#define QSPI_PIN_IO3                  23      // IO3 

#define NRFX_QSPI_PIN_SCK             19      // SCK  pin
#define NRFX_QSPI_PIN_CSN             17      // CS pin
#define NRFX_QSPI_PIN_IO0             20      // IO0 
#define NRFX_QSPI_PIN_IO1             21      // IO1 
#define NRFX_QSPI_PIN_IO2             22      // IO2 
#define NRFX_QSPI_PIN_IO3             23      // IO3 


/* NRFX_QSPI_CONFIG_SCK_DELAY - tSHSL, tWHSL and tSHWL in number 
of 16 MHz periods (62.5 ns).  <0-255>  */
#define QSPI_CONFIG_SCK_DELAY         1
#define NRFX_QSPI_CONFIG_SCK_DELAY    1

/* NRFX_QSPI_CONFIG_XIP_OFFSET - Address offset in the external memory 
for Execute in Place operation.  */
#define QSPI_CONFIG_XIP_OFFSET        0
#define NRFX_QSPI_CONFIG_XIP_OFFSET   0


/* NRFX_QSPI_CONFIG_READOC - Number of data lines and opcode used for reading. */
// <0=> FastRead 
// <1=> Read2O 
// <2=> Read2IO 
// <3=> Read4O 
// <4=> Read4IO 
#define QSPI_CONFIG_READOC            4
#define NRFX_QSPI_CONFIG_READOC       4

/* NRFX_QSPI_CONFIG_WRITEOC  - Number of data lines and opcode used for writing. */
// <0=> PP 
// <1=> PP2O 
// <2=> PP4O 
// <3=> PP4IO 
#define QSPI_CONFIG_WRITEOC           3
#define NRFX_QSPI_CONFIG_WRITEOC      3

/* NRFX_QSPI_CONFIG_ADDRMODE  - Addressing mode. */
// <0=> 24bit 
// <1=> 32bit 
#define NRFX_QSPI_CONFIG_ADDRMODE     0
#define QSPI_CONFIG_ADDRMODE          0

/* <o> NRFX_QSPI_CONFIG_MODE  - SPI mode.  */
// <0=> Mode 0 
// <1=> Mode 1 
#define NRFX_QSPI_CONFIG_MODE         0
#define QSPI_CONFIG_MODE              0

/* <o> NRFX_QSPI_CONFIG_FREQUENCY  - Frequency divider. */
// <0=> 32MHz/1 
// <1=> 32MHz/2 
// <2=> 32MHz/3 
// <3=> 32MHz/4 
// <4=> 32MHz/5 
// <5=> 32MHz/6 
// <6=> 32MHz/7 
// <7=> 32MHz/8 
// <8=> 32MHz/9 
// <9=> 32MHz/10 
// <10=> 32MHz/11 
// <11=> 32MHz/12 
// <12=> 32MHz/13 
// <13=> 32MHz/14 
// <14=> 32MHz/15 
// <15=> 32MHz/16 
#define QSPI_CONFIG_FREQUENCY         1
#define NRFX_QSPI_CONFIG_FREQUENCY    1

#define NRFX_QSPI_CONFIG_IRQ_PRIORITY 2
#define QSPI_CONFIG_IRQ_PRIORITY      2




/*##########################################################################
                  Configuration for enabling APP USB CDC
############################################################################*/

/* Enable USB Device library */
#define APP_USBD_ENABLED              1

/* <s> APP_USBD_VID - Vendor ID. */
#define APP_USBD_VID                  0x4507

/* APP_USBD_PID - Product ID. */
#define APP_USBD_PID                  0x10DE

/* APP_USBD_DEVICE_VER_MAJOR - Major device version  <0-99>  */
#define APP_USBD_DEVICE_VER_MAJOR     1

/* APP_USBD_DEVICE_VER_MINOR - Minor device version  <0-9> */
#define APP_USBD_DEVICE_VER_MINOR     0

/* <o> APP_USBD_DEVICE_VER_SUB - Sub-minor device version  <0-9>  */
#define APP_USBD_DEVICE_VER_SUB       0

/*Self-powered device, as opposed to bus-powered. */
#define APP_USBD_CONFIG_SELF_POWERED  1

/* MaxPower field in configuration descriptor in milliamps.  <0-500> */
#define APP_USBD_CONFIG_MAX_POWER     100

/* APP_USBD_CONFIG_POWER_EVENTS_PROCESS  - Process power events.  */
#define APP_USBD_CONFIG_POWER_EVENTS_PROCESS 1

/*Enable event queue. */
#define APP_USBD_CONFIG_EVENT_QUEUE_ENABLE   1

/* The size of the event queue.  <16-64>  */
#define APP_USBD_CONFIG_EVENT_QUEUE_SIZE     32

/* Change SOF events handling mode.  */
// <0=> Normal queue 
// <1=> Compress queue 
// <2=> Interrupt 
#define APP_USBD_CONFIG_SOF_HANDLING_MODE     1


/* Provide a function that generates timestamps for logs based on the current SOF.*/
#define APP_USBD_CONFIG_SOF_TIMESTAMP_PROVIDE 0

/* Maximum size of the NULL-terminated string of the string descriptor.  <31-254> */
#define APP_USBD_CONFIG_DESC_STRING_SIZE      31

/*  Enable UTF8 conversion. */
#define APP_USBD_CONFIG_DESC_STRING_UTF_ENABLED 0

/* Supported languages identifiers. */
#define APP_USBD_STRINGS_LANGIDS APP_USBD_LANG_AND_SUBLANG(APP_USBD_LANG_ENGLISH, APP_USBD_SUBLANG_ENGLISH_US)

#define APP_USBD_STRING_ID_MANUFACTURER         1
#define APP_USBD_STRINGS_MANUFACTURER_EXTERN    0

/* APP_USBD_STRINGS_MANUFACTURER - String descriptor for the manufacturer name. */
#define APP_USBD_STRINGS_MANUFACTURER APP_USBD_STRING_DESC("DotCom IoT")

/* Setting ID to 0 disables the string. */
#define APP_USBD_STRING_ID_PRODUCT              2
#define APP_USBD_STRINGS_PRODUCT_EXTERN         0

/* APP_USBD_STRINGS_PRODUCT - String descriptor for the product name. */
#define APP_USBD_STRINGS_PRODUCT APP_USBD_STRING_DESC("SNode")

/* APP_USBD_STRING_ID_SERIAL - Define serial number string ID. */
#define APP_USBD_STRING_ID_SERIAL               3

/*Define whether @ref APP_USBD_STRING_SERIAL is created by macro or declared as a global variable. */
#define APP_USBD_STRING_SERIAL_EXTERN           1

/* <s> APP_USBD_STRING_SERIAL - String descriptor for the serial number.*/
#define APP_USBD_STRING_SERIAL g_extern_serial_number

/* <e> NRFX_USBD_ENABLED - nrfx_usbd - USBD peripheral driver */
#define NRFX_USBD_ENABLED                      1
#define USBD_ENABLED                           1

/* NRFX_USBD_CONFIG_IRQ_PRIORITY  - Interrupt priority */ 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 
#define NRFX_USBD_CONFIG_IRQ_PRIORITY          6
#define USBD_CONFIG_IRQ_PRIORITY               6


/* NRFX_USBD_CONFIG_DMASCHEDULER_MODE  - USBD DMA scheduler working scheme */
// <0=> Prioritized access 
// <1=> Round Robin 
#define NRFX_USBD_CONFIG_DMASCHEDULER_MODE     0
#define USBD_CONFIG_DMASCHEDULER_MODE          0


/* NRFX_USBD_CONFIG_DMASCHEDULER_ISO_BOOST  - Give priority to isochronous transfers */
#define NRFX_USBD_CONFIG_DMASCHEDULER_ISO_BOOST       1
#define USBD_CONFIG_DMASCHEDULER_ISO_BOOST            1


/* Respond to an IN token on ISO IN endpoint with ZLP when no data is ready */ 
#define NRFX_USBD_CONFIG_ISO_IN_ZLP           0
#define USBD_CONFIG_ISO_IN_ZLP                0

/* Define configuration string ID. */
#define APP_USBD_STRING_ID_CONFIGURATION      4

/* Define whether @ref APP_USBD_STRINGS_CONFIGURATION is created by macro or declared as global variable. */
#define APP_USBD_STRING_CONFIGURATION_EXTERN 0

/* String descriptor for the device configuration. */
#define APP_USBD_STRINGS_CONFIGURATION APP_USBD_STRING_DESC("Default configuration")

// <s> APP_USBD_STRINGS_USER - Default values for user strings.
#define APP_USBD_STRINGS_USER X(APP_USER_1, , APP_USBD_STRING_DESC("User 1"))

// </h> 
//==========================================================

// <h> app_usbd_cdc_acm - USB CDC ACM class

//==========================================================
// <q> APP_USBD_CDC_ACM_ENABLED  - Enabling USBD CDC ACM Class library
 

#define APP_USBD_CDC_ACM_ENABLED 1

// <q> APP_USBD_CDC_ACM_ZLP_ON_EPSIZE_WRITE  - Send ZLP on write with same size as endpoint
 

// <i> If enabled, CDC ACM class will automatically send a zero length packet after transfer which has the same size as endpoint.
// <i> This may limit throughput if a lot of binary data is sent, but in terminal mode operation it makes sure that the data is always displayed right after it is sent.

#define APP_USBD_CDC_ACM_ZLP_ON_EPSIZE_WRITE 1



// <e> POWER_ENABLED - nrf_drv_power - POWER peripheral driver - legacy layer
//==========================================================
#define POWER_ENABLED 1
// <o> POWER_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#define POWER_CONFIG_IRQ_PRIORITY 6

// <q> POWER_CONFIG_DEFAULT_DCDCEN  - The default configuration of main DCDC regulator
 

// <i> This settings means only that components for DCDC regulator are installed and it can be enabled.

#define POWER_CONFIG_DEFAULT_DCDCEN 0

// <q> POWER_CONFIG_DEFAULT_DCDCENHV  - The default configuration of High Voltage DCDC regulator
 

// <i> This settings means only that components for DCDC regulator are installed and it can be enabled.

#define POWER_CONFIG_DEFAULT_DCDCENHV 0

// </e>

// <e> NRFX_POWER_ENABLED - nrfx_power - POWER peripheral driver
//==========================================================
#define NRFX_POWER_ENABLED 1
// <o> NRFX_POWER_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#define NRFX_POWER_CONFIG_IRQ_PRIORITY 6

// <q> NRFX_POWER_CONFIG_DEFAULT_DCDCEN  - The default configuration of main DCDC regulator
 

// <i> This settings means only that components for DCDC regulator are installed and it can be enabled.

#define NRFX_POWER_CONFIG_DEFAULT_DCDCEN 0

// <q> NRFX_POWER_CONFIG_DEFAULT_DCDCENHV  - The default configuration of High Voltage DCDC regulator
 

// <i> This settings means only that components for DCDC regulator are installed and it can be enabled.
#define NRFX_POWER_CONFIG_DEFAULT_DCDCENHV 0
// </e>


// <q> SYSTICK_ENABLED  - nrf_drv_systick - ARM(R) SysTick driver - legacy layer
#define SYSTICK_ENABLED 1

// <q> NRFX_SYSTICK_ENABLED  - nrfx_systick - ARM(R) SysTick driver 
#define NRFX_SYSTICK_ENABLED 1



/*############################################################
     Configuratin to enbale APP SD Card 
#############################################################*/

#define APP_SDCARD_ENABLED          1        // Enable SD Card support 

/* APP_SDCARD_SPI_INSTANCE  - SPI instance used */ 
// <0=> 0 
// <1=> 1 
// <2=> 2 
#define APP_SDCARD_SPI_INSTANCE     2

/* APP_SDCARD_FREQ_INIT  - SPI frequency */
// <33554432=> 125 kHz 
// <67108864=> 250 kHz 
// <134217728=> 500 kHz 
// <268435456=> 1 MHz 
// <536870912=> 2 MHz 
// <1073741824=> 4 MHz 
// <2147483648=> 8 MHz 
#define APP_SDCARD_FREQ_INIT        67108864

/* APP_SDCARD_FREQ_DATA  - SPI frequency */ 
// <33554432=> 125 kHz 
// <67108864=> 250 kHz 
// <134217728=> 500 kHz 
// <268435456=> 1 MHz 
// <536870912=> 2 MHz 
// <1073741824=> 4 MHz 
// <2147483648=> 8 MHz 
#define APP_SDCARD_FREQ_DATA        1073741824

/* Enable SPIM Peripheral driver  */
#define NRFX_SPIM_ENABLED           1   
#define NRFX_SPIM0_ENABLED          0     //SPIM0 Instance
#define NRFX_SPIM1_ENABLED          0     //SPIM1 Instance
#define NRFX_SPIM2_ENABLED          0     //SPIM2 Instance
#define NRFX_SPIM3_ENABLED          0     //SPIM3 Instance

/* NRFX_SPIM_EXTENDED_ENABLED  - Enable extended SPIM features  */
#define NRFX_SPIM_EXTENDED_ENABLED  0

/* NRFX_SPIM_MISO_PULL_CFG  - MISO pin pull configuration. */ 
// <0=> NRF_GPIO_PIN_NOPULL 
// <1=> NRF_GPIO_PIN_PULLDOWN 
// <3=> NRF_GPIO_PIN_PULLUP 
#define NRFX_SPIM_MISO_PULL_CFG     1

/* NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority */ 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 
#define NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY   6


/* Enable SPI peripheral driver */
#define SPI_ENABLED                 1   // Legacy layer
#define SPI0_ENABLED                0   // SPI0 Instance  
#define SPI1_ENABLED                0   // SPI1 Instance  
#define SPI2_ENABLED                1   // SPI2 Instance  

#define NRFX_SPI_ENABLED            1   // Enanle NRFX SPI driver
#define NRFX_SPI0_ENABLED           0   // NRFX SPI0 instance
#define NRFX_SPI1_ENABLED           0   // NRFX SPI1 instance
#define NRFX_SPI2_ENABLED           0   // NRFX SPI2 instance

/* Enable DMA */
#define SPI2_USE_EASY_DMA           1   


// <o> NRFX_SPI_MISO_PULL_CFG  - MISO pin pull configuration. 
// <0=> NRF_GPIO_PIN_NOPULL 
// <1=> NRF_GPIO_PIN_PULLDOWN 
// <3=> NRF_GPIO_PIN_PULLUP 
#define NRF_SPI_DRV_MISO_PULLUP_CFG 1
#define NRFX_SPI_MISO_PULL_CFG      1

/* <o> NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority */ 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 
#define SPI_DEFAULT_CONFIG_IRQ_PRIORITY         6
#define NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY    6



//==========================================================
// <e> NRFX_TIMER_ENABLED - nrfx_timer - TIMER periperal driver
//==========================================================
#ifndef NRFX_TIMER_ENABLED
#define NRFX_TIMER_ENABLED 1
#endif
// <q> NRFX_TIMER0_ENABLED  - Enable TIMER0 instance
 

#ifndef NRFX_TIMER0_ENABLED
#define NRFX_TIMER0_ENABLED 1
#endif

// <q> NRFX_TIMER1_ENABLED  - Enable TIMER1 instance
 

#ifndef NRFX_TIMER1_ENABLED
#define NRFX_TIMER1_ENABLED 1
#endif

// <q> NRFX_TIMER2_ENABLED  - Enable TIMER2 instance
 

#ifndef NRFX_TIMER2_ENABLED
#define NRFX_TIMER2_ENABLED 1
#endif

// <q> NRFX_TIMER3_ENABLED  - Enable TIMER3 instance
 

#ifndef NRFX_TIMER3_ENABLED
#define NRFX_TIMER3_ENABLED 1
#endif

// <q> NRFX_TIMER4_ENABLED  - Enable TIMER4 instance
 

#ifndef NRFX_TIMER4_ENABLED
#define NRFX_TIMER4_ENABLED 1
#endif

// <o> NRFX_TIMER_DEFAULT_CONFIG_FREQUENCY  - Timer frequency if in Timer mode
 
// <0=> 16 MHz 
// <1=> 8 MHz 
// <2=> 4 MHz 
// <3=> 2 MHz 
// <4=> 1 MHz 
// <5=> 500 kHz 
// <6=> 250 kHz 
// <7=> 125 kHz 
// <8=> 62.5 kHz 
// <9=> 31.25 kHz 

#ifndef NRFX_TIMER_DEFAULT_CONFIG_FREQUENCY
#define NRFX_TIMER_DEFAULT_CONFIG_FREQUENCY 4
#endif

// <o> NRFX_TIMER_DEFAULT_CONFIG_MODE  - Timer mode or operation
 
// <0=> Timer 
// <1=> Counter 

#ifndef NRFX_TIMER_DEFAULT_CONFIG_MODE
#define NRFX_TIMER_DEFAULT_CONFIG_MODE 0
#endif

// <o> NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH  - Timer counter bit width
 
// <0=> 16 bit 
// <1=> 8 bit 
// <2=> 24 bit 
// <3=> 32 bit 

#ifndef NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH
#define NRFX_TIMER_DEFAULT_CONFIG_BIT_WIDTH 3
#endif

// <o> NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY
#define NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY 6
#endif

// <e> NRFX_TIMER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_TIMER_CONFIG_LOG_ENABLED
#define NRFX_TIMER_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_TIMER_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_TIMER_CONFIG_LOG_LEVEL
#define NRFX_TIMER_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_TIMER_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_TIMER_CONFIG_INFO_COLOR
#define NRFX_TIMER_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_TIMER_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_TIMER_CONFIG_DEBUG_COLOR
#define NRFX_TIMER_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// </e>

// <e> TIMER_ENABLED - nrf_drv_timer - TIMER periperal driver - legacy layer
//==========================================================
#ifndef TIMER_ENABLED
#define TIMER_ENABLED 1
#endif
// <o> TIMER_DEFAULT_CONFIG_FREQUENCY  - Timer frequency if in Timer mode
 
// <0=> 16 MHz 
// <1=> 8 MHz 
// <2=> 4 MHz 
// <3=> 2 MHz 
// <4=> 1 MHz 
// <5=> 500 kHz 
// <6=> 250 kHz 
// <7=> 125 kHz 
// <8=> 62.5 kHz 
// <9=> 31.25 kHz 

#ifndef TIMER_DEFAULT_CONFIG_FREQUENCY
#define TIMER_DEFAULT_CONFIG_FREQUENCY 4
#endif

// <o> TIMER_DEFAULT_CONFIG_MODE  - Timer mode or operation
 
// <0=> Timer 
// <1=> Counter 

#ifndef TIMER_DEFAULT_CONFIG_MODE
#define TIMER_DEFAULT_CONFIG_MODE 0
#endif

// <o> TIMER_DEFAULT_CONFIG_BIT_WIDTH  - Timer counter bit width
 
// <0=> 16 bit 
// <1=> 8 bit 
// <2=> 24 bit 
// <3=> 32 bit 

#ifndef TIMER_DEFAULT_CONFIG_BIT_WIDTH
#define TIMER_DEFAULT_CONFIG_BIT_WIDTH 3
#endif

// <o> TIMER_DEFAULT_CONFIG_IRQ_PRIORITY  - Interrupt priority
 

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef TIMER_DEFAULT_CONFIG_IRQ_PRIORITY
#define TIMER_DEFAULT_CONFIG_IRQ_PRIORITY 6
#endif

// <q> TIMER0_ENABLED  - Enable TIMER0 instance
 

#ifndef TIMER0_ENABLED
#define TIMER0_ENABLED 1
#endif

// <q> TIMER1_ENABLED  - Enable TIMER1 instance
 

#ifndef TIMER1_ENABLED
#define TIMER1_ENABLED 1
#endif

// <q> TIMER2_ENABLED  - Enable TIMER2 instance
 

#ifndef TIMER2_ENABLED
#define TIMER2_ENABLED 1
#endif

// <q> TIMER3_ENABLED  - Enable TIMER3 instance
 

#ifndef TIMER3_ENABLED
#define TIMER3_ENABLED 1
#endif

// <q> TIMER4_ENABLED  - Enable TIMER4 instance
 

#ifndef TIMER4_ENABLED
#define TIMER4_ENABLED 1
#endif

// </e>


#define HARDFAULT_HANDLER_ENABLED 1U


// <e> NRF_STACK_GUARD_ENABLED - nrf_stack_guard - Stack guard
//==========================================================
#define NRF_STACK_GUARD_ENABLED 1

// <o> NRF_STACK_GUARD_CONFIG_SIZE  - Size of the stack guard.
 
// <5=> 32 bytes 
// <6=> 64 bytes 
// <7=> 128 bytes 
// <8=> 256 bytes 
// <9=> 512 bytes 
// <10=> 1024 bytes 
// <11=> 2048 bytes 
// <12=> 4096 bytes 

#define NRF_STACK_GUARD_CONFIG_SIZE 7

// </e>


// </e>



// <e> NRFX_PDM_ENABLED - nrfx_pdm - PDM peripheral driver
//==========================================================
#ifndef NRFX_PDM_ENABLED
#define NRFX_PDM_ENABLED 1
#endif
// <o> NRFX_PDM_CONFIG_MODE  - Mode
 
// <0=> Stereo 
// <1=> Mono 

#ifndef NRFX_PDM_CONFIG_MODE
#define NRFX_PDM_CONFIG_MODE 1
#endif

// <o> NRFX_PDM_CONFIG_EDGE  - Edge
 
// <0=> Left falling 
// <1=> Left rising 

#ifndef NRFX_PDM_CONFIG_EDGE
#define NRFX_PDM_CONFIG_EDGE 0
#endif

// <o> NRFX_PDM_CONFIG_CLOCK_FREQ  - Clock frequency
 
// <134217728=> 1000k 
// <138412032=> 1032k (default) 
// <142606336=> 1067k 

#ifndef NRFX_PDM_CONFIG_CLOCK_FREQ
#define NRFX_PDM_CONFIG_CLOCK_FREQ 138412032
#endif

// <o> NRFX_PDM_CONFIG_IRQ_PRIORITY  - Interrupt priority
 
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 

#ifndef NRFX_PDM_CONFIG_IRQ_PRIORITY
#define NRFX_PDM_CONFIG_IRQ_PRIORITY 6
#endif

// <e> NRFX_PDM_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NRFX_PDM_CONFIG_LOG_ENABLED
#define NRFX_PDM_CONFIG_LOG_ENABLED 0
#endif
// <o> NRFX_PDM_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NRFX_PDM_CONFIG_LOG_LEVEL
#define NRFX_PDM_CONFIG_LOG_LEVEL 3
#endif

// <o> NRFX_PDM_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_PDM_CONFIG_INFO_COLOR
#define NRFX_PDM_CONFIG_INFO_COLOR 0
#endif

// <o> NRFX_PDM_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NRFX_PDM_CONFIG_DEBUG_COLOR
#define NRFX_PDM_CONFIG_DEBUG_COLOR 0
#endif

//==========================================================



#endif   // APP_CONFIG_H