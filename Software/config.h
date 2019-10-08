/***************************************************************************//**
 * @file
 * @brief	Project configuration file
 * @author	Ralf Gerhauser
 * @version	2016-02-27
 *
 * This file allows to set miscellaneous configuration parameters.  It must be
 * included by all modules.
 *
 ****************************************************************************//*
Revision History:
2016-02-26,rage	Increased LOG_BUF_SIZE to 4KB.
2016-02-10,rage	Set DFLT_RFID_POWER_OFF_TIMEOUT to 6 minutes.
2014-11-11,rage	Derived from project "AlarmClock".
*/

#ifndef __INC_config_h
#define __INC_config_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"

/*=============================== Definitions ================================*/

/*
 * Basic defines
 */
    /* terminators for lists and strings */

#define	EOL		NULL		/* EndOfList		*/
#define EOS		'\0'		/* EndOfString		*/
#define	NONE		(-1)

    /* macro to calculate the number of elements of an array */
#define ELEM_CNT(array)  (sizeof (array) / sizeof ((array)[0]))

/*
 * LED Definitions for this project
 */
    /*!@brief GPIO Port of the (red) Power-LED. */
#define POWER_LED_PORT		gpioPortA
    /*!@brief GPIO Pin of the (red) Power-LED is PA2. */
#define POWER_LED_PIN		2
    /*! @brief Macro to set or clear the Power-LED */
#define POWER_LED   IO_Bit(GPIO->P[POWER_LED_PORT].DOUT, POWER_LED_PIN)
    /*!@brief GPIO Port of the (green) Log Flush LED. */
#define LOG_FLUSH_LED_PORT	gpioPortA
    /*!@brief GPIO Pin of the (green) Log Flush LED is PA5. */
#define LOG_FLUSH_LED_PIN	5		//! State: 0=OFF, 1=ON
    /*! @brief Macro to set or clear the Log Flush LED */
#define LOG_FLUSH_LED IO_Bit(GPIO->P[LOG_FLUSH_LED_PORT].DOUT, LOG_FLUSH_LED_PIN)

/*!
 * @brief MPU Clock Configuration.
 *
 * Set to 0 to use the internal RC oscillator, if 1 the external 32MHz XTAL
 * is used.  The frequency of the RC oscillator is 14MHz per default.
 */
#define USE_EXT_32MHZ_CLOCK	1

/*
 * Configuration for module "AlarmClock"
 */
    /*!@brief RTC frequency in [Hz]. */
#define RTC_COUNTS_PER_SEC	32768


/*!
 * @brief Interrupt Priority Settings
 *
 * There are 8 priority levels 0 to 7 with 0 to be the highest and 7 to be
 * the lowest priority.  DCF77 (which is called from the EXTI handler) and RTC
 * must use the same priority level to lock-out each other, because they
 * both use function localtime() and this is not multithreading save.
 * Funktion localtime_r() would be the right choice here, unfortunately it
 * is not available with the IAR compiler library.
 */
#define INT_PRIO_UART	2		//!<  UART IRQs for RFID and Scales
#define INT_PRIO_LEUART	2		//!<  LEUART RX interrupt (not used)
#define INT_PRIO_DMA	2		//!<  DMA is used for LEUART
#define INT_PRIO_SMB	2		//!<  SMBus used by the battery monitor
#define INT_PRIO_RTC	3		//!<  lower priority than others
#define INT_PRIO_EXTI	INT_PRIO_RTC	//!<  must be the same as @ref INT_PRIO_RTC


/*
 * Configuration for Atomic Clock module "DCF77.c"
 */
    /*!@brief Use Hardware Enable pin of the DCF77 receiver. */
#define DCF77_HARDWARE_ENABLE	1

    /*!@brief Activate DCF77 only once per day. */
#define DCF77_ONCE_PER_DAY	1

    /*!@brief Call ShowDCF77Indicator() to flash red LED on DCF77 signal. */
#define DCF77_INDICATOR		1


/*
 * Configuration for module "RFID"
 */
    /*!@brief Flag, if RFID reader is triggered by a light barrier. */
#define RFID_TRIGGERED_BY_LIGHT_BARRIER	0	// NO light barrier present


/*
 * Configuration for module "Logging"
 */
    /*!@brief Size of the log buffer in bytes. */
#define LOG_BUF_SIZE	4096

    /*!@brief Use this define to specify a function to be called for monitoring
     * the log activity.  Here, monitoring is done via the LEUART interface.
     */
#define LOG_MONITOR_FUNCTION	drvLEUART_puts

/* forward declaration */
void    drvLEUART_puts(const char *str);

    /*!@brief Disable "alive" message by setting this interval to 0. */
#define LOG_ALIVE_INTERVAL	0


/*!@name DMA Channel Assignment
 *
 * The following definitions assign the 8 DMA channels to the respective
 * devices or drivers.  These defines are used as index within the global
 * DMA_DESCRIPTOR_TypeDef structure @ref g_DMA_ControlBlock.
 */
//@{
#define DMA_CHAN_LEUART_RX	0	//!< LEUART Rx uses DMA channel 0
#define DMA_CHAN_LEUART_TX	1	//!< LEUART Tx uses DMA channel 1
//@}


/*================================== Macros ==================================*/

#ifdef DEBUG
    /*
     * Debugging output via ITM or LEUART
     */
    #if DEBUG_VIA_ITM
	#define DBG_PUTC(ch)	ITM_SendChar(ch)
	#define DBG_PUTS(str)	ITM_SendStr(str)
	uint32_t ITM_SendChar (uint32_t ch);
	void ITM_SendStr(const char *pStr);
    #else
	#define DBG_PUTC(ch)	drvLEUART_putc(ch)
	#define DBG_PUTS(str)	drvLEUART_puts(str)
	void	drvLEUART_putc(char ch);
	void	drvLEUART_puts(const char *str);
    #endif
    void dbgInit(void);
#else
    #define DBG_PUTC(ch)
    #define DBG_PUTS(str)
#endif

#define INCLUDE_DEBUG_TRACE 0
#if INCLUDE_DEBUG_TRACE
    #define DEBUG_TRACE(id)	DebugTrace(id)
    #define DEBUG_TRACE_STOP	DebugTraceStop()
    void DebugTrace(uint32_t id);
    void DebugTraceStop(void);
    #undef  LOG_BUF_SIZE
    #define LOG_BUF_SIZE	2048
    #define DEBUG_TRACE_COUNT	512
#else
    #define DEBUG_TRACE(id)
    #define DEBUG_TRACE_STOP
#endif

    /*! Macro to address a single bit in the I/O range (peripheral range) in
     *  an atomic manner.
     * @param address   I/O register address.
     * @param bitNum    Bit number within this register.
     */
#define IO_BIT_ADDR(address, bitNum)					\
	((__IO uint32_t *) (BITBAND_PER_BASE				\
			+ (((uint32_t)(address)) - PER_MEM_BASE) * 32	\
			+ (bitNum) * 4))

    /*! Shortcut to directly access an I/O-bit. */
#define IO_Bit(regName, bitNum)	*IO_BIT_ADDR(&regName, bitNum)

    /*! Macro to address a single bit in an SRAM variable in an atomic manner.
     * @param address   Address of the variable in SRAM.
     * @param bitNum    Bit number within this variable.
     */
#define SRAM_BIT_ADDR(address, bitNum)					\
	((__IO uint32_t *) (BITBAND_RAM_BASE				\
			+ (((uint32_t)(address)) - RAM_MEM_BASE) * 32	\
			+ (bitNum) * 4))

    /*! Shortcut to directly access a bit in a variable. */
#define Bit(varName, bitNum)	*SRAM_BIT_ADDR(&varName, bitNum)

/*=========================== Typedefs and Structs ===========================*/

/*!@brief Structure to hold Project Information */
typedef struct
{
    char const  ID[12];
    char const  Date[16];
    char const  Time[10];
    char const  Version[16];
} PRJ_INFO;


/*!@brief Enumeration of Alarm Identifiers
 *
 * This is the list of Alarm IDs used by this application.  They are used to
 * identify a particular alarm time entry via the <b>alarmNum</b> parameter
 * when calling alarm functions, e.g. AlarmSet().
 */
typedef enum
{
    ALARM_DCF77_WAKE_UP,    //!< Wake up DCF77 to synchronize the system clock
    ALARM_BATTERY_MON_1,    //!< Time #1 for logging battery status
    ALARM_BATTERY_MON_2,    //!< Time #2 for logging battery status
    ALARM_ON_TIME_1,        //!< Time #1 when to switch the system ON
    ALARM_ON_TIME_2,        //!< Time #2 when to switch the system ON
    ALARM_ON_TIME_3,        //!< Time #3 when to switch the system ON
    ALARM_ON_TIME_4,        //!< Time #4 when to switch the system ON
    ALARM_ON_TIME_5,        //!< Time #5 when to switch the system ON
    ALARM_OFF_TIME_1,       //!< Time #1 when to switch te system OFF
    ALARM_OFF_TIME_2,       //!< Time #2 when to switch te system OFF
    ALARM_OFF_TIME_3,       //!< Time #3 when to switch te system OFF
    ALARM_OFF_TIME_4,       //!< Time #4 when to switch te system OFF
    ALARM_OFF_TIME_5,       //!< Time #5 when to switch te system OFF
    NUM_ALARM_IDS
} ALARM_ID;

/*
 * !@brief These defines hold the first and last alarm time ENUM for Power
 * Output control, and the number of alarm ON, resp. OFF times.
 */
//@{
#define FIRST_POWER_ALARM	ALARM_ON_TIME_1
#define LAST_POWER_ALARM	ALARM_OFF_TIME_5
#define NUM_POWER_ALARMS	(LAST_POWER_ALARM - ALARM_OFF_TIME_1 + 1)
//@}

/*!@brief Enumeration of the EM1 Modules
 *
 * This is the list of Software Modules that require EM1 to work, i.e. they
 * will not work in EM2 because clocks, etc. would be disabled.  These enums
 * are used to set/clear the appropriate bit in the @ref g_EM1_ModuleMask.
 */
typedef enum
{
    EM1_MOD_RFID,	//!<  0: The RFID Module uses UART#1
    EM1_MOD_SCALES,	//!<  1: The Scales Module uses UART#0
    END_EM1_MODULES
} EM1_MODULES;

/*!@brief Enumeration of Error Bits
 *
 * This is the list of error sources, i.e. these enums identify sources for
 * system errors.  See also SetError() and ClearError().
 */
typedef enum
{
    ERR_SRC_SCALES,	//!<  0: The Scales Module reports an error
    END_ERR_SRC
} ERR_SRC;

/*======================== External Data and Routines ========================*/

extern volatile bool	 g_flgIRQ;		// Flag: Interrupt occurred
extern volatile uint16_t g_EM1_ModuleMask;	// Modules that require EM1

/*================================ Prototypes ================================*/

    /* Set source of a system error */
void	SetError (ERR_SRC errorSource);

    /* Clear source of a system error */
void	ClearError (ERR_SRC errorSource);


#endif /* __INC_config_h */
