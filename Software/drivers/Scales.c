/***************************************************************************//**
 * @file
 * @brief	SCALES
 * @author	Peter Loes
 * @version	2019-06-04
 *
 * This module provides the functionality to communicate with the SCALES.
 * It contains the following parts:
 * - USART driver to transmit and receive data from the Amplifier
 * - Handler for the received data
 * - Power management for Amplifier DAD 141.1
 *
 * After powering up the Scales System, the following actions are performed:
 * -# Initialization of USART and the related Rx and Tx pins.
 * -# Configuring the Rx pin with a pull-up resistor to prevent the basic state
 *    of the Rx signal to go low again (Scale's firmware releases the signal
 *    which tends to low).
 * -# Waiting @ref POWER_UP_DELAY seconds for Scales being ready.
 * -# Sending <b>ID</b> command to get the type of hardware (e.g. 1411).
 * -# Sending <b>IH</b> command to get the hardware version (e.g. 14100101).
 * -# Sending <b>IV</b> command to get the firmware version (e.g. 0101).
 * -# Sending <b>RS</b> command to get the serial number (e.g. 0147301).
 * -# Sending <b>TW</b> and <b>TI</b> commands to activate <i>automatic tare</i>
 *    (if the variables @ref SCALES_CFG_TW and @ref SCALES_CFG_TI have been set
 *    in the configuration file <a href="../../CONFIG.TXT"><i>CONFIG.TXT</i></a>).
 * -# Sending <b>NR</b> and <b>NT</b> commands to configure the maximum weight
 *    deviation in [g] over time in [ms] for detecting <i>silence</i>.
 *    These must be set by the configuration variables @ref SCALES_CFG_NR and
 *    @ref SCALES_CFG_NT.
 * -# Sending <b>SD</b> and <b>MT</b> commands which configure the <i>start
 *    delay</i> and <i>measure time</i> in [ms].  These must be set by the
 *    configuration variables @ref SCALES_CFG_SD and @ref SCALES_CFG_MT.
 * -# Sending <b>TL</b> command to specify a trigger level in gram [g].  This
 *    value must be specified by variable @ref SCALES_CFG_TL.  The Scales
 *    firmware automatically starts a measure cycle when this trigger level has
 *    been exceeded.
 * -# Sending <b>ST</b> command (set tare) to change mode from "brutto" (raw)
 *    to "netto" (tare value is subtracted).  This is possible only if the
 *    silence criteria is met.  If this is not the case, the command returns
 *    error and will be retried after @ref ST_RETRY_DELAY seconds for up to
 *    @ref MAX_ST_RETRY_CNT times.
 * -# After the <b>ST</b> command has been executed successfully, the <b>SA</b>
 *    command will be send to enable auto-send mode of weight measuring.
 * -# Now the Scales system has entered operational state.  Measurements are
 *    reported according to the actual configuration.
 *
 * @note
 * The DAD 141.1 provides many configuration parameters and only a few can be
 * set via <i>CONFIG.TXT</i>.  The vast majority must be pre-configured and
 * stored in the amplifier's EEPROM, especially the following parameters and
 * its value:
 * - FM 3
 * - UR 7
 *
 * The manual often states a unit of <b>(d)</b> which means <b>digits</b>.
 * These are pure digits without any decimal point.
 *
 * @internal
 * As long as @ref l_flgScalesActivate is false, scales cannot be enabled.
 * The variable is set true after the configuration file has been read and
 * ScalesInit() has been called.
 * @endinternal
 ****************************************************************************//*
Revision History:
2019-07-01,rage	Implemented retry mechanism for "ST" command.
2019-05-22,rage	Introduced automatic tare handling.
2018-11-15,rage	Changed serial driver (avoid requirement for atomic execution).
2018-07-16,rage	Reworked power management and interrupt handling.
2017-11-08,Loës	Initial version.
*/

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <string.h>
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "em_emu.h"
#include "config.h"		// include project configuration parameters
#include "AlarmClock.h"
#include "Scales.h"
#include "Logging.h"
#include "Control.h"

/*=============================== Definitions ================================*/

    // Module Debugging
#define MOD_DEBUG	0	// set 1 to enable debugging of this module

    /*!@brief Internal logical states of the Scales system. */
typedef enum
{
    SCALES_STATE_OFF,		//!<  0: Scales system is OFF
    SCALES_STATE_POWER_ON,	//!<  1: Scales system is powered on now
    SCALES_STATE_GET_ID,	//!<  2: Get Scale's ID (hardware type)
    SCALES_STATE_GET_HW_VERS,	//!<  3: Get Scale's Hardware Version
    SCALES_STATE_GET_FW_VERS,	//!<  4: Get Scale's Firmware Version
    SCALES_STATE_GET_SN,	//!<  5: Get Scale's Serial Number
    SCALES_STATE_SEND_TW,	//!<  6: Send Tare Window [g]
    SCALES_STATE_SEND_TI,	//!<  7: Send Tare Interval [ms]
    SCALES_STATE_SEND_NR,	//!<  8: Send Nominal silence (Ruhe) value [g]
    SCALES_STATE_SEND_NT,	//!<  9: Send Nominal silence Time [ms]
    SCALES_STATE_SEND_SD,	//!< 10: Send Start Delay [ms]
    SCALES_STATE_SEND_MT,	//!< 11: Send Measure Time [ms]
    SCALES_STATE_SEND_TL,	//!< 12: Send Trigger Level [g]
    SCALES_STATE_SEND_ST,	//!< 13: Send Set Tare
    SCALES_STATE_SEND_SA,	//!< 14: [S]end (Mean Value) [A]utomatically
    SCALES_STATE_OPERATIONAL,	//!< 15: Scales system is operational
    SCALES_STATE_RECOVER,	//!< 16: Try to recover after communication error
    END_SCALES_STATE
} SCALES_STATE;

    /*!@brief Time in [s] to wait for Scales being ready after power-up. */
#define POWER_UP_DELAY		15

    /*!@brief Maximum Communication Error Count before giving up. */
#define MAX_COM_ERROR_CNT	10

    /*!@brief Maximum Retry Count for the "ST" command. */
#define MAX_ST_RETRY_CNT	 5

    /*!@brief Delay in [s] before sending the "ST" command again. */
#define ST_RETRY_DELAY		 5

/*=========================== Typedefs and Structs ===========================*/

/*!@brief Local structure to hold UART specific parameters */
typedef struct
{
    USART_TypeDef *	   const UART;		//!< UART device to use
    CMU_Clock_TypeDef	   const cmuClock_UART;	//!< CMU clock for the UART
    IRQn_Type		   const UART_Rx_IRQn;	//!< Rx interrupt number
    GPIO_Port_TypeDef	   const UART_Rx_Port;	//!< Port for RX pin
    uint32_t		   const UART_Rx_Pin;	//!< Rx pin on this port
    IRQn_Type		   const UART_Tx_IRQn;	//!< Tx interrupt number
    GPIO_Port_TypeDef	   const UART_Tx_Port;	//!< Port for TX pin
    uint32_t		   const UART_Tx_Pin;	//!< Tx pin on this port
    uint32_t		   const UART_Route;	//!< Route location
    uint32_t		   const Baudrate;	//!< Baudrate for the Scales
    USART_Databits_TypeDef const DataBits;	//!< Number of data bits
    USART_Parity_TypeDef   const Parity;	//!< Parity mode
    USART_Stopbits_TypeDef const StopBits;	//!< Number of stop bits
} USART_ParmsScales;


/*========================= Global Data and Routines =========================*/

    /*!@brief Scales power output. */
PWR_OUT   g_ScalesPower = PWR_OUT_NONE;

    /*!@brief Tare Window: Value in gram [g] the ADC measurements must not
     * exceed during automatic tare. */
uint32_t  g_ScalesCfg_TW;

    /*!@brief Tare Interval: Duration in [ms] to build a new tare value. */
uint32_t  g_ScalesCfg_TI;

    /*!@brief Nominal silence (Ruhe): Value in gram [g] the ADC measurements
     * must be stable to be treated as valid. */
uint32_t  g_ScalesCfg_NR;

    /*!@brief Nominal silence Time: Duration in [ms] the ADC measurements must
     * be stable to be treated as valid. */
uint32_t  g_ScalesCfg_NT;

    /*!@brief Trigger Level: Value in gram [g] the cyclic measurement must
     * exceed to trigger a measurement that is reported. */
uint32_t  g_ScalesCfg_TL;

    /*!@brief Start Delay: Duration in [ms] before measurement starts. */
uint32_t  g_ScalesCfg_SD;

    /*!@brief Measure Time: Duration in [ms] measuring is performed to build
     * a mean value. */
uint32_t  g_ScalesCfg_MT;

/*================================ Local Data ================================*/

    /*!@brief Retrieve information after Scales has been initialized. */
static bool	 l_flgInit;

    /*!@brief Flag that determines if Scales is in use. */
static bool	 l_flgScalesActivate;

/*! USART parameters for Scales communication */
static const USART_ParmsScales l_Scales_USART =
{
    USART0, cmuClock_USART0,		//!< select USART0
    USART0_RX_IRQn, gpioPortE, 11,	//!< Rx is PE11
    USART0_TX_IRQn, gpioPortE, 10,	//!< Tx is PE10
    USART_ROUTE_LOCATION_LOC0,		//!< routed thru location #0
    9600, usartDatabits8,		//!< Communication parameters for the
    usartNoParity, usartStopbits1	//!< DAD 141.1: 9600/8/N/1
};

    /*! Flag if Scales should be powered on. */
static volatile bool	l_flgScalesOn;

    /*! Flag if Scales is currently powered on. */
static volatile bool	l_flgScalesIsOn;

    /*! Current state of the Scales system. */
static volatile SCALES_STATE l_State;

    /*! Timer handle for "Communication Watchdog". */
static volatile TIM_HDL	l_hdlWdog = NONE;

    /*! Timer handle for "Command Retry". */
static volatile TIM_HDL	l_hdlCmdRetry = NONE;

    /*! Variables for the communication with the Scales amplifier. */
static char	l_TxBuffer[10];		//!< Transmit buffer
static volatile uint8_t	l_TxIdx;	//!< Index within the transmit buffer
static volatile bool	l_flgTxComplete;//!< true: Command has been sent
static volatile uint8_t	l_ComErrorCnt;	//!< Communication Error Count
static volatile uint8_t	l_ST_RetryCnt;	//!< "ST" command Retry Count
static char	l_RxBuffer[150];	//!< Receive buffer
static volatile uint8_t	l_RxIdx;	//!< Index within the receive buffer

/*=========================== Forward Declarations ===========================*/

    /*! Scales Communication Timeout */
static void ScalesComTimeout(TIM_HDL hdl);

    /*! Scales Command Retry */
static void ScalesCmdRetry(TIM_HDL hdl);

    /* Power On Scales */
static void ScalesPowerOn (void);

    /*! Start sending a Command Sequence to Scales. */
static void ScalesSendCmdSeq (SCALES_STATE state);

    /*! Check Scales Data */
static void CheckScalesData (void);

    /*! Scales UART Setup Routine */
static void ScalesUartSetup(void);


/***************************************************************************//**
 *
 * @brief	Initialize Scales system
 *
 * This routine initializes the GPIO pins which are connected to the external
 * SCALES-Amp.  The GPIO ports and pins have been defined in the header file.
 *
 ******************************************************************************/
void	ScalesInit (void)
{
    /* Check if scales is already in use */
    if (l_flgScalesActivate)
	ScalesPowerOff();	// power-off and reset scales and UART

    /* Now the scales isn't active any more */
    l_flgScalesActivate = false;

    if (g_ScalesPower == PWR_OUT_NONE)
	return;

    /* Scales should be activated and initialized */
    l_flgScalesActivate = true;
    l_flgInit = true;

#ifdef LOGGING
    Log ("Initializing Scales for Power Output %s",
	 g_enum_PowerOutput[g_ScalesPower]);

    if (g_ScalesCfg_TW > 0  &&  g_ScalesCfg_TI > 0)
    {
	Log ("Scales auto-tare uses a Tare Window TW of %ldg and a Tare"
	     " Interval TI of %ldms", g_ScalesCfg_TW, g_ScalesCfg_TI);
    }
    else
    {
	Log ("Scales auto-tare function is disabled");
    }

    Log ("Scales Nominal silence (Ruhe) value NR for measurement is %ldg",
	 g_ScalesCfg_NR);
#endif
    if (g_ScalesCfg_NR < 1  ||  g_ScalesCfg_NR > 65535)
	LogError("Scales: NR value must be between 1 and 65535 g");

#ifdef LOGGING
    Log ("Scales Nominal silence Time NT for measurement is %ldms",
	 g_ScalesCfg_NT);
#endif
    if (g_ScalesCfg_NT < 1  ||  g_ScalesCfg_NT > 65535)
	LogError("Scales: NT value must be between 1 and 65535 ms");

#ifdef LOGGING
    Log ("Scales Trigger Level TL for measurement is %ldg", g_ScalesCfg_TL);
#endif
    if (g_ScalesCfg_TL < 10)
	LogError("Scales: Trigger Level TL must be set to a minimum of 10g");

#ifdef LOGGING
    if (g_ScalesCfg_SD > 0  ||  g_ScalesCfg_MT > 0)
    {
	Log ("Scales mean value calculation uses a Start Delay SD of %ldms and"
	     " a Measure Time MT of %ldms", g_ScalesCfg_SD, g_ScalesCfg_MT);
    }
    else
    {
	Log ("Scales mean value calculation is disabled");
    }
#endif

    /* Create timer for a "Communication Watchdog" */
    if (l_hdlWdog == NONE)
	l_hdlWdog = sTimerCreate (ScalesComTimeout);

    /* Create timer for "Command Retry" */
    if (l_hdlCmdRetry == NONE)
	l_hdlCmdRetry = sTimerCreate (ScalesCmdRetry);
}


/***************************************************************************//**
 *
 * @brief	Enable Scales
 *
 * This routine enables the scales, i.e. it notifies the scales software
 * module to power up and initialize the scales and the related hardware.
 * It is usually called by PowerControl().
 *
 * @see ScalesDisable(), ScalesTimedDisable(), ScalesCheck()
 *
 ******************************************************************************/
void ScalesEnable (void)
{
    /* initiate power-on of the Scales hardware */
    l_flgScalesOn = true;
}


/***************************************************************************//**
 *
 * @brief	Disable Scales
 *
 * This routine immediately disables the scales.
 *
 * @see ScalesEnable(), ScalesTimedDisable(), ScalesCheck()
 *
 ******************************************************************************/
void ScalesDisable (void)
{
    if (l_flgScalesOn)
    {
	l_flgScalesOn = false;

	/* Scales should be powered OFF */
	if (l_flgScalesIsOn)
	{
	    ScalesPowerOff();
	    l_flgScalesIsOn = false;
	}
    }
}


/***************************************************************************//**
 *
 * @brief	Power Scales On
 *
 * This routine powers the Scales on and initializes the related hardware.
 *
 ******************************************************************************/
static void ScalesPowerOn (void)
{
    if (l_flgScalesActivate)
    {
#ifdef LOGGING
	/* Generate Log Message */
	Log ("Scales is powered ON");
#endif
	/* (Re-)initialize variables */
	l_flgTxComplete = false;
	l_ST_RetryCnt = l_RxIdx = l_TxIdx = 0;

	/* Module Scales requires EM1, set bit in bit mask */
	Bit(g_EM1_ModuleMask, EM1_MOD_SCALES) = 1;

	/* Prepare UART to communicate with Scales */
	ScalesUartSetup();

	/* Set Power Enable Pin for the scales hardware to ON */
	PowerOutput (g_ScalesPower, PWR_ON);

	/* Wait some time until Scales is up and running */
	l_State = SCALES_STATE_POWER_ON;
	if (l_hdlWdog != NONE)
	    sTimerStart (l_hdlWdog, POWER_UP_DELAY);

#ifdef LOGGING
	/* Generate Log Message */
	Log ("Waiting %ds for Scales being ready to accept commands...",
	     POWER_UP_DELAY);
#endif
    }
}


/***************************************************************************//**
 *
 * @brief	Scales Check
 *
 * This routine is called from Control.c to get the weight of the birds.
 * After we received a transponder number. The weighting cell reacts
 * strong to environmental effects, like wind!
 ******************************************************************************/
void	ScalesCheck (void)
{
    if (l_flgScalesOn)
    {
	/* Scales should be powered ON */
	if (! l_flgScalesIsOn)
	{
	    ScalesPowerOn();
	    l_flgScalesIsOn = true;
	}
    }
    else
    {
	/* Scales should be powered OFF */
	if (l_flgScalesIsOn)
	{
	    ScalesPowerOff();
	    l_flgScalesIsOn = false;
	}
    }
}


/***************************************************************************//**
 *
 * @brief	Power Off Scales
 *
 * This routine powers the Scales immediately off.
 *
 ******************************************************************************/
void ScalesPowerOff (void)
{
    /* Set Power Enable Pin for the Scales to OFF */
    PowerOutput (g_ScalesPower, PWR_OFF);
    l_State = SCALES_STATE_OFF;

    /* Clear Scales-related error conditions */
    ClearError(ERR_SRC_SCALES);

    /* Disable clock for USART module */
    CMU_ClockEnable(l_Scales_USART.cmuClock_UART, false);

    /* Disable Rx and Tx pins */
    GPIO_PinModeSet(l_Scales_USART.UART_Rx_Port,
		    l_Scales_USART.UART_Rx_Pin, gpioModeDisabled, 0);
    GPIO_PinModeSet(l_Scales_USART.UART_Tx_Port,
		    l_Scales_USART.UART_Tx_Pin, gpioModeDisabled, 0);

    /* Module Scales is no longer active, clear bit in bit mask */
    Bit(g_EM1_ModuleMask, EM1_MOD_SCALES) = 0;

#ifdef LOGGING
    /* Generate Log Message */
    Log ("Scales is powered off");
#endif
}


/***************************************************************************//**
 *
 * @brief	Scales Power Fail Handler
 *
 * This function will be called in case of power-fail to bring the Scales
 * hardware into a quiescent, power-saving state.
 *
 ******************************************************************************/
void	ScalesPowerFailHandler (void)
{
    /* Switch Scales off */
    l_flgScalesOn = false;

    if (l_flgScalesIsOn)
    {
	ScalesPowerOff();
	l_flgScalesIsOn = false;
    }
}


/***************************************************************************//**
 *
 * @brief	Scales Communication Timeout
 *
 * This routine is called from the RTC interrupt handler, after the specified
 * amount of time has elapsed to notify a "Communication Timeout" with the
 * scales amplifier.  This means the firmware did not respond within time.
 * The error is logged, then the recovery of the scales system is initiated.
 *
 ******************************************************************************/
static void ScalesComTimeout(TIM_HDL hdl)
{
SCALES_STATE	startState;

    (void) hdl;		// suppress compiler warning "unused parameter"

    /* Check error count */
    if (l_ComErrorCnt > MAX_COM_ERROR_CNT)
    {
	l_State = SCALES_STATE_OFF;
	ScalesDisable();
	return;
    }

    if (l_State == SCALES_STATE_RECOVER)
    {
	l_State = SCALES_STATE_POWER_ON;
	ScalesEnable();
	return;
    }

    /* Check for power-up problems */
    if (l_ComErrorCnt == 0  &&  l_State == SCALES_STATE_GET_ID)
    {
#ifdef LOGGING
	LogError ("Scales: Timeout during initialization"
		  " - Scales not connected?");
#endif
	l_State = SCALES_STATE_OFF;
	ScalesDisable();
	return;
    }

    /* See if power-up time of scales is over */
    if (l_State == SCALES_STATE_POWER_ON)
    {
	if (l_flgInit)
	{
	    l_flgInit = false;		// do this only once
#ifdef LOGGING
	    Log ("Scales should be ready, retrieving hard- and software"
		 " information");
#endif
	    startState = SCALES_STATE_GET_ID;
	}
	else
	{
#ifdef LOGGING
	    Log ("Scales should be ready, sending configuration values");
#endif
	    startState = SCALES_STATE_SEND_TW;
	}
	l_flgTxComplete = true;
	ScalesSendCmdSeq(startState);

	return;
    }

    /* Otherwise it is a real timeout, i.e. error */
    l_ComErrorCnt++;	// increase error count

#ifdef LOGGING
    LogError ("Scales: %d. Communication Timeout in state %d",
	      l_ComErrorCnt, l_State);
#endif

    /* Otherwise initiate recovery of the scales system */
    if (l_ComErrorCnt < MAX_COM_ERROR_CNT)
    {
	/* Immediately disable and power off the scales system */
	ScalesDisable();	// calls ScalesPowerOff(), sets SCALES_STATE_OFF

	/* Try to recover in 3 seconds */
	l_State = SCALES_STATE_RECOVER;
#ifdef LOGGING
	Log ("Try to recover Scales");
#endif
	if (l_hdlWdog != NONE)
	    sTimerStart (l_hdlWdog, 3);
    }
    else
    {
#ifdef LOGGING
	LogError ("Scales: MAX_COM_ERROR_CNT (%d) exceeded", MAX_COM_ERROR_CNT);
#endif
    }
}


/***************************************************************************//**
 *
 * @brief	Scales Command Retry
 *
 * This routine is called from the RTC interrupt handler, after the specified
 * amount of time has elapsed to initiate a "Command Retry".  This is required
 * if the "ST" command returns "ERR" (error).
 *
 ******************************************************************************/
static void ScalesCmdRetry(TIM_HDL hdl)
{
    (void) hdl;		// suppress compiler warning "unused parameter"

    /* Currently only the "ST" command uses this timer */
    ScalesSendCmdSeq(SCALES_STATE_SEND_ST);
}


/***************************************************************************//**
 *
 * @brief	Start sending a Command Sequence to Scales
 *
 * This routine starts to send the specified command of a complete sequence.
 * The next command is usually selected by CheckScalesData().
 *
 * @param[in] state
 *	Must be of type @ref SCALES_STATE.  Specifies the command to send.
 *
 ******************************************************************************/
static void ScalesSendCmdSeq(SCALES_STATE state)
{
char  buffer[40];
const char *cmd;

    cmd = buffer;
    switch (state)
    {
	case SCALES_STATE_GET_ID:	// Get Scale's ID (hardware type)
	    cmd = "ID\r";
	    break;

	case SCALES_STATE_GET_HW_VERS:	// Get Scale's Hardware Version
	    cmd = "IH\r";
	    break;

	case SCALES_STATE_GET_FW_VERS:	// Get Scale's Firmware Version
	    cmd = "IV\r";
	    break;

	case SCALES_STATE_GET_SN:	// Get Scale's Serial Number
	    cmd = "RS\r";
	    break;

	case SCALES_STATE_SEND_TW:	// Send Tare Window [g]
	    sprintf(buffer, "TW %ld\r", g_ScalesCfg_TW);
	    break;

	case SCALES_STATE_SEND_TI:	// Send Tare Interval [ms]
	    sprintf(buffer, "TI %ld\r", g_ScalesCfg_TI);
	    break;

	case SCALES_STATE_SEND_NR:	// Send Nominal silence (Ruhe) value [g]
	    sprintf(buffer, "NR %ld\r", g_ScalesCfg_NR);
	    break;

	case SCALES_STATE_SEND_NT:	// Send Nominal silence Time [ms]
	    sprintf(buffer, "NT %ld\r", g_ScalesCfg_NT);
	    break;

	case SCALES_STATE_SEND_SD:	// Send Start Delay [ms]
	    sprintf(buffer, "SD %ld\r", g_ScalesCfg_SD);
	    break;

	case SCALES_STATE_SEND_MT:	// Send Measure Time [ms]
	    sprintf(buffer, "MT %ld\r", g_ScalesCfg_MT);
	    break;

	case SCALES_STATE_SEND_TL:	// Send Trigger Level [g]
	    sprintf(buffer, "TL %ld\r", g_ScalesCfg_TL);
	    break;

	case SCALES_STATE_SEND_ST:	// Send Set Tare
	    sprintf(buffer, "ST\r");
	    break;

	case SCALES_STATE_SEND_SA:	// [S]end (Mean Value) [A]utomatically
	    cmd = "SA\r";
	    break;

	default:
#ifdef LOGGING
	    LogError("Scales ScalesSendCmdSeq(): INVALID STATE %d", state);
#endif
	    cmd =  NULL;
	    state = SCALES_STATE_OFF;
    }

    l_State = state;		// save current state into local variable

    if (cmd != NULL)
	SendCmd(cmd);
}


/***************************************************************************//**
 *
 * @brief	Send Command
 *
 * Send a command to the Scales amplifier.
 *
 * @param[in] pCmdStr
 *	Address pointer of the string to be written into the transmit buffer.
 *
 ******************************************************************************/
void SendCmd (const char *pCmdStr)
{
    /* Check if previous command has been written already */
    if (! l_flgTxComplete)
    {
#ifdef LOGGING
	LogError("Scales SendCmd(%s): Previous command still pending", pCmdStr);
#endif
    }

    /* Check length */
    unsigned int len = strlen(pCmdStr);
    if (len > sizeof(l_TxBuffer) - 2)
    {
#ifdef LOGGING
	LogError("Scales SendCmd(%s): Command too long (%d bytes)", pCmdStr, len);
#endif
	return;		// ignore this command
    }

    /* Copy command string into transmit buffer */
    strcpy(l_TxBuffer, pCmdStr);

    /* Clear flag, reset index */
    l_flgTxComplete = false;
    l_TxIdx = 0;

    /* Enable Tx interrupt to start sending */
    USART_IntSet(l_Scales_USART.UART, USART_IF_TXBL);
    USART_IntEnable(l_Scales_USART.UART, USART_IEN_TXBL);

    /* Start watchdog */
    if (l_hdlWdog != NONE)
	sTimerStart (l_hdlWdog, 3);
}


/***************************************************************************//**
 *
 * @brief	Check Scales Data
 *
 * This routine is called from Scales RX Handler to check the weight of the
 * bird and other information from scales.
 *
 ******************************************************************************/
static void CheckScalesData (void)
{
int  i;

    /* Cancel watchdog timer */
    if (l_hdlWdog != NONE)
	sTimerCancel(l_hdlWdog);

#if MOD_DEBUG	// for debugging only
    Log("Scales Data: '%s' state=%d", l_RxBuffer, l_State);
#endif

    /* Consider state */
    switch (l_State)
    {
	case SCALES_STATE_GET_ID:	// Get Scale's ID (hardware type)
	    if (strncmp(l_RxBuffer, "D:", 2) == 0)
	    {
		Log("Scales Hardware Type    : %s", l_RxBuffer+2);
		ScalesSendCmdSeq(l_State+1);
	    }
	    else
	    {
		LogError("Scales: GET_ID received \"%s\"", l_RxBuffer);
		SetError(ERR_SRC_SCALES);	// indicate error via LED
	    }
	    break;

	case SCALES_STATE_GET_HW_VERS:	// Get Scale's Hardware Version
	    if (strncmp(l_RxBuffer, "H:", 2) == 0)
	    {
		/* Remove 'FFFF...' at the end of the string */
		for (i = 2;  l_RxBuffer[i] != EOS;  i++)
		{
		    if (l_RxBuffer[i] == 'F')
		    {
			l_RxBuffer[i] = EOS;
			break;
		    }
		}

		Log("Scales Hardware Version : %s", l_RxBuffer+2);
		ScalesSendCmdSeq(l_State+1);
	    }
	    else
	    {
		LogError("Scales: GET_HW_VERS received \"%s\"", l_RxBuffer);
		SetError(ERR_SRC_SCALES);	// indicate error via LED
	    }
	    break;

	case SCALES_STATE_GET_FW_VERS:	// Get Scale's Firmware Version
	    if (strncmp(l_RxBuffer, "V:", 2) == 0)
	    {
		Log("Scales Firmware Version : %s", l_RxBuffer+2);
		ScalesSendCmdSeq(l_State+1);
	    }
	    else
	    {
		LogError("Scales: GET_FW_VERS received \"%s\"", l_RxBuffer);
		SetError(ERR_SRC_SCALES);	// indicate error via LED
	    }
	    break;

	case SCALES_STATE_GET_SN:	// Get Scale's Serial Number
	    if (strncmp(l_RxBuffer, "S+", 2) == 0)
	    {
		Log("Scales Serial Number    : %s", l_RxBuffer+2);
		ScalesSendCmdSeq(l_State+1);
	    }
	    else
	    {
		LogError("Scales: GET_SN received \"%s\"", l_RxBuffer);
		SetError(ERR_SRC_SCALES);	// indicate error via LED
	    }
	    break;

	case SCALES_STATE_SEND_TW:	// Send Tare Window [g]
	    if (strcmp(l_RxBuffer, "OK") == 0)
	    {
		ScalesSendCmdSeq(l_State+1);
	    }
	    else
	    {
		LogError("Scales: SEND_TW received \"%s\"", l_RxBuffer);
		SetError(ERR_SRC_SCALES);	// indicate error via LED
	    }
	    break;

	case SCALES_STATE_SEND_TI:	// Send Tare Interval [ms]
	    if (strcmp(l_RxBuffer, "OK") == 0)
	    {
		ScalesSendCmdSeq(l_State+1);
	    }
	    else
	    {
		LogError("Scales: SEND_TI received \"%s\"", l_RxBuffer);
		SetError(ERR_SRC_SCALES);	// indicate error via LED
	    }
	    break;

	case SCALES_STATE_SEND_NR:	// Send Nominal silence (Ruhe) value [g]
	    if (strcmp(l_RxBuffer, "OK") == 0)
	    {
		ScalesSendCmdSeq(l_State+1);
	    }
	    else
	    {
		LogError("Scales: SEND_NR received \"%s\"", l_RxBuffer);
		SetError(ERR_SRC_SCALES);	// indicate error via LED
	    }
	    break;

	case SCALES_STATE_SEND_NT:	// Send Nominal silence Time [ms]
	    if (strcmp(l_RxBuffer, "OK") == 0)
	    {
		ScalesSendCmdSeq(l_State+1);
	    }
	    else
	    {
		LogError("Scales: SEND_NT received \"%s\"", l_RxBuffer);
		SetError(ERR_SRC_SCALES);	// indicate error via LED
	    }
	    break;

	case SCALES_STATE_SEND_SD:	// Send Start Delay [ms]
	    if (strcmp(l_RxBuffer, "OK") == 0)
	    {
		ScalesSendCmdSeq(l_State+1);
	    }
	    else
	    {
		LogError("Scales: SEND_SD received \"%s\"", l_RxBuffer);
		SetError(ERR_SRC_SCALES);	// indicate error via LED
	    }
	    break;

	case SCALES_STATE_SEND_MT:	// Send Measure Time [ms]
	    if (strcmp(l_RxBuffer, "OK") == 0)
	    {
		ScalesSendCmdSeq(l_State+1);
	    }
	    else
	    {
		LogError("Scales: SEND_MT received \"%s\"", l_RxBuffer);
		SetError(ERR_SRC_SCALES);	// indicate error via LED
	    }
	    break;

	case SCALES_STATE_SEND_TL:	// Send Trigger Level [g]
	    if (strcmp(l_RxBuffer, "OK") == 0)
	    {
		ScalesSendCmdSeq(l_State+1);
	    }
	    else
	    {
		LogError("Scales: SEND_TL received \"%s\"", l_RxBuffer);
		SetError(ERR_SRC_SCALES);	// indicate error via LED
	    }
	    break;

	case SCALES_STATE_SEND_ST:	// Send Set Tare
	    if (strcmp(l_RxBuffer, "OK") == 0)
	    {
		ScalesSendCmdSeq(l_State+1);
	    }
	    else
	    {
		LogError("Scales: SEND_ST received \"%s\"", l_RxBuffer);
		SetError(ERR_SRC_SCALES);	// indicate error via LED

		if (strcmp(l_RxBuffer, "ERR") == 0)
		{
		    /* "ST" command failed - scales is not silent currently */
		    if (l_ST_RetryCnt >= MAX_ST_RETRY_CNT)
		    {
			LogError("Scales: SEND_ST - Reached maximum retry count"
				 " (%d), powering scales off", MAX_ST_RETRY_CNT);

			ScalesDisable();	// Disable Scales System
		    }
		    else
		    {
			l_ST_RetryCnt++;
			Log ("Scales: SEND_ST - %d. retry in %d seconds",
			     l_ST_RetryCnt, ST_RETRY_DELAY);

			if (l_hdlCmdRetry != NONE)
			    sTimerStart (l_hdlCmdRetry, ST_RETRY_DELAY);
		    }
		}
	    }
	    break;

	case SCALES_STATE_SEND_SA:	// [S]end (Mean Value) [A]utomatically
	    if (strcmp(l_RxBuffer, "OK") == 0)
	    {
		l_State = SCALES_STATE_OPERATIONAL;
		Log ("Scales is operational now");
		ClearError(ERR_SRC_SCALES);	// command sequence completed
	    }
	    else
	    {
		LogError("Scales: SEND_SA received \"%s\"", l_RxBuffer);
		SetError(ERR_SRC_SCALES);	// indicate error via LED
	    }
	    break;

	case SCALES_STATE_OPERATIONAL:	// Scales system is operational
	    if (l_RxBuffer[0] == 'A'
	    &&  (l_RxBuffer[1] == '+'  ||  l_RxBuffer[1] == '-'))
	    {
		// Scales first sends "A+99999.9" - ignore this value
		if (strncmp(l_RxBuffer, "A+9999", 6) != 0)
		    Log("Scales Weight: %s", l_RxBuffer+1);
	    }
	    else
	    {
		Log("Scales: Received message \"%s\"", l_RxBuffer);
	    }
	    break;

	default:			// unknown state
	    LogError("Scales: Received \"%s\" for unhandled state %d",
		     l_RxBuffer, l_State);
	    SetError(ERR_SRC_SCALES);		// indicate error via LED
    }
}


/*============================================================================*/
/*=============================== UART Routines ==============================*/
/*============================================================================*/

/* Setup UART in async mode for RS232*/
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

/**************************************************************************//**
 * @brief Scales UART Setup Routine
 *****************************************************************************/
static void ScalesUartSetup(void)
{
    /* Enable clock for USART module */
    CMU_ClockEnable(l_Scales_USART.cmuClock_UART, true);

    /* Configure GPIO Rx and Tx pins - enable pull-up for Rx */
    GPIO_PinModeSet(l_Scales_USART.UART_Rx_Port,
		    l_Scales_USART.UART_Rx_Pin, gpioModeInputPull, 1);
    GPIO_PinModeSet(l_Scales_USART.UART_Tx_Port,
		    l_Scales_USART.UART_Tx_Pin, gpioModePushPull, 1);

    /* Prepare structure for initializing UART in asynchronous mode */
    uartInit.enable       = usartDisable;   // Don't enable UART upon initialization
    uartInit.refFreq      = 0;              // Set to 0 to use reference frequency
    uartInit.baudrate     = l_Scales_USART.Baudrate;
    uartInit.oversampling = usartOVS16;     // Oversampling. Range is 4x, 6x, 8x or 16x
    uartInit.databits     = l_Scales_USART.DataBits;
    uartInit.parity       = l_Scales_USART.Parity;
    uartInit.stopbits     = l_Scales_USART.StopBits;
#if defined( USART_INPUT_RXPRS ) && defined( USART_CTRL_MVDIS )
    uartInit.mvdis        = false;          // Disable majority voting
    uartInit.prsRxEnable  = false;          // Enable USART Rx via Peripheral Reflex System
    uartInit.prsRxCh      = usartPrsRxCh0;  // Select PRS channel if enabled
#endif

    /* Initialize USART with uartInit structure */
    USART_InitAsync(l_Scales_USART.UART, &uartInit);

    /* Prepare UART Rx and Tx interrupts */
    USART_IntClear(l_Scales_USART.UART, _USART_IFC_MASK);
    USART_IntEnable(l_Scales_USART.UART, USART_IEN_RXDATAV);
    NVIC_SetPriority(l_Scales_USART.UART_Rx_IRQn, INT_PRIO_UART);
    NVIC_SetPriority(l_Scales_USART.UART_Tx_IRQn, INT_PRIO_UART);
    NVIC_ClearPendingIRQ(l_Scales_USART.UART_Rx_IRQn);
    NVIC_ClearPendingIRQ(l_Scales_USART.UART_Tx_IRQn);
    NVIC_EnableIRQ(l_Scales_USART.UART_Rx_IRQn);
    NVIC_EnableIRQ(l_Scales_USART.UART_Tx_IRQn);

    /* Enable I/O pins at UART location #2 */
    l_Scales_USART.UART->ROUTE  = USART_ROUTE_RXPEN
				| USART_ROUTE_TXPEN
				| l_Scales_USART.UART_Route;

    /* Enable UART receiver only */
    USART_Enable(l_Scales_USART.UART, usartEnable);
}


/**************************************************************************//**
 * @brief USART0 RX IRQ Handler
 *****************************************************************************/
void USART0_RX_IRQHandler(void)
{
uint8_t rxData;

    /* Check for RX data valid interrupt */
    if (l_Scales_USART.UART->IF & USART_IF_RXDATAV)
    {
	/* Get byte from RX data register */
	rxData = l_Scales_USART.UART->RXDATA;
#if MOD_DEBUG	// for debugging only
	Log("DBG: 0x%02X ('%c')", rxData,rxData < ' '? '.':rxData);
#endif
	switch (rxData)
	{
	    case 0x00:	// may be sent after power-up - just ignore
	    case '\n':	// Ignore <NL>
		break;

	    case '\r':	// Termination character
		l_RxBuffer[l_RxIdx] = EOS;		// terminate string
		l_RxIdx = 0;				// reset index
		CheckScalesData();
		break;

	    default:	// any other character
		if (l_RxIdx < sizeof(l_RxBuffer) - 2)
		{
		    l_RxBuffer[l_RxIdx++] = rxData;
		}
		else if (l_RxIdx < sizeof(l_RxBuffer) - 1)
		{
		    l_RxBuffer[l_RxIdx++] = EOS;
		    LogError("Scales: l_RxBuffer full - Data=\"%s\"",
			     l_RxBuffer);
		}
		break;
	} // switch (rxData)
    }
}


/**************************************************************************//**
 * @brief USART0 TX IRQ Handler
 *****************************************************************************/
void USART0_TX_IRQHandler(void)
{
uint8_t  txData;

    /* Check TX buffer level status */
    if (l_Scales_USART.UART->IF & USART_IF_TXBL)
    {
	/* Get the next character from the transmit buffer */
	txData = l_TxBuffer[l_TxIdx];

	if (txData != 0)
	{
	    /* Transmit next character */
	    l_Scales_USART.UART->TXDATA = (uint32_t)txData;
	    l_TxIdx++;
	}
	else
	{
	    /* Disable Tx interrupt if no more bytes in buffer */
	    USART_IntDisable(l_Scales_USART.UART, USART_IEN_TXBL);

	    /* Set flag to indicate data has been transmitted completely */
	    l_flgTxComplete = true;
	}
    }
}
