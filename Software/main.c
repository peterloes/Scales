/***************************************************************************//**
 * @file
 * @brief	MOMO
 * @author	Peter Loes
 * @author	Ralf Gerhauser
 * @version	2019-05-24
 *
 * This application consists of the following modules:
 * - main.c - Initialization code and main execution loop.
 * - DMA_ControlBlock.c - Control structures for the DMA channels.
 * - Control.c - Sequence Control module.
 * - CfgData.c - Handling of configuration data.
 * - ExtInt.c - External interrupt handler.
 * - AlarmClock.c - Alarm clock and timers facility.
 * - DCF77.c - DCF77 Atomic Clock Decoder
 * - clock.c - An implementation of the POSIX time() function.
 * - RFID.c - RFID reader to receive transponder IDs.
 * - Scales.c - Scales to measure bird's weight.
 * - BatteryMon.c - Battery monitor, periodically reads the state of the
 *   battery via the SMBus.
 * - LEUART.c - The Low-Energy UART can be used as monitoring and debugging
 *   connection to a host computer.
 * - microsd.c - Together with the files "diskio.c" and "ff.c", this module
 *   provides an implementation of a FAT file system on the @ref SD_Card.
 * - Logging.c - Logging facility to send messages to the LEUART and store
 *   them into a file on the SD-Card.
 * - PowerFail.c - Handler to switch off all loads in case of Power Fail.
 *
 * Parts of the code are based on the example code of AN0006 "tickless calender"
 * from Energy Micro AS.
 *
 ***************************************************************************//**
 *
 * Parts are Copyright 2013 Energy Micro AS, http://www.energymicro.com
 *
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ****************************************************************************//*
Revision History:
2020-05-12,rage	Call CheckAlarmTimes() after CONFIG.TXT has been read.
2019-06-20,rage	- Moved DMA related variables to module "DMA_ControlBlock.c".
		- Report available memory (debug version only).
2019-05-24,rage	- Used basic logic from project TAMDL.
2017-11-07,Loës	- Initial version for MOMO, derived from MAPRDL.
*/

/*!
 * @mainpage
 * @section desc Description
 * MOMO is used to log bird's transponder ID and weight.  It consists of the
 * following components.
 *
 * @subsection ssa00 Memory Map
 * FLASH and SRAM memory ranges:
 * <center><table>
 * <tr><th>Address</th> <th>Description</th></tr>
 * <tr><td>0x00000000</td> <td>Start of FLASH, here resides the Booter (32KB)</td></tr>
 * <tr><td>0x00008000</td> <td>Start of Application Software (up to 88KB)</td></tr>
 * <tr><td>0x0001FFFF</td> <td>End of 128KB FLASH</td></tr>
 * <tr><td>0x10000000</td> <td>Start of SRAM when accessed as Code</td></tr>
 * <tr><td>0x10003FFF</td> <td>End of 16KB SRAM when accessed as Code</td></tr>
 * <tr><td>0x20000000</td> <td>Start of SRAM when accessed as Data</td></tr>
 * <tr><td>0x20003FFF</td> <td>End of 16KB SRAM when accessed as Data</td></tr>
 * </table></center>
 *
 * @subsection ssa01 Microcontroller
 * The heart of the board is an EFM32G230 microcontroller.  It provides two
 * different clock domains: All low-energy peripheral is clocked via a
 * 32.768kHz external XTAL.  The MCU and other high performance peripheral
 * uses a high-frequency clock.  The board can be configured to use the
 * internal RC-oscillator, or an external 32MHz XTAL for that purpose,
 * see define @ref USE_EXT_32MHZ_CLOCK.
 *
 * @subsection DCF77_Atomic_Clock DCF77 Atomic Clock
 * When powered on, the DCF77 hardware module is enabled to receive the time
 * information and set the system clock.  During this phase, the Power-On LED
 * (i.e. LED1) shows the current state of the DCF signal.  The further behavior
 * depends on the settings in the <i>config.h</i> file.  If the define @ref
 * DCF77_ONCE_PER_DAY is 1, the receiver (and also the LED) will be switched
 * off after time has been synchronized.  As the name of the define suggests,
 * it will be switched on once per day.  To properly detect a change between
 * winter and summer time, i.e. <i>normal time</i> (MEZ) and <i>daylight saving
 * time</i> (MESZ), this happens at 01:55 for MEZ, and 02:55 for MESZ.<br>
 * If the define is 0, the receiver remains switched on, but the LED will only
 * get active again when the time signal gets out-of-sync.<br>
 *
 * @subsection RFID_Reader RFID Reader
 * The RFID reader is used to receive the transponder number of the bird.
 * The module can be configured as Short Range (SR) or Long Range (LR) reader
 * via configuration variable @ref RFID_TYPE.  The reader hardware is supplied
 * by one of the @ref Power_Outputs.  Use variable @ref RFID_POWER to select the
 * right one.  Be sure to verify the output voltage is correct, otherwise
 * the reader may be destroyed.<br>
 * The RFID reader is enabled by the ON-Times, and disabled by the respective
 * OFF-Times.  If a transponder ID has been detected, it is logged to a file
 * on the @ref SD_Card.  If the configuration variable @ref
 * RFID_ABSENT_DETECT_TIMEOUT is set to a value greater than 0, the absence
 * detection is enabled, and another event will be logged after the transponder
 * cannot be seen (read) anymore, for the amount of time.
 *
 * @subsection SCALES Scales
 * The Scales is used to measure the weight of the bird.  It is switched ON or
 * OFF together with the @ref RFID_Reader.  Scales uses the amplifier DAD 141.1
 * which handles automatic tare and measurement.  Amplifier and Scales are
 * powered by one of the @ref Power_Outputs, usually <b>UA2</b>.  Use variable
 * @ref SCALES_POWER to select the power output.
 * See manual <a href="../Handbuch_DAD_141.1.pdf">Handbuch DAD 141.1</a> for
 * more information.
 *
 * @subsection ssa07 LEDs
 * There are two LEDs, one is red, the other one is green.
 * - The red LED is the Power-On LED.  It shows the current state of the
 *   DCF77 signal during receiving of the time information.  In normal
 *   condition the LED will be switched off.
 * - The green LED is the Log Flush LED.  It flashes whenever there are write
 *   accesses to SD-Card.
 *
 * @subsection SD_Card SD-Card
 * The SD-Card is used to store configuration and logging data.  Only formatted
 * cards can be used, supported file systems are FAT12, FAT16, and FAT32.  The
 * filenames must follow the DOS schema 8+3, i.e. maximum 8 characters for
 * the basename, and 3 for the extension.  If the SD-Card contains a file
 * <b>BOX<i>nnnn</i>.TXT</b>, where <b><i>nnnn</i></b> can be any decimal
 * number, this will be used as the new log file.  In this way the filename
 * allows you to assign a specific box number to a dedicated unit.  If no
 * such file exists on the media, file "BOX0999.TXT" is created.
 * Another file on the SD-Card is <a href="../../CONFIG.TXT"><i>CONFIG.TXT</i></a>
 * which contains the configuration for the feeder.  Have a look at the example
 * configuration file for a description of all possible variables defined in
 * @ref l_CfgVarList.
 * - Removing an SD-Card
 *   -# Generate a log message by some action, e.g. with a transponder,
 *      to force a LogFlush().
 *   -# Wait about 5 seconds (@ref LOG_SAMPLE_TIMEOUT) until the green LED
 *      starts flashing.  Log data is now written to the SD-Card.
 *   -# When the green LED stops flashing, you have a guaranteed duration
 *      of 15 seconds (@ref LOG_FLUSH_PAUSE) where no further data will be
 *      written to the SD-Card.
 *   -# Remove the SD-Card within this time.  All newly generated log
 *      messages will be stored in memory until another media is available.
 *
 * - Inserting an SD-Card
 *   -# Just insert the SD-Card into the slot.
 *   -# The file system on the SD-Card will be mounted and the free space of
 *      the media is reported.
 *   -# If a firmware update file (*.UPD) exists on the SD-Card, a reboot will
 *      be initiated to pass control to the booter.
 *   -# Otherwise the firmware looks for a "BOX<n>.TXT" file to use as new log
 *      file.
 *
 * @subsection ssa08 Battery Monitor
 * The battery pack has its own controller.  It is connected to the EFM32
 * microcontroller via I2C-bus.  The battery monitor requests status
 * information from the battery pack and logs this on a regular basis, i.e.
 * at @ref ALARM_BAT_MON_TIME_1 and @ref ALARM_BAT_MON_TIME_2.
 *
 * @subsection Low_Energy_UART Low-Energy UART
 * The Low-Energy UART (LEUART) provides a connection to a host computer (PC).
 * It can be used as monitoring and debugging interface.  All log messages,
 * written to the SD-Card, are sent through this interface also.  This behaviour
 * can be changed by defining @ref LOG_MONITOR_FUNCTION to @ref NONE.
 * The format of this UART is 9600 baud, 8 data bits, no parity.
 *
 * @subsection Power_Outputs Power Outputs
 * There are 8 switchable power outputs:
 * - <b>UA1</b> from DC/DC converter IC3
 * - <b>UA2</b> from DC/DC converter IC4
 * - <b>SERVO</b> from BATT_INPUT via FET T12
 * - <b>LINEAR</b> from BATT_INPUT via FET T13
 * - <b>RFID1</b> from VMCU via FET T4
 * - <b>RFID2</b> from VMCU via FET T7
 * - <b>RFID3</b> from VMCU via FET T9
 * - <b>RFID_GND_LB</b> switchable ground via FET T5
 * For the MOMO application only UA1 and UA2 are used.
 *
 * All of these outputs can be switched on or off at selectable times.  Up to
 * five <b>on</b> and <b>off</b> times can be defined via configuration
 * variables in file <a href="../../CONFIG.TXT"><i>CONFIG.TXT</i></a>, see
 * @ref alarm_times.
 *
 * If the @ref RFID_Reader is configured for one of this outputs, it will
 * automatically be enabled or disabled together with the respective power
 * output.  The same appears to @ref SCALES.
 *
 * @subsection ssa11 Firmware
 * The firmware consists of an initialization part and a main loop, also called
 * service execution loop.  The initialization part sets up all modules, enables
 * devices and interrupts.  The service execution loop handles all tasks that
 * must not be executed in interrupt context.
 *
 * After power-up or reset the following actions are performed:
 * -# Basic initialization of MCU and clocks
 * -# Low-Energy UART is set up
 * -# LEDs are switched on for test purposes (lamp test)
 * -# The logging facility is initialized and the firmware version is logged
 * -# Further hardware initialization (DCF77, RFID reader, SD-Card interface,
 *    Interrupts, Alarm Clock, and Control Module)
 * -# The DCF77 Atomic Clock is enabled to receive the time signal
 * -# When valid times have been received twice, the system clock is set and
 *    alarm times are converted to the appropriate time zone (MEZ or MESZ)
 * -# Alarm times are checked against the current time, devices are enabled
 *    when time is in range
 *
 * The program then enters the Service Execution Loop which takes care of:
 * - Power management for the RFID reader
 * - SD-Card change detection, re-mounting the filesystem, and reading the
 *   configuration file <a href="../../CONFIG.TXT"><i>CONFIG.TXT</i></a>
 * - Battery monitoring
 * - Writing the log buffer to the SD-Card
 * - Entering the right energy mode
 *
 * @section cfg Configuration Variables
 * The behaviour of the system can be changed by setting configuration
 * variables in the file <a href="../../CONFIG.TXT"><i>CONFIG.TXT</i></a>
 * which resides on the SD-Card.<br>
 *
 * <b>A Note about Time Variables:</b><br>
 * The system clock always starts with <b>standard</b> time, i.e. <b>MEZ</b>.
 * When the @ref DCF77_Atomic_Clock time has been received, and this notifies
 * <b>daylight saving time</b>, i.e. <b>MESZ</b>, all alarm times will be
 * converted to MESZ by adding one hour.  This requires time variables, e.g.
 * @ref alarm_times to be set in <b>MEZ</b> time zone, even if MESZ is
 * currently valid.<br>
 *
 * @subsection alarm_times ON_TIME_1~5, OFF_TIME_1~5
 * These variables determine the ON and OFF time for the RFID reader and Scales.
 *
 * @subsection RFID_TYPE
 * Specifies the @ref RFID_Reader type.  Use <b>SR</b> for a Short Range, and
 * <b>LR</b> for a Long Range reader.
 *
 * @subsection RFID_POWER
 * Selects one of the @ref Power_Outputs to be used as power supply for the
 * @ref RFID_Reader.  If this output is switched by an ON or OFF time, the
 * reader will be enabled or disabled also.
 *
 * @subsection RFID_ABSENT_DETECT_TIMEOUT
 * If set to a value greater than 0, the absence of a transponder, i.e. when it
 * cannot be seen (read) anymore for the amount of seconds, will also be logged.
 * A value of 0 disables the absence detection.  This is useful for "fly thru"
 * designs, e.g. with the Smart Nest Box.
 *
 * @subsection SCALES_POWER
 * Selects one of the @ref Power_Outputs to be used as power supply for the
 * @ref SCALES.  If this output is switched by an ON or OFF time, Scales will
 * be enabled or disabled.
 *
 * @subsection SCALES_CFG_TW
 * Parameter <b>TW</b> specifies the <i>Tare Weight</i> in [g].  See manual
 * <a href="../Handbuch_DAD_141.1.pdf">Handbuch DAD 141.1</a>, chapter 10.5.7
 * for more information.
 *
 * @subsection SCALES_CFG_TI
 * Parameter <b>TI</b> specifies the <i>Tare Interval</i> in [ms].  See manual
 * <a href="../Handbuch_DAD_141.1.pdf">Handbuch DAD 141.1</a>, chapter 10.5.8
 * for more information.
 *
 * @subsection SCALES_CFG_NR
 * Parameter <b>NR</b> specifies the <i>Nominal silence (Ruhe)</i> value in [g].
 * See manual <a href="../Handbuch_DAD_141.1.pdf">Handbuch DAD 141.1</a>,
 * chapter 10.3.1 for more information.
 *
 * @subsection SCALES_CFG_NT
 * Parameter <b>NT</b> specifies the <i>Nominal silence Time</i> in [ms].
 * See manual <a href="../Handbuch_DAD_141.1.pdf">Handbuch DAD 141.1</a>,
 * chapter 10.3.2 for more information.
 *
 * @subsection SCALES_CFG_SD
 * Parameter <b>SD</b> specifies the <i>Start Delay</i> for measurement in [ms].
 * See manual <a href="../Handbuch_DAD_141.1.pdf">Handbuch DAD 141.1</a>,
 * chapter 10.13.1 for more information.
 *
 * @subsection SCALES_CFG_MT
 * Parameter <b>MT</b> specifies the <i>Measure Time</i> in [ms].  A mean value
 * is calculated from the cyclic measurements over this duration.
 * See manual <a href="../Handbuch_DAD_141.1.pdf">Handbuch DAD 141.1</a>,
 * chapter 10.13.2 for more information.
 *
 * @subsection SCALES_CFG_TL
 * Parameter <b>TL</b> specifies the <i>Trigger Level</i> in [g].  This is the
 * minimum weight which is required to trigger a measurement.  See manual
 * <a href="../Handbuch_DAD_141.1.pdf">Handbuch DAD 141.1</a>, chapter 10.13.6
 * for more information.
 */
/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_dma.h"
#include "config.h"		// include project configuration parameters
#include "ExtInt.h"
#include "DCF77.h"
#include "RFID.h"
#include "AlarmClock.h"
#include "LEUART.h"
#include "BatteryMon.h"
#include "Logging.h"
#include "CfgData.h"
#include "Control.h"
#include "PowerFail.h"
#include "Scales.h"

#ifdef DEBUG
#include <malloc.h>
#endif

#include "ff.h"		// FS_FAT12/16/32
#include "diskio.h"	// DSTATUS
#include "microsd.h"

/*================================ Global Data ===============================*/

extern PRJ_INFO const  prj;		// Project Information

/*! @brief Flag to indicate that an Interrupt occurred in the meantime.
 *
 * This flag must be set <b>true</b> by any interrupt service routine that
 * requires actions in the service execution loop of main().  This prevents
 * the system from entering sleep mode, so the action can be taken before.
 */
volatile bool		g_flgIRQ;

/*! @brief Modules that require EM1.
 *
 * This global variable is a bit mask for all modules that require EM1.
 * Standard peripherals would stop working in EM2 because clocks, etc. are
 * disabled.  Therefore it is required for software modules that make use
 * of such devices, to set the appropriate bit in this mask, as long as they
 * need EM1.  This prevents the power management of this application to enter
 * EM2.  The enumeration @ref EM1_MODULES lists those modules.
 * Low-Power peripherals, e.g. the LEUART still work in EM1.
 *
 * Examples:
 *
   @code
   // Module RFID requires EM1, set bit in bit mask
   Bit(g_EM1_ModuleMask, EM1_MOD_RFID) = 1;
   ...
   // Module RFID is no longer active, clear bit in bit mask
   Bit(g_EM1_ModuleMask, EM1_MOD_RFID) = 0;
   @endcode
 */
volatile uint16_t	g_EM1_ModuleMask;


/*! @brief Error Flags Variable
 *
 * This variable holds the current error state of the system, each bit
 * represents a specific error, see @ref ERR_SRC definition and functions
 * SetError(), ClearError().
 */
volatile uint16_t	l_ErrorFlags;

/*================================ Local Data ================================*/

/*! EXTI initialization structure
 *
 * Connect the external interrupts of the push buttons to the key handler, the
 * DCF77 signal to the atomic clock module, the outer and inner light barrier
 * to their handler.
 */
static const EXTI_INIT  l_ExtIntCfg[] =
{   //	IntBitMask,	IntFct
    {	DCF_EXTI_MASK,	DCF77Handler		},	// DCF77
    {	PF_EXTI_MASK,	PowerFailHandler	},	// Power Fail
    {	0,		NULL			}
};

/*!@brief Array of functions to be called in case of power-fail.
 *
 * Initialization array to define the power-fail handlers required for some
 * modules.
 * This array must be 0-terminated.
 */
static const POWER_FAIL_FCT l_PowerFailFct[] =
{
    RFID_PowerFailHandler,	// switch off RFID reader
    ScalesPowerFailHandler,	// switch off Scales
    ControlPowerFailHandler,	// switch off power outputs
    NULL
};

/* Return code for CMU_Select_TypeDef as string */
static const char *CMU_Select_String[] =
{ "Error", "Disabled", "LFXO", "LFRCO", "HFXO", "HFRCO", "LEDIV2", "AUXHFRCO" };

/*=========================== Forward Declarations ===========================*/

static void cmuSetup(void);
static void Reboot(void);
#if ENABLE_LEUART_RECEIVER
static void CheckCommand(void);
#endif
#ifdef DEBUG
static void MemInfo (void);
#endif

/***************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main( void )
{
    /* Initialize chip - handle erratas */
    CHIP_Init();

    /* EFM32 NVIC implementation provides 8 interrupt levels (0~7) */
    NVIC_SetPriorityGrouping (4);	// 8 priority levels, NO sub-priority

    /* Set up clocks */
    cmuSetup();

    /* Init Low Energy UART with 9600bd (this is the maximum) */
    drvLEUART_Init (9600);

#ifdef DEBUG
    dbgInit();
#endif

    /* Output version string to SWO or LEUART */
    drvLEUART_puts("\n***** MOMO V");
    drvLEUART_puts(prj.Version);
    drvLEUART_puts(" *****\n\n");

    /* Configure PA2 to drive the red Power-On LED (LED1) - show we are alive */
    GPIO_PinModeSet (POWER_LED_PORT, POWER_LED_PIN, gpioModePushPull, 1);

    /* Configure PA5 to drive the green Ready LED - show we are alive */
    GPIO_PinModeSet (LOG_FLUSH_LED_PORT, LOG_FLUSH_LED_PIN, gpioModePushPull, 1);

    /*
     * All modules that make use of external interrupts (EXTI) should be
     * initialized before calling ExtIntInit() because this enables the
     * interrupts, so IRQ handler may be executed immediately!
     */

    /* Initialize Logging (do this early) */
    LogInit();

    /* Log Firmware Revision and Clock Info */
    Log ("MOMO V%s (%s %s)", prj.Version, prj.Date, prj.Time);
    uint32_t freq = CMU_ClockFreqGet(cmuClock_HF);
    Log ("Using %s Clock at %ld.%03ldMHz",
	 CMU_Select_String[CMU_ClockSelectGet(cmuClock_HF)],
	 freq / 1000000L, (freq % 1000000L) / 1000L);

#ifdef DEBUG
    MemInfo();		// report available memory
#endif

     /* Initialize DCF77 hardware, configure Interrupt */
    DCF77Init();

    /* Initialize SD-Card Interface */
    DiskInit();

    /* Introduce Power-Fail Handlers, configure Interrupt */
    PowerFailInit (l_PowerFailFct);

    /* Initialize External Interrupts */
    ExtIntInit (l_ExtIntCfg);

    /* Initialize the Alarm Clock module */
    AlarmClockInit();

    /* Initialize control module */
    ControlInit();

    /* Switch Log Flush LED OFF */
    LOG_FLUSH_LED = 0;

    /* Initialize Battery Monitor */
    BatteryMonInit();

    /* Enable the DCF77 Atomic Clock Decoder */
    DCF77Enable();

    /* Enable all other External Interrupts */
    ExtIntEnableAll();

    /* ============================================ *
     * ========== Service Execution Loop ========== *
     * ============================================ */
    while (1)
    {
	/* Check for power-fail */
	if (! PowerFailCheck())
	{
#if ENABLE_LEUART_RECEIVER
	    /* Check for command from Debug Console */
	    CheckCommand();
#endif
	    /* Check if to power-on or off the RFID reader */
	    RFID_Check();

	    /* Check if to power-on or off Scales */
	    ScalesCheck();

	    /* Check if SD-Card has been inserted or removed */
	    if (DiskCheck())
	    {
		/* First check if an "*.UPD" file exists on this SD-Card */
		if (FindFile ("/", "*.UPD") != NULL)
		{
		    /*
		     * In this case the SD-Card contains update images.  We must
		     * pass control to the booter to perform a firmware upgrade.
		     */
		    Reboot();
		}

		/* New File System mounted - (re-)open Log File */
		LogFileOpen("BOX*.TXT", "BOX0999.TXT");

		/* Be sure to flush current log buffer so it is empty */
		LogFlush(true);	// keep SD-Card power on!

		/* Log information about the MCU and the battery */
		uint32_t uniquHi = DEVINFO->UNIQUEH;
		Log ("MCU: %s HW-ID: 0x%08lX%08lX",
		     PART_NUMBER, uniquHi, DEVINFO->UNIQUEL);
		LogBatteryInfo (BAT_LOG_INFO_VERBOSE);

		/* Clear (previous) Configuration - switch devices off */
		ClearConfiguration();

		/* Read and parse configuration file */
		CfgRead("CONFIG.TXT");

		/* Initialize RFID reader according to (new) configuration */
		RFID_Init();

		/* Initialize Scales according to (new) configuration */
		ScalesInit();

		/* Flush log buffer again and switch SD-Card power off */
		LogFlush(false);

		/* See if devices must be switched on at this time */
		CheckAlarmTimes();
	    }

	    /* Check Battery State */
	    BatteryCheck();

	    /* Check if to flush the log buffer */
	    LogFlushCheck();
	}

	/*
	 * Check for current power mode:  If a minimum of one active module
	 * requires EM1, i.e. <g_EM1_ModuleMask> is not 0, this will be
	 * entered.  If no one requires EM1 activity, EM2 is entered.
	 */
	if (! g_flgIRQ)		// enter EM only if no IRQ occurred
	{
	    if (g_EM1_ModuleMask)
		EMU_EnterEM1();		// EM1 - Sleep Mode
	    else
		EMU_EnterEM2(true);	// EM2 - Deep Sleep Mode
	}
	else
	{
	    g_flgIRQ = false;	// clear flag to enter EM the next time
	}
    }
}


/***************************************************************************//**
 * @brief   Configure Clocks
 *
 * This local routine is called once from main() to configure all required
 * clocks of the EFM32 device.
 *
 *****************************************************************************/
static void cmuSetup(void)
{
    /* Start LFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

#if USE_EXT_32MHZ_CLOCK
    /* Start HFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

    /* Select HFXO as clock source for HFCLK */
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

    /* Disable HFRCO */
    CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
#endif

    /* Route the LFXO clock to the RTC and set the prescaler */
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);	// RTC, LETIMER
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);	// LEUART0/1
    CMU_ClockEnable(cmuClock_RTC, true);

    /* Prescaler of 1 = 30 us of resolution and overflow each 8 min */
    CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);

    /* Enable clock to low energy modules */
    CMU_ClockEnable(cmuClock_CORELE, true);

    /* Enable clock for HF peripherals (ADC, DAC, I2C, TIMER, and USART) */
    CMU_ClockEnable(cmuClock_HFPER, true);

    /* Enable clock to GPIO */
    CMU_ClockEnable(cmuClock_GPIO, true);
}


/***************************************************************************//**
 * @brief   Reboot the System
 *
 * This local routine brings the system into a quiescent state and then
 * generates a reset.  It is typically used to transfer control from the
 * application to the booter for firmware upgrades.
 *
 *****************************************************************************/
static void Reboot(void)
{
int	n, i;

     /* Disable external interrupts */
    ExtIntDisableAll();

    /* Shut down peripheral devices */
    BatteryMonDeinit();
    RFID_PowerOff();
    ScalesPowerOff();
    DCF77Disable();

    drvLEUART_puts ("Shutting down system for reboot\n");

    /*
     * Show LED Pattern before resetting:
     * 3x 5-short-pulses, separated by a pause,
     * finally a dimming LED from maximum brightness to off.
     */
    for (n = 0;  n < 3;  n++)		// 3x patterns
    {
	for (i = 0;  i < 5;  i++)	// a' 5 pulses
	{
	    POWER_LED = 1;
	    msDelay(100);
	    POWER_LED = 0;
	    msDelay(100);
	}
	msDelay(800);			// pause
    }

    for (n = 0;  n < 200;  n++)
    {
	POWER_LED = 1;
	for (i = 0;  i < (200 - n);  i++)
	    DelayTick();

	POWER_LED = 0;
	for (i = 0;  i < n;  i++)
	    DelayTick();
    }

    /* Perform RESET */
    NVIC_SystemReset();
}


/***************************************************************************//**
 * @brief   Set Error Condition
 *
 * @param[in] errorSource
 *	Enum of type @ref ERR_SRC to identify the error source.
 *
 * This routine sets an error condition for the specified source.  The error
 * conditions of the system are stored as single bits in the local variable
 * @ref l_ErrorFlags.  If one or more errors are active, the red LED is on.
 *
 * @see
 * ClearError()
 *
 *****************************************************************************/
void	SetError (ERR_SRC errorSource)
{
    Bit(l_ErrorFlags, errorSource) = 1;
    POWER_LED = 1;
}


/***************************************************************************//**
 * @brief   Clear Error Condition
 *
 * This routine clears an error condition for the specified source.  The error
 * conditions of the system are stored as single bits in the local variable
 * @ref l_ErrorFlags.  If one or more errors are active, the red LED is on,
 * if all errors have been cleared, the LED will be switched off.  It can then
 * be used as DCF77 indicator, see ShowDCF77Indicator().
 *
 * @see
 * SetError()
 *
 *****************************************************************************/
void	ClearError (ERR_SRC errorSource)
{
    Bit(l_ErrorFlags, errorSource) = 0;
    if (l_ErrorFlags == 0)
	POWER_LED = 0;
}


/***************************************************************************//**
 * @brief   Show DCF77 Signal Indicator
 *
 * This routine is called by the DCF77 module during the synchronisation of
 * the clock to indicate the current state of the DCF77 signal.
 * On the MAPRDL board it sets the red power LED to the current state of
 * the DCF77 signal, LED on means high, LED off low level.
 *
 * @note
 * If an error condition is active, the LED will be on permanently.
 *
 *****************************************************************************/
void	ShowDCF77Indicator (bool enable)
{
    if (l_ErrorFlags)
	return;		// Errors are set - do not change the red LED

    POWER_LED = enable;
}


/***************************************************************************//**
 * @brief   Check for Command from Debug Console
 *
 * This debugging routine is called from the main loop to check if a command
 * has been entered via the debug console.  The receiver part of the LEUART
 * must be enabled to make this work.
 *
 *****************************************************************************/
#if ENABLE_LEUART_RECEIVER
static void CheckCommand(void)
{
    if (! g_flgCmdLine)
	return;

    g_flgCmdLine = false;

    if (g_CmdLine[0] != EOS)	// skip empty command lines, i.e. <CR> only
    {
	drvLEUART_puts(g_CmdLine);
	drvLEUART_puts("\n");
	if (strcmp("E", g_CmdLine) == 0)
	    ScalesEnable();
	else if (strcmp("D", g_CmdLine) == 0)
	    ScalesDisable();
	else
	{
	    strcat(g_CmdLine, "\r");	// <CR> (NOT <LF>!)
	    SendCmd(g_CmdLine);
	}
    }
}
#endif

#ifdef DEBUG
// Debug routine to show remaining memory pool size
static void MemInfo (void)
{
    /* Report available memory */
    size_t size;
    for (size = 128;  size < 32 * 1024;  size += 128)
    {
	void *ptr = malloc (size);	// allocate memory
	if (ptr == NULL)
	    break;

	free (ptr);			// free memory
    }
    Log ("Memory Info: %ld bytes available", size - 128);
}
#endif
