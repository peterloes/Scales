/***************************************************************************//**
 * @file
 * @brief	Sequence Control
 * @author	Ralf Gerhauser
 * @version	2018-03-26
 *
 * This is the automatic sequence control module.  It controls the power
 * outputs that may be activated via alarm times @ref alarm_times.
 * This module also defines the configuration variables for the file
 * <a href="../../CONFIG.TXT"><i>CONFIG.TXT</i></a>.
 *
 ****************************************************************************//*
Revision History:
2018-03-26,rage	Initial version, based on MAPRDL.
*/

/*=============================== Header Files ===============================*/

#include "em_cmu.h"
#include "em_int.h"
#include "em_gpio.h"
#include "ExtInt.h"
#include "Logging.h"
#include "AlarmClock.h"
#include "RFID.h"
#include "Scales.h"
#include "CfgData.h"
#include "Control.h"


/*=============================== Definitions ================================*/

    /*!@brief Structure to define power outputs. */
typedef struct
{
    __IO uint32_t *BitBandAddr;	// Bit band address of GPIO power enable pin
    bool	   HighActive;	// true: The power enable pin is high-active
} PWR_OUT_DEF;

    /*!@brief Macro to calculate a GPIO bit address for a port and pin. */
#define GPIO_BIT_ADDR(port, pin)					\
	IO_BIT_ADDR((&(GPIO->P[(port)].DOUT)), (pin))

/*!@brief Macro to extract the port number from a GPIO bit address.  */
#define GPIO_BIT_ADDR_TO_PORT(bitAddr)		(GPIO_Port_TypeDef)	\
	(((((uint32_t)(bitAddr) - BITBAND_PER_BASE) >> 5)		\
	 + PER_MEM_BASE - GPIO_BASE) / sizeof(GPIO_P_TypeDef))

    /*!@brief Macro to extract the pin number from a GPIO bit address.  */
#define GPIO_BIT_ADDR_TO_PIN(bitAddr)					\
	(((uint32_t)(bitAddr) >> 2) & 0x1F)

/*================================ Global Data ===============================*/

    /*!@brief CFG_VAR_TYPE_ENUM_2: Enum names for Power Outputs. */
const char *g_enum_PowerOutput[] = { "UA1", "UA2", ",SERVO", "LINEAR",
				     "RFID1", "RFID2", "RFID3", "RFID_GND_LB",
				      NULL };

/*================================ Local Data ================================*/

    /*!@brief Power output port and pin assignment - keep in sync with enums
     * @ref PWR_OUT and string array @ref g_enum_PowerOutput !
     */
static const PWR_OUT_DEF  l_PwrOutDef[NUM_PWR_OUT] =
{   //   BitBandAddr,              HighActive
    { GPIO_BIT_ADDR(gpioPortA,  3), true },	// PWR_OUT_UA1
    { GPIO_BIT_ADDR(gpioPortA,  4), true },	// PWR_OUT_UA2
    { GPIO_BIT_ADDR(gpioPortE,  8), true },	// PWR_OUT_VDD_SERVO
    { GPIO_BIT_ADDR(gpioPortC, 15), true },	// PWR_OUT_VDD_LINEAR
    { GPIO_BIT_ADDR(gpioPortC,  8), true },	// PWR_OUT_VDD_RFID1
    { GPIO_BIT_ADDR(gpioPortC,  9), true },	// PWR_OUT_VDD_RFID2
    { GPIO_BIT_ADDR(gpioPortC, 10), true },	// PWR_OUT_VDD_RFID3
    { GPIO_BIT_ADDR(gpioPortA,  6), true },	// PWR_OUT_RFID_GND_LB
};

    /*!@brief List of configuration variables.
     * Alarm times, i.e. @ref CFG_VAR_TYPE_TIME must be defined first, because
     * the array index is used to specify the alarm number \<alarmNum\>,
     * starting with @ref alarm_times, when calling AlarmSet().
     */
static const CFG_VAR_DEF l_CfgVarList[] =
{
    { "ON_TIME_1",		CFG_VAR_TYPE_TIME,	NULL		},
    { "ON_TIME_2",		CFG_VAR_TYPE_TIME,	NULL		},
    { "ON_TIME_3",		CFG_VAR_TYPE_TIME,	NULL		},
    { "ON_TIME_4",		CFG_VAR_TYPE_TIME,	NULL		},
    { "ON_TIME_5",		CFG_VAR_TYPE_TIME,	NULL		},
    { "OFF_TIME_1",		CFG_VAR_TYPE_TIME,	NULL		},
    { "OFF_TIME_2",		CFG_VAR_TYPE_TIME,	NULL		},
    { "OFF_TIME_3",		CFG_VAR_TYPE_TIME,	NULL		},
    { "OFF_TIME_4",		CFG_VAR_TYPE_TIME,	NULL		},
    { "OFF_TIME_5",		CFG_VAR_TYPE_TIME,	NULL		},
    { "RFID_TYPE",		CFG_VAR_TYPE_ENUM_1,	&g_RFID_Type	},
    { "RFID_POWER",		CFG_VAR_TYPE_ENUM_2,	&g_RFID_Power	},
    { "RFID_ABSENT_DETECT_TIMEOUT",CFG_VAR_TYPE_INTEGER,
					&g_RFID_AbsentDetectTimeout	},
    { "SCALES_POWER",		CFG_VAR_TYPE_ENUM_2,	&g_ScalesPower	},
    { "SCALES_CFG_TW",		CFG_VAR_TYPE_INTEGER,	&g_ScalesCfg_TW	},
    { "SCALES_CFG_TI",		CFG_VAR_TYPE_INTEGER,	&g_ScalesCfg_TI	},
    { "SCALES_CFG_NR",		CFG_VAR_TYPE_INTEGER,	&g_ScalesCfg_NR	},
    { "SCALES_CFG_NT",		CFG_VAR_TYPE_INTEGER,	&g_ScalesCfg_NT	},
    { "SCALES_CFG_TL",		CFG_VAR_TYPE_INTEGER,	&g_ScalesCfg_TL	},
    { "SCALES_CFG_SD",		CFG_VAR_TYPE_INTEGER,	&g_ScalesCfg_SD	},
    { "SCALES_CFG_MT",		CFG_VAR_TYPE_INTEGER,	&g_ScalesCfg_MT	},
    {  NULL,			END_CFG_VAR_TYPE,	NULL		}
};

    /*!@brief List of all enum definitions. */
static const ENUM_DEF l_EnumList[] =
{
    g_enum_RFID_Type,		// CFG_VAR_TYPE_ENUM_1
    g_enum_PowerOutput,		// CFG_VAR_TYPE_ENUM_2
};


/*=========================== Forward Declarations ===========================*/

static void	PowerControl (int alarmNum);


/***************************************************************************//**
 *
 * @brief	Initialize control module
 *
 * This routine initializes the sequence control module.
 *
 ******************************************************************************/
void	ControlInit (void)
{
int	i;

    /* Introduce variable list to configuration data module */
    CfgDataInit (l_CfgVarList, l_EnumList);

    /* Initialize power output enable pins */
    for (i = 0;  i < NUM_PWR_OUT;  i++)
    {
	/* Configure Power Enable Pin, switch it OFF per default */
	GPIO_PinModeSet (GPIO_BIT_ADDR_TO_PORT(l_PwrOutDef[i].BitBandAddr),
			 GPIO_BIT_ADDR_TO_PIN (l_PwrOutDef[i].BitBandAddr),
			 gpioModePushPull, l_PwrOutDef[i].HighActive ? 0:1);
    }

    /* Use same routine for all power-related alarms */
    for (i = FIRST_POWER_ALARM;  i <= LAST_POWER_ALARM;  i++)
	AlarmAction (i, PowerControl);

    /* Initialize configuration with default values */
    ClearConfiguration();
}


/***************************************************************************//**
 *
 * @brief	Clear Configuration
 *
 * This routine disables all alarm times and switches the corresponding power
 * outputs off.  It then sets all configuration variables to default values.
 * It must be executed <b>before</b> calling CfgRead() to to ensure the correct
 * settings for variables which are <b>not</b> set within a new configuration.
 *
 ******************************************************************************/
void	ClearConfiguration (void)
{
int	i;

    /* Disable all power-related alarms */
    for (i = FIRST_POWER_ALARM;  i <= LAST_POWER_ALARM;  i++)
    {
	if (AlarmIsEnabled(i))
	{
	    if (i >= ALARM_OFF_TIME_1)
		ExecuteAlarmAction(i);	// OFF-Time: Switch device off

	    AlarmDisable(i);		// Disable this alarm
	}
    }

    /* Disable RFID functionality */
    g_RFID_Type = RFID_TYPE_NONE;
    g_RFID_Power = PWR_OUT_NONE;
    g_RFID_AbsentDetectTimeout = DFLT_RFID_ABSENT_DETECT_TIMEOUT;

    /* Disable Scales functionality */
    g_ScalesPower = PWR_OUT_NONE;
    g_ScalesCfg_TW = 0;
    g_ScalesCfg_TI = 0;
    g_ScalesCfg_NR = 0;
    g_ScalesCfg_NT = 0;
    g_ScalesCfg_TL = 0;
    g_ScalesCfg_SD = 0;
    g_ScalesCfg_MT = 0;
}


/***************************************************************************//**
 *
 * @brief	Inform the control module about a new transponder ID
 *
 * This routine must be called to inform the control module about a new
 * transponder ID.
 *
 ******************************************************************************/
void	ControlUpdateID (char *transponderID)
{
char	 line[120];
char	*pStr;
#if 0	// Currently there are no actions planned for specific birds
ID_PARM	*pID;
#endif

    pStr = line;

#if 0	// Currently there are no actions planned for specific birds
    pID = CfgLookupID (transponderID);
    if (pID == NULL)
    {
	/* Specified ID not found, look for an "ANY" entry */
	pID = CfgLookupID ("ANY");
	if (pID == NULL)
	{
	    /* No "ANY" entry defined, treat ID as "UNKNOWN" */
	    pID = CfgLookupID ("UNKNOWN");
	    if (pID == NULL)
	    {
		/* Even no "UNKNOWN" entry exists - abort */
		Log ("Transponder: %s not found - aborting", transponderID);
		return;
	    }
	    else
	    {
		pStr += sprintf (pStr, "Transponder: %s not found -"
				 " using UNKNOWN", transponderID);
	    }
	}
	else
	{
	    pStr += sprintf (pStr, "Transponder: %s not found -"
			     " using ANY", transponderID);
	}
    }
    else
#endif
    {
	pStr += sprintf (pStr, "Transponder: %s", transponderID);
    }

#ifdef LOGGING
    Log (line);
#endif
}


/***************************************************************************//**
 *
 * @brief	Power-Fail Handler for Control Module
 *
 * This function will be called in case of power-fail to switch off devices
 * that consume too much power, e.g. the camera.
 *
 ******************************************************************************/
void	ControlPowerFailHandler (void)
{
int	i;

#ifdef LOGGING
    /* Generate Log Message */
    Log ("Switching all power outputs OFF");
#endif

    /* Switch off all power outputs immediately */
    for (i = 0;  i < NUM_PWR_OUT;  i++)
	PowerOutput ((PWR_OUT)i, PWR_OFF);
}


/******************************************************************************
 *
 * @brief	Switch the specified power output on or off
 *
 * This routine enables or disables the specified power output.
 *
 * @param[in] output
 *	Power output to be changed.
 *
 * @param[in] enable
 *	If true (PWR_ON), the power output will be enabled, false (PWR_OFF)
 *	disables it.
 *
 *****************************************************************************/
void	PowerOutput (PWR_OUT output, bool enable)
{
    /* Parameter check */
    if (output == PWR_OUT_NONE)
	return;		// power output not assigned, nothing to be done

    if ((PWR_OUT)0 > output  ||  output >= NUM_PWR_OUT)
    {
#ifdef LOGGING
	/* Generate Error Log Message */
	LogError ("PowerOutput(%d, %d): Invalid output parameter",
		  output, enable);
#endif
	return;
    }

    /* See if Power Output is already in the right state */
    if ((bool)*l_PwrOutDef[output].BitBandAddr == enable)
	return;		// Yes - nothing to be done

    /* Switch power output on or off */
    *l_PwrOutDef[output].BitBandAddr = enable;

#ifdef LOGGING
    Log ("Power Output %s %sabled",
	 g_enum_PowerOutput[output], enable ? "en":"dis");
#endif
}


/******************************************************************************
 *
 * @brief	Determine if the specified power output is switched on
 *
 * This routine determines the current state of a power output.
 *
 * @param[in] output
 *	Power output to be checked.
 *
 *****************************************************************************/
bool	IsPowerOutputOn (PWR_OUT output)
{
    /* Parameter check */
    if (output == PWR_OUT_NONE)
	return false;	// power output not assigned, return false (off)

    EFM_ASSERT ((PWR_OUT)0 <= output  &&  output < NUM_PWR_OUT);

    /* Determine the current state of this power output */
    return (*l_PwrOutDef[output].BitBandAddr ? true : false);
}


/***************************************************************************//**
 *
 * @brief	Alarm routine for Power Control
 *
 * This routine is called when one of the power alarm times has been reached.
 * The alarm number is an enum value between @ref ALARM_ON_TIME_1 and
 * @ref ALARM_OFF_TIME_5.<br>
 * When an RFID reader has been installed, the function decides whether to
 * call RFID_Enable(), or RFID_Disable().  If Scales has been configured,
 * this will also be switched on or off together the RFID reader.
 *
 ******************************************************************************/
static void	PowerControl (int alarmNum)
{
int	 pwrState;

    /* Parameter check */
    EFM_ASSERT (FIRST_POWER_ALARM <= alarmNum
		&& alarmNum <= LAST_POWER_ALARM);

    /* Determine switching state */
    pwrState = (alarmNum >= ALARM_OFF_TIME_1 ? PWR_OFF:PWR_ON);

    /* RFID reader and Scales are always switched on or off together */
    if (pwrState == PWR_ON)
    {
	RFID_Enable();
	ScalesEnable();
    }
    else
    {
	RFID_Disable();
	ScalesDisable();
    }

    g_flgIRQ = true;	// keep on running
}
