/***************************************************************************//**
 * @file
 * @brief	Header file of module BatteryMon.c
 * @author	Ralf Gerhauser
 * @version	2018-03-25
 ****************************************************************************//*
Revision History:
2018-03-25,rage	Added prototypes for BatteryInfoReq() and BatteryInfoGet().
		New SBS_CMD enum SBS_NONE to mark "no request".
		Defined structure BAT_INFO which is used by BatteryInfoGet().
2016-04-05,rage	Made global variables of type "volatile".
2016-02-26,rage	Set BAT_MON_INTERVAL 0 to disable monitor interval, defined
		ALARM_BAT_MON_TIME_x to read battery status two times per day.
		Added BAT_LOG_INFO_LVL for function LogBatteryInfo().
		Added prototype for BatteryMonDeinit().
2015-07-06,rage	Corrected SBS_ManufacturerData enum value.
2015-03-29,rage	Completed documentation.
2014-12-20,rage	Initial version.
*/

#ifndef __INC_BatteryMon_h
#define __INC_BatteryMon_h

/*=============================== Header Files ===============================*/

#include "em_device.h"
#include "em_gpio.h"
#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

/*!@brief Interval in seconds for battery monitoring (use 0 to disable). */
#ifndef BAT_MON_INTERVAL
    #define BAT_MON_INTERVAL	0
#endif

/*!@brief Time 1 (11:00) when battery status should be logged.  Do not use
 * 12:00, because this is the default, until DCF77 adjusts the real time.  So
 * battery status would be logged twice at power-up.
 */
#define ALARM_BAT_MON_TIME_1   11, 00
/*!@brief Time 2 (23:00) when battery status should be logged */
#define ALARM_BAT_MON_TIME_2   23, 00


/*!@brief Enumeration of Battery Logging Information Level */
typedef enum
{
    BAT_LOG_INFO_DISPLAY_ONLY,	//!< Get information for the display only
    BAT_LOG_INFO_SHORT,		//!< Log short information of battery status
    BAT_LOG_INFO_VERBOSE,	//!< Log verbose information of battery status
    END_BAT_LOG_INFO_LVL
} BAT_LOG_INFO_LVL;


/*!@brief SBS Commands
 *
 * Here follow the register addresses of the battery controller as used for the
 * SBS commands sent via I2C.  Bit 7:0 of the enum contain the address, while
 * bit 15:8 represent the number of bytes to read for block commands.  A zero
 * means the register is a 16bit word and must be read via function
 * BatteryRegReadWord().  All commands beginn with device address 0x0A to
 * access the battery controller.
 *
 * @see
 * Functions BatteryRegReadWord() and BatteryRegReadBlock() use these enums.
 * A list of all registers can be found in document
 * <a href="../SBS_Commands.pdf">SBS Commands</a>.
 *
 */
typedef enum
{
    SBS_NONE = (-1),			//!< No Request, see @ref BAT_INFO
    SBS_ManufacturerAccess = 0x00,	//!< 0x00 Word:
    SBS_RemainingCapacityAlarm,		//!< 0x01 Word: in [mAh]
    SBS_RemainingTimeAlarm,		//!< 0x02 Word: in [min]
    SBS_BatteryMode,			//!< 0x03 Word: @see SBS_03_BITS
    SBS_CoreTemperature	 = 0x08,	//!< 0x08 Word: Chip Temperature in [°K]
    SBS_Voltage,			//!< 0x09 Word: in [mV]
    SBS_BatteryCurrent,			//!< 0x0A Word: Actual Current in [mA]
    SBS_BatteryAverageCurrent,		//!< 0x0B Word: in [mA]
    SBS_RelativeStateOfCharge = 0x0D,	//!< 0x0D Word: in [%]
    SBS_AbsoluteStateOfCharge,		//!< 0x0E Word: in [%]
    SBS_RemainingCapacity,		//!< 0x0F Word: in [mAh]
    SBS_FullChargeCapacity,		//!< 0x10 Word: in [mAh]
    SBS_RunTimeToEmpty,			//!< 0x11 Word: in [min]
    SBS_AverageTimeToEmpty,		//!< 0x12 Word: in [min]
    SBS_AverageTimeToFull,		//!< 0x13 Word: in [min]
    SBS_ChargingCurrent,		//!< 0x14 Word: in [mA]
    SBS_ChargingVoltage,		//!< 0x15 Word: in [mV]
    SBS_BatteryStatus,			//!< 0x16 Word: @see SBS_16_BITS
    SBS_ChargerCycleCount,		//!< 0x17 Word: Number of Cycles
    SBS_DesignCapacity,			//!< 0x18 Word: in [mAh]
    SBS_DesignVoltage,			//!< 0x19 Word: in [mV]
    SBS_SpecificationInfo,		//!< 0x1A Word: Version and Revision
    SBS_ManufactureDate,		//!< 0x1B Word: [????]
    SBS_SerialNumber,			//!< 0x1C Word: Binary coded S/N
    SBS_ManufacturerName = 0x1020,	//!< 0x20 Block: (16 character string)
    SBS_DeviceName	 = 0x1021,	//!< 0x21 Block: (16 character string)
    SBS_DeviceChemistry	 = 0x1022,	//!< 0x22 Block: (16 character string)
    SBS_ManufacturerData = 0x1023,	//!< 0x23 Block: (16 character string)
    SBS_ShuntResistance	 = 0x2A,	//!< 0x2A Word: in [µOhm]
    SBS_CellsInSeries	 = 0x3C,	//!< 0x3C Word: Number of Cells
    SBS_OverCurrentReactionTime,	//!< 0x3D Word:	Level 1 Time in [??]
    SBS_OverCurrentCharge,		//!< 0x3E Word: Level 1 Charge in [mA]
    SBS_OverCurrentDischarge,		//!< 0x3F Word: Level 1 Discharge in [mA]
    SBS_HighCurrentReactionTime,	//!< 0x40 Word: Level 2 Time in [??]
    SBS_HighCurrentCharge,		//!< 0x41 Word: Level 2 Charge in [mA]
    SBS_HighCurrentDischarge,		//!< 0x42 Word: Level 2 Discharge in [mA]
    SBS_VoltageCell4	 = 0x44,	//!< 0x44 Word: in [mV]
    SBS_VoltageCell3,			//!< 0x45 Word: in [mV]
    SBS_VoltageCell2,			//!< 0x46 Word: in [mV]
    SBS_VoltageCell1,			//!< 0x47 Word: in [mV]
    SBS_CellMinVoltage	 = 0x54,	//!< 0x54 Word: in [mV]
    SBS_CellMaxVoltage,			//!< 0x55 Word: in [mV]
    SBS_CellPowerOffVoltage,		//!< 0x56 Word: in [mV]
    END_SBS_CMD,			//!< End of SBS Command Definitions
} SBS_CMD;

    /*!@brief Macro to extract size from @ref SBS_CMD enum. */
#define SBS_CMD_SIZE(cmd)	((cmd >> 8) & 0xFF)

    /*!@name SBS_03_BITS - Bits of Battery Controller Register 0x03 */
//@{
#define SBS_03_BIT_BALANCING_CELL3	12	//!< Balancing Cell 3
#define SBS_03_BIT_BALANCING_CELL2	11	//!< Balancing Cell 2
#define SBS_03_BIT_BALANCING_CELL1	10	//!< Balancing Cell 1
#define SBS_03_BIT_BALANCING_CELL4	 6	//!< Balancing Cell 4
#define SBS_03_BIT_DUVRD		 5	//!< Deep Under Voltage Recovery
#define SBS_03_BIT_CPS			 4	//!< Current Protection Status
#define SBS_03_BIT_DFE			 3	//!< Discharge FET status
#define SBS_03_BIT_CFE			 2	//!< Charge FET status
//@}

    /*!@name SBS_16_BITS - Bits of Battery Controller Register 0x16 */
//@{
#define SBS_16_BIT_OVER_CHARGE_ALARM	15	//!< Over Charge Alarm
#define SBS_16_BIT_TERM_CHARGE_ALARM	14	//!< Terminate Charge Alarm
#define SBS_16_BIT_OVER_TEMP_ALARM	12	//!< Over Temperature Alarm
#define SBS_16_BIT_TERM_DISCHARGE_ALARM	11	//!< Terminate Discharge Alarm
#define SBS_16_BIT_BATTERY_PROTECTION	10	//!< FETs have been switched off
#define SBS_16_BIT_REMAIN_CAP_ALARM	 9	//!< Remaining Capacity Alarm
#define SBS_16_BIT_REMAIN_TIME_ALARM	 8	//!< Remaining Time Alarm
#define SBS_16_BIT_INITIALIZED		 7	//!< Controller is initialized
#define SBS_16_BIT_DISCHARGING		 6	//!< Battery is discharged
#define SBS_16_BIT_FULLY_CHARGED	 5	//!< Battery is fully charged
#define SBS_16_BIT_FULLY_DISCHARGED	 4	//!< Battery is fully discharged
//@}

    /*!@brief Error code for I2C timeout, additionally to @ref
     * I2C_TransferReturn_TypeDef
     */
#define i2cTransferTimeout		-10

    /*!@brief Error code for invalid parameter, additionally to @ref
     * I2C_TransferReturn_TypeDef
     */
#define i2cInvalidParameter		-11

    /*!@brief Error code for power-fail condition, additionally to @ref
     * I2C_TransferReturn_TypeDef
     */
#define i2cPowerFail			-12

/*!@brief Structure to request Battery Information (up to two requests). */
typedef struct
{
    bool	Done;		// Flag shows when requests have been completed
    SBS_CMD	Req_1;		// Request 1
    int		Data_1;		// Result 1
    SBS_CMD	Req_2;		// Request 2, or SBS_NONE
    int		Data_2;		// Result 2
    uint8_t	Buffer[18];	// Buffer for Block Commands
} BAT_INFO;

/*================================ Global Data ===============================*/

extern volatile int16_t   g_BattMilliVolt;
extern volatile uint16_t  g_BattCapacity;

/*================================ Prototypes ================================*/

    /* Initialize or deinitialize  Battery Monitor module */
void	BatteryMonInit (void);
void	BatteryMonDeinit (void);

    /* Register read functions */
int	BatteryRegReadWord  (SBS_CMD cmd);
int	BatteryRegReadBlock (SBS_CMD cmd, uint8_t *pBuf, size_t bufSize);

    /* Info routines */
void	LogBatteryInfo (BAT_LOG_INFO_LVL infoLvl);
void	BatteryCheck (void);
void	BatteryInfoReq (SBS_CMD req_1, SBS_CMD req_2);
BAT_INFO *BatteryInfoGet (void);

    /* Power Fail Handler of the battery monitor module */
void	BatteryMonPowerFailHandler (void);


#endif /* __INC_BatteryMon_h */
