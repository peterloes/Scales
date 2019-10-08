/***************************************************************************//**
 * @file
 * @brief	Header file of module Scales.c
 * @author	Peter Loës
 * @version	2017-11-08
 ****************************************************************************//*
Revision History:
2019-05-22,rage	Added global configuration values.
2017-11-08,Loës	Initial version.
*/

#ifndef __INC_Scales_h
#define __INC_Scales_h

/*=============================== Header Files ===============================*/

#include "config.h"		// include project configuration parameters
#include "Control.h"

/*================================ Global Data ===============================*/

extern PWR_OUT   g_ScalesPower;
extern uint32_t  g_ScalesCfg_TW;
extern uint32_t  g_ScalesCfg_TI;
extern uint32_t  g_ScalesCfg_NR;
extern uint32_t  g_ScalesCfg_NT;
extern uint32_t  g_ScalesCfg_SD;
extern uint32_t  g_ScalesCfg_MT;
extern uint32_t  g_ScalesCfg_TL;

/*================================ Prototypes ================================*/

    /* Initialize Scales Amplifier */
void	ScalesInit (void);

    /* Enable Scales */
void	ScalesEnable (void);

    /* Disable Scales */
void	ScalesDisable (void);

    /* Check Scales */
void	ScalesCheck (void);

    /* Power Off Scales */
void	ScalesPowerOff (void);

    /* Scales Power Fail Handler */
void	ScalesPowerFailHandler (void);

    /* Send Command to Scales */
void	SendCmd (const char *pCmdStr);


#endif /* __INC_Scales_h */
