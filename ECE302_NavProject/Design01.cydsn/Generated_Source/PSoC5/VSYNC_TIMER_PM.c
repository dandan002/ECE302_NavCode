/*******************************************************************************
* File Name: VSYNC_TIMER_PM.c
* Version 2.80
*
*  Description:
*     This file provides the power management source code to API for the
*     Timer.
*
*   Note:
*     None
*
*******************************************************************************
* Copyright 2008-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#include "VSYNC_TIMER.h"

static VSYNC_TIMER_backupStruct VSYNC_TIMER_backup;


/*******************************************************************************
* Function Name: VSYNC_TIMER_SaveConfig
********************************************************************************
*
* Summary:
*     Save the current user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  VSYNC_TIMER_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void VSYNC_TIMER_SaveConfig(void) 
{
    #if (!VSYNC_TIMER_UsingFixedFunction)
        VSYNC_TIMER_backup.TimerUdb = VSYNC_TIMER_ReadCounter();
        VSYNC_TIMER_backup.InterruptMaskValue = VSYNC_TIMER_STATUS_MASK;
        #if (VSYNC_TIMER_UsingHWCaptureCounter)
            VSYNC_TIMER_backup.TimerCaptureCounter = VSYNC_TIMER_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!VSYNC_TIMER_UDB_CONTROL_REG_REMOVED)
            VSYNC_TIMER_backup.TimerControlRegister = VSYNC_TIMER_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: VSYNC_TIMER_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  VSYNC_TIMER_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void VSYNC_TIMER_RestoreConfig(void) 
{   
    #if (!VSYNC_TIMER_UsingFixedFunction)

        VSYNC_TIMER_WriteCounter(VSYNC_TIMER_backup.TimerUdb);
        VSYNC_TIMER_STATUS_MASK =VSYNC_TIMER_backup.InterruptMaskValue;
        #if (VSYNC_TIMER_UsingHWCaptureCounter)
            VSYNC_TIMER_SetCaptureCount(VSYNC_TIMER_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!VSYNC_TIMER_UDB_CONTROL_REG_REMOVED)
            VSYNC_TIMER_WriteControlRegister(VSYNC_TIMER_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: VSYNC_TIMER_Sleep
********************************************************************************
*
* Summary:
*     Stop and Save the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  VSYNC_TIMER_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void VSYNC_TIMER_Sleep(void) 
{
    #if(!VSYNC_TIMER_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(VSYNC_TIMER_CTRL_ENABLE == (VSYNC_TIMER_CONTROL & VSYNC_TIMER_CTRL_ENABLE))
        {
            /* Timer is enabled */
            VSYNC_TIMER_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            VSYNC_TIMER_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    VSYNC_TIMER_Stop();
    VSYNC_TIMER_SaveConfig();
}


/*******************************************************************************
* Function Name: VSYNC_TIMER_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  VSYNC_TIMER_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void VSYNC_TIMER_Wakeup(void) 
{
    VSYNC_TIMER_RestoreConfig();
    #if(!VSYNC_TIMER_UDB_CONTROL_REG_REMOVED)
        if(VSYNC_TIMER_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                VSYNC_TIMER_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
