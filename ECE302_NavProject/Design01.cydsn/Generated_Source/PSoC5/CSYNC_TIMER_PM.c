/*******************************************************************************
* File Name: CSYNC_TIMER_PM.c
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

#include "CSYNC_TIMER.h"

static CSYNC_TIMER_backupStruct CSYNC_TIMER_backup;


/*******************************************************************************
* Function Name: CSYNC_TIMER_SaveConfig
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
*  CSYNC_TIMER_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void CSYNC_TIMER_SaveConfig(void) 
{
    #if (!CSYNC_TIMER_UsingFixedFunction)
        CSYNC_TIMER_backup.TimerUdb = CSYNC_TIMER_ReadCounter();
        CSYNC_TIMER_backup.InterruptMaskValue = CSYNC_TIMER_STATUS_MASK;
        #if (CSYNC_TIMER_UsingHWCaptureCounter)
            CSYNC_TIMER_backup.TimerCaptureCounter = CSYNC_TIMER_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!CSYNC_TIMER_UDB_CONTROL_REG_REMOVED)
            CSYNC_TIMER_backup.TimerControlRegister = CSYNC_TIMER_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: CSYNC_TIMER_RestoreConfig
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
*  CSYNC_TIMER_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void CSYNC_TIMER_RestoreConfig(void) 
{   
    #if (!CSYNC_TIMER_UsingFixedFunction)

        CSYNC_TIMER_WriteCounter(CSYNC_TIMER_backup.TimerUdb);
        CSYNC_TIMER_STATUS_MASK =CSYNC_TIMER_backup.InterruptMaskValue;
        #if (CSYNC_TIMER_UsingHWCaptureCounter)
            CSYNC_TIMER_SetCaptureCount(CSYNC_TIMER_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!CSYNC_TIMER_UDB_CONTROL_REG_REMOVED)
            CSYNC_TIMER_WriteControlRegister(CSYNC_TIMER_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: CSYNC_TIMER_Sleep
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
*  CSYNC_TIMER_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void CSYNC_TIMER_Sleep(void) 
{
    #if(!CSYNC_TIMER_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(CSYNC_TIMER_CTRL_ENABLE == (CSYNC_TIMER_CONTROL & CSYNC_TIMER_CTRL_ENABLE))
        {
            /* Timer is enabled */
            CSYNC_TIMER_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            CSYNC_TIMER_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    CSYNC_TIMER_Stop();
    CSYNC_TIMER_SaveConfig();
}


/*******************************************************************************
* Function Name: CSYNC_TIMER_Wakeup
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
*  CSYNC_TIMER_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void CSYNC_TIMER_Wakeup(void) 
{
    CSYNC_TIMER_RestoreConfig();
    #if(!CSYNC_TIMER_UDB_CONTROL_REG_REMOVED)
        if(CSYNC_TIMER_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                CSYNC_TIMER_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
