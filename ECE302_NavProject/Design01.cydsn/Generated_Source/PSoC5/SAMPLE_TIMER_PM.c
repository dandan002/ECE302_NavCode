/*******************************************************************************
* File Name: SAMPLE_TIMER_PM.c
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

#include "SAMPLE_TIMER.h"

static SAMPLE_TIMER_backupStruct SAMPLE_TIMER_backup;


/*******************************************************************************
* Function Name: SAMPLE_TIMER_SaveConfig
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
*  SAMPLE_TIMER_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void SAMPLE_TIMER_SaveConfig(void) 
{
    #if (!SAMPLE_TIMER_UsingFixedFunction)
        SAMPLE_TIMER_backup.TimerUdb = SAMPLE_TIMER_ReadCounter();
        SAMPLE_TIMER_backup.InterruptMaskValue = SAMPLE_TIMER_STATUS_MASK;
        #if (SAMPLE_TIMER_UsingHWCaptureCounter)
            SAMPLE_TIMER_backup.TimerCaptureCounter = SAMPLE_TIMER_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!SAMPLE_TIMER_UDB_CONTROL_REG_REMOVED)
            SAMPLE_TIMER_backup.TimerControlRegister = SAMPLE_TIMER_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: SAMPLE_TIMER_RestoreConfig
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
*  SAMPLE_TIMER_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void SAMPLE_TIMER_RestoreConfig(void) 
{   
    #if (!SAMPLE_TIMER_UsingFixedFunction)

        SAMPLE_TIMER_WriteCounter(SAMPLE_TIMER_backup.TimerUdb);
        SAMPLE_TIMER_STATUS_MASK =SAMPLE_TIMER_backup.InterruptMaskValue;
        #if (SAMPLE_TIMER_UsingHWCaptureCounter)
            SAMPLE_TIMER_SetCaptureCount(SAMPLE_TIMER_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!SAMPLE_TIMER_UDB_CONTROL_REG_REMOVED)
            SAMPLE_TIMER_WriteControlRegister(SAMPLE_TIMER_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: SAMPLE_TIMER_Sleep
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
*  SAMPLE_TIMER_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void SAMPLE_TIMER_Sleep(void) 
{
    #if(!SAMPLE_TIMER_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(SAMPLE_TIMER_CTRL_ENABLE == (SAMPLE_TIMER_CONTROL & SAMPLE_TIMER_CTRL_ENABLE))
        {
            /* Timer is enabled */
            SAMPLE_TIMER_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            SAMPLE_TIMER_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    SAMPLE_TIMER_Stop();
    SAMPLE_TIMER_SaveConfig();
}


/*******************************************************************************
* Function Name: SAMPLE_TIMER_Wakeup
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
*  SAMPLE_TIMER_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void SAMPLE_TIMER_Wakeup(void) 
{
    SAMPLE_TIMER_RestoreConfig();
    #if(!SAMPLE_TIMER_UDB_CONTROL_REG_REMOVED)
        if(SAMPLE_TIMER_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                SAMPLE_TIMER_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
