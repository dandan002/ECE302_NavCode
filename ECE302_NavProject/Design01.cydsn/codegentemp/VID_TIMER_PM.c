/*******************************************************************************
* File Name: VID_TIMER_PM.c
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

#include "VID_TIMER.h"

static VID_TIMER_backupStruct VID_TIMER_backup;


/*******************************************************************************
* Function Name: VID_TIMER_SaveConfig
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
*  VID_TIMER_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void VID_TIMER_SaveConfig(void) 
{
    #if (!VID_TIMER_UsingFixedFunction)
        VID_TIMER_backup.TimerUdb = VID_TIMER_ReadCounter();
        VID_TIMER_backup.InterruptMaskValue = VID_TIMER_STATUS_MASK;
        #if (VID_TIMER_UsingHWCaptureCounter)
            VID_TIMER_backup.TimerCaptureCounter = VID_TIMER_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!VID_TIMER_UDB_CONTROL_REG_REMOVED)
            VID_TIMER_backup.TimerControlRegister = VID_TIMER_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: VID_TIMER_RestoreConfig
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
*  VID_TIMER_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void VID_TIMER_RestoreConfig(void) 
{   
    #if (!VID_TIMER_UsingFixedFunction)

        VID_TIMER_WriteCounter(VID_TIMER_backup.TimerUdb);
        VID_TIMER_STATUS_MASK =VID_TIMER_backup.InterruptMaskValue;
        #if (VID_TIMER_UsingHWCaptureCounter)
            VID_TIMER_SetCaptureCount(VID_TIMER_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!VID_TIMER_UDB_CONTROL_REG_REMOVED)
            VID_TIMER_WriteControlRegister(VID_TIMER_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: VID_TIMER_Sleep
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
*  VID_TIMER_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void VID_TIMER_Sleep(void) 
{
    #if(!VID_TIMER_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(VID_TIMER_CTRL_ENABLE == (VID_TIMER_CONTROL & VID_TIMER_CTRL_ENABLE))
        {
            /* Timer is enabled */
            VID_TIMER_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            VID_TIMER_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    VID_TIMER_Stop();
    VID_TIMER_SaveConfig();
}


/*******************************************************************************
* Function Name: VID_TIMER_Wakeup
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
*  VID_TIMER_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void VID_TIMER_Wakeup(void) 
{
    VID_TIMER_RestoreConfig();
    #if(!VID_TIMER_UDB_CONTROL_REG_REMOVED)
        if(VID_TIMER_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                VID_TIMER_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
