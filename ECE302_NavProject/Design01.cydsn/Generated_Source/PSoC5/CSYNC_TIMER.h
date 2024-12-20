/*******************************************************************************
* File Name: CSYNC_TIMER.h
* Version 2.80
*
*  Description:
*     Contains the function prototypes and constants available to the timer
*     user module.
*
*   Note:
*     None
*
********************************************************************************
* Copyright 2008-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#if !defined(CY_TIMER_CSYNC_TIMER_H)
#define CY_TIMER_CSYNC_TIMER_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */

extern uint8 CSYNC_TIMER_initVar;

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component Timer_v2_80 requires cy_boot v3.0 or later
#endif /* (CY_ PSOC5LP) */


/**************************************
*           Parameter Defaults
**************************************/

#define CSYNC_TIMER_Resolution                 16u
#define CSYNC_TIMER_UsingFixedFunction         0u
#define CSYNC_TIMER_UsingHWCaptureCounter      0u
#define CSYNC_TIMER_SoftwareCaptureMode        0u
#define CSYNC_TIMER_SoftwareTriggerMode        0u
#define CSYNC_TIMER_UsingHWEnable              0u
#define CSYNC_TIMER_EnableTriggerMode          0u
#define CSYNC_TIMER_InterruptOnCaptureCount    1u
#define CSYNC_TIMER_RunModeUsed                0u
#define CSYNC_TIMER_ControlRegRemoved          0u

#if defined(CSYNC_TIMER_TimerUDB_sCTRLReg_SyncCtl_ctrlreg__CONTROL_REG)
    #define CSYNC_TIMER_UDB_CONTROL_REG_REMOVED            (0u)
#elif  (CSYNC_TIMER_UsingFixedFunction)
    #define CSYNC_TIMER_UDB_CONTROL_REG_REMOVED            (0u)
#else 
    #define CSYNC_TIMER_UDB_CONTROL_REG_REMOVED            (1u)
#endif /* End CSYNC_TIMER_TimerUDB_sCTRLReg_SyncCtl_ctrlreg__CONTROL_REG */


/***************************************
*       Type defines
***************************************/


/**************************************************************************
 * Sleep Wakeup Backup structure for Timer Component
 *************************************************************************/
typedef struct
{
    uint8 TimerEnableState;
    #if(!CSYNC_TIMER_UsingFixedFunction)

        uint16 TimerUdb;
        uint8 InterruptMaskValue;
        #if (CSYNC_TIMER_UsingHWCaptureCounter)
            uint8 TimerCaptureCounter;
        #endif /* variable declarations for backing up non retention registers in CY_UDB_V1 */

        #if (!CSYNC_TIMER_UDB_CONTROL_REG_REMOVED)
            uint8 TimerControlRegister;
        #endif /* variable declaration for backing up enable state of the Timer */
    #endif /* define backup variables only for UDB implementation. Fixed function registers are all retention */

}CSYNC_TIMER_backupStruct;


/***************************************
*       Function Prototypes
***************************************/

void    CSYNC_TIMER_Start(void) ;
void    CSYNC_TIMER_Stop(void) ;

void    CSYNC_TIMER_SetInterruptMode(uint8 interruptMode) ;
uint8   CSYNC_TIMER_ReadStatusRegister(void) ;
/* Deprecated function. Do not use this in future. Retained for backward compatibility */
#define CSYNC_TIMER_GetInterruptSource() CSYNC_TIMER_ReadStatusRegister()

#if(!CSYNC_TIMER_UDB_CONTROL_REG_REMOVED)
    uint8   CSYNC_TIMER_ReadControlRegister(void) ;
    void    CSYNC_TIMER_WriteControlRegister(uint8 control) ;
#endif /* (!CSYNC_TIMER_UDB_CONTROL_REG_REMOVED) */

uint16  CSYNC_TIMER_ReadPeriod(void) ;
void    CSYNC_TIMER_WritePeriod(uint16 period) ;
uint16  CSYNC_TIMER_ReadCounter(void) ;
void    CSYNC_TIMER_WriteCounter(uint16 counter) ;
uint16  CSYNC_TIMER_ReadCapture(void) ;
void    CSYNC_TIMER_SoftwareCapture(void) ;

#if(!CSYNC_TIMER_UsingFixedFunction) /* UDB Prototypes */
    #if (CSYNC_TIMER_SoftwareCaptureMode)
        void    CSYNC_TIMER_SetCaptureMode(uint8 captureMode) ;
    #endif /* (!CSYNC_TIMER_UsingFixedFunction) */

    #if (CSYNC_TIMER_SoftwareTriggerMode)
        void    CSYNC_TIMER_SetTriggerMode(uint8 triggerMode) ;
    #endif /* (CSYNC_TIMER_SoftwareTriggerMode) */

    #if (CSYNC_TIMER_EnableTriggerMode)
        void    CSYNC_TIMER_EnableTrigger(void) ;
        void    CSYNC_TIMER_DisableTrigger(void) ;
    #endif /* (CSYNC_TIMER_EnableTriggerMode) */


    #if(CSYNC_TIMER_InterruptOnCaptureCount)
        void    CSYNC_TIMER_SetInterruptCount(uint8 interruptCount) ;
    #endif /* (CSYNC_TIMER_InterruptOnCaptureCount) */

    #if (CSYNC_TIMER_UsingHWCaptureCounter)
        void    CSYNC_TIMER_SetCaptureCount(uint8 captureCount) ;
        uint8   CSYNC_TIMER_ReadCaptureCount(void) ;
    #endif /* (CSYNC_TIMER_UsingHWCaptureCounter) */

    void CSYNC_TIMER_ClearFIFO(void) ;
#endif /* UDB Prototypes */

/* Sleep Retention APIs */
void CSYNC_TIMER_Init(void)          ;
void CSYNC_TIMER_Enable(void)        ;
void CSYNC_TIMER_SaveConfig(void)    ;
void CSYNC_TIMER_RestoreConfig(void) ;
void CSYNC_TIMER_Sleep(void)         ;
void CSYNC_TIMER_Wakeup(void)        ;


/***************************************
*   Enumerated Types and Parameters
***************************************/

/* Enumerated Type B_Timer__CaptureModes, Used in Capture Mode */
#define CSYNC_TIMER__B_TIMER__CM_NONE 0
#define CSYNC_TIMER__B_TIMER__CM_RISINGEDGE 1
#define CSYNC_TIMER__B_TIMER__CM_FALLINGEDGE 2
#define CSYNC_TIMER__B_TIMER__CM_EITHEREDGE 3
#define CSYNC_TIMER__B_TIMER__CM_SOFTWARE 4



/* Enumerated Type B_Timer__TriggerModes, Used in Trigger Mode */
#define CSYNC_TIMER__B_TIMER__TM_NONE 0x00u
#define CSYNC_TIMER__B_TIMER__TM_RISINGEDGE 0x04u
#define CSYNC_TIMER__B_TIMER__TM_FALLINGEDGE 0x08u
#define CSYNC_TIMER__B_TIMER__TM_EITHEREDGE 0x0Cu
#define CSYNC_TIMER__B_TIMER__TM_SOFTWARE 0x10u


/***************************************
*    Initialial Parameter Constants
***************************************/

#define CSYNC_TIMER_INIT_PERIOD             255u
#define CSYNC_TIMER_INIT_CAPTURE_MODE       ((uint8)((uint8)1u << CSYNC_TIMER_CTRL_CAP_MODE_SHIFT))
#define CSYNC_TIMER_INIT_TRIGGER_MODE       ((uint8)((uint8)0u << CSYNC_TIMER_CTRL_TRIG_MODE_SHIFT))
#if (CSYNC_TIMER_UsingFixedFunction)
    #define CSYNC_TIMER_INIT_INTERRUPT_MODE (((uint8)((uint8)0u << CSYNC_TIMER_STATUS_TC_INT_MASK_SHIFT)) | \
                                                  ((uint8)((uint8)1 << CSYNC_TIMER_STATUS_CAPTURE_INT_MASK_SHIFT)))
#else
    #define CSYNC_TIMER_INIT_INTERRUPT_MODE (((uint8)((uint8)0u << CSYNC_TIMER_STATUS_TC_INT_MASK_SHIFT)) | \
                                                 ((uint8)((uint8)1 << CSYNC_TIMER_STATUS_CAPTURE_INT_MASK_SHIFT)) | \
                                                 ((uint8)((uint8)0 << CSYNC_TIMER_STATUS_FIFOFULL_INT_MASK_SHIFT)))
#endif /* (CSYNC_TIMER_UsingFixedFunction) */
#define CSYNC_TIMER_INIT_CAPTURE_COUNT      (2u)
#define CSYNC_TIMER_INIT_INT_CAPTURE_COUNT  ((uint8)((uint8)(1u - 1u) << CSYNC_TIMER_CTRL_INTCNT_SHIFT))


/***************************************
*           Registers
***************************************/

#if (CSYNC_TIMER_UsingFixedFunction) /* Implementation Specific Registers and Register Constants */


    /***************************************
    *    Fixed Function Registers
    ***************************************/

    #define CSYNC_TIMER_STATUS         (*(reg8 *) CSYNC_TIMER_TimerHW__SR0 )
    /* In Fixed Function Block Status and Mask are the same register */
    #define CSYNC_TIMER_STATUS_MASK    (*(reg8 *) CSYNC_TIMER_TimerHW__SR0 )
    #define CSYNC_TIMER_CONTROL        (*(reg8 *) CSYNC_TIMER_TimerHW__CFG0)
    #define CSYNC_TIMER_CONTROL2       (*(reg8 *) CSYNC_TIMER_TimerHW__CFG1)
    #define CSYNC_TIMER_CONTROL2_PTR   ( (reg8 *) CSYNC_TIMER_TimerHW__CFG1)
    #define CSYNC_TIMER_RT1            (*(reg8 *) CSYNC_TIMER_TimerHW__RT1)
    #define CSYNC_TIMER_RT1_PTR        ( (reg8 *) CSYNC_TIMER_TimerHW__RT1)

    #if (CY_PSOC3 || CY_PSOC5LP)
        #define CSYNC_TIMER_CONTROL3       (*(reg8 *) CSYNC_TIMER_TimerHW__CFG2)
        #define CSYNC_TIMER_CONTROL3_PTR   ( (reg8 *) CSYNC_TIMER_TimerHW__CFG2)
    #endif /* (CY_PSOC3 || CY_PSOC5LP) */
    #define CSYNC_TIMER_GLOBAL_ENABLE  (*(reg8 *) CSYNC_TIMER_TimerHW__PM_ACT_CFG)
    #define CSYNC_TIMER_GLOBAL_STBY_ENABLE  (*(reg8 *) CSYNC_TIMER_TimerHW__PM_STBY_CFG)

    #define CSYNC_TIMER_CAPTURE_LSB         (* (reg16 *) CSYNC_TIMER_TimerHW__CAP0 )
    #define CSYNC_TIMER_CAPTURE_LSB_PTR       ((reg16 *) CSYNC_TIMER_TimerHW__CAP0 )
    #define CSYNC_TIMER_PERIOD_LSB          (* (reg16 *) CSYNC_TIMER_TimerHW__PER0 )
    #define CSYNC_TIMER_PERIOD_LSB_PTR        ((reg16 *) CSYNC_TIMER_TimerHW__PER0 )
    #define CSYNC_TIMER_COUNTER_LSB         (* (reg16 *) CSYNC_TIMER_TimerHW__CNT_CMP0 )
    #define CSYNC_TIMER_COUNTER_LSB_PTR       ((reg16 *) CSYNC_TIMER_TimerHW__CNT_CMP0 )


    /***************************************
    *    Register Constants
    ***************************************/

    /* Fixed Function Block Chosen */
    #define CSYNC_TIMER_BLOCK_EN_MASK                     CSYNC_TIMER_TimerHW__PM_ACT_MSK
    #define CSYNC_TIMER_BLOCK_STBY_EN_MASK                CSYNC_TIMER_TimerHW__PM_STBY_MSK

    /* Control Register Bit Locations */
    /* Interrupt Count - Not valid for Fixed Function Block */
    #define CSYNC_TIMER_CTRL_INTCNT_SHIFT                  0x00u
    /* Trigger Polarity - Not valid for Fixed Function Block */
    #define CSYNC_TIMER_CTRL_TRIG_MODE_SHIFT               0x00u
    /* Trigger Enable - Not valid for Fixed Function Block */
    #define CSYNC_TIMER_CTRL_TRIG_EN_SHIFT                 0x00u
    /* Capture Polarity - Not valid for Fixed Function Block */
    #define CSYNC_TIMER_CTRL_CAP_MODE_SHIFT                0x00u
    /* Timer Enable - As defined in Register Map, part of TMRX_CFG0 register */
    #define CSYNC_TIMER_CTRL_ENABLE_SHIFT                  0x00u

    /* Control Register Bit Masks */
    #define CSYNC_TIMER_CTRL_ENABLE                        ((uint8)((uint8)0x01u << CSYNC_TIMER_CTRL_ENABLE_SHIFT))

    /* Control2 Register Bit Masks */
    /* As defined in Register Map, Part of the TMRX_CFG1 register */
    #define CSYNC_TIMER_CTRL2_IRQ_SEL_SHIFT                 0x00u
    #define CSYNC_TIMER_CTRL2_IRQ_SEL                      ((uint8)((uint8)0x01u << CSYNC_TIMER_CTRL2_IRQ_SEL_SHIFT))

    #if (CY_PSOC5A)
        /* Use CFG1 Mode bits to set run mode */
        /* As defined by Verilog Implementation */
        #define CSYNC_TIMER_CTRL_MODE_SHIFT                 0x01u
        #define CSYNC_TIMER_CTRL_MODE_MASK                 ((uint8)((uint8)0x07u << CSYNC_TIMER_CTRL_MODE_SHIFT))
    #endif /* (CY_PSOC5A) */
    #if (CY_PSOC3 || CY_PSOC5LP)
        /* Control3 Register Bit Locations */
        #define CSYNC_TIMER_CTRL_RCOD_SHIFT        0x02u
        #define CSYNC_TIMER_CTRL_ENBL_SHIFT        0x00u
        #define CSYNC_TIMER_CTRL_MODE_SHIFT        0x00u

        /* Control3 Register Bit Masks */
        #define CSYNC_TIMER_CTRL_RCOD_MASK  ((uint8)((uint8)0x03u << CSYNC_TIMER_CTRL_RCOD_SHIFT)) /* ROD and COD bit masks */
        #define CSYNC_TIMER_CTRL_ENBL_MASK  ((uint8)((uint8)0x80u << CSYNC_TIMER_CTRL_ENBL_SHIFT)) /* HW_EN bit mask */
        #define CSYNC_TIMER_CTRL_MODE_MASK  ((uint8)((uint8)0x03u << CSYNC_TIMER_CTRL_MODE_SHIFT)) /* Run mode bit mask */

        #define CSYNC_TIMER_CTRL_RCOD       ((uint8)((uint8)0x03u << CSYNC_TIMER_CTRL_RCOD_SHIFT))
        #define CSYNC_TIMER_CTRL_ENBL       ((uint8)((uint8)0x80u << CSYNC_TIMER_CTRL_ENBL_SHIFT))
    #endif /* (CY_PSOC3 || CY_PSOC5LP) */

    /*RT1 Synch Constants: Applicable for PSoC3 and PSoC5LP */
    #define CSYNC_TIMER_RT1_SHIFT                       0x04u
    /* Sync TC and CMP bit masks */
    #define CSYNC_TIMER_RT1_MASK                        ((uint8)((uint8)0x03u << CSYNC_TIMER_RT1_SHIFT))
    #define CSYNC_TIMER_SYNC                            ((uint8)((uint8)0x03u << CSYNC_TIMER_RT1_SHIFT))
    #define CSYNC_TIMER_SYNCDSI_SHIFT                   0x00u
    /* Sync all DSI inputs with Mask  */
    #define CSYNC_TIMER_SYNCDSI_MASK                    ((uint8)((uint8)0x0Fu << CSYNC_TIMER_SYNCDSI_SHIFT))
    /* Sync all DSI inputs */
    #define CSYNC_TIMER_SYNCDSI_EN                      ((uint8)((uint8)0x0Fu << CSYNC_TIMER_SYNCDSI_SHIFT))

    #define CSYNC_TIMER_CTRL_MODE_PULSEWIDTH            ((uint8)((uint8)0x01u << CSYNC_TIMER_CTRL_MODE_SHIFT))
    #define CSYNC_TIMER_CTRL_MODE_PERIOD                ((uint8)((uint8)0x02u << CSYNC_TIMER_CTRL_MODE_SHIFT))
    #define CSYNC_TIMER_CTRL_MODE_CONTINUOUS            ((uint8)((uint8)0x00u << CSYNC_TIMER_CTRL_MODE_SHIFT))

    /* Status Register Bit Locations */
    /* As defined in Register Map, part of TMRX_SR0 register */
    #define CSYNC_TIMER_STATUS_TC_SHIFT                 0x07u
    /* As defined in Register Map, part of TMRX_SR0 register, Shared with Compare Status */
    #define CSYNC_TIMER_STATUS_CAPTURE_SHIFT            0x06u
    /* As defined in Register Map, part of TMRX_SR0 register */
    #define CSYNC_TIMER_STATUS_TC_INT_MASK_SHIFT        (CSYNC_TIMER_STATUS_TC_SHIFT - 0x04u)
    /* As defined in Register Map, part of TMRX_SR0 register, Shared with Compare Status */
    #define CSYNC_TIMER_STATUS_CAPTURE_INT_MASK_SHIFT   (CSYNC_TIMER_STATUS_CAPTURE_SHIFT - 0x04u)

    /* Status Register Bit Masks */
    #define CSYNC_TIMER_STATUS_TC                       ((uint8)((uint8)0x01u << CSYNC_TIMER_STATUS_TC_SHIFT))
    #define CSYNC_TIMER_STATUS_CAPTURE                  ((uint8)((uint8)0x01u << CSYNC_TIMER_STATUS_CAPTURE_SHIFT))
    /* Interrupt Enable Bit-Mask for interrupt on TC */
    #define CSYNC_TIMER_STATUS_TC_INT_MASK              ((uint8)((uint8)0x01u << CSYNC_TIMER_STATUS_TC_INT_MASK_SHIFT))
    /* Interrupt Enable Bit-Mask for interrupt on Capture */
    #define CSYNC_TIMER_STATUS_CAPTURE_INT_MASK         ((uint8)((uint8)0x01u << CSYNC_TIMER_STATUS_CAPTURE_INT_MASK_SHIFT))

#else   /* UDB Registers and Register Constants */


    /***************************************
    *           UDB Registers
    ***************************************/

    #define CSYNC_TIMER_STATUS              (* (reg8 *) CSYNC_TIMER_TimerUDB_rstSts_stsreg__STATUS_REG )
    #define CSYNC_TIMER_STATUS_MASK         (* (reg8 *) CSYNC_TIMER_TimerUDB_rstSts_stsreg__MASK_REG)
    #define CSYNC_TIMER_STATUS_AUX_CTRL     (* (reg8 *) CSYNC_TIMER_TimerUDB_rstSts_stsreg__STATUS_AUX_CTL_REG)
    #define CSYNC_TIMER_CONTROL             (* (reg8 *) CSYNC_TIMER_TimerUDB_sCTRLReg_SyncCtl_ctrlreg__CONTROL_REG )
    
    #if(CSYNC_TIMER_Resolution <= 8u) /* 8-bit Timer */
        #define CSYNC_TIMER_CAPTURE_LSB         (* (reg8 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__F0_REG )
        #define CSYNC_TIMER_CAPTURE_LSB_PTR       ((reg8 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__F0_REG )
        #define CSYNC_TIMER_PERIOD_LSB          (* (reg8 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__D0_REG )
        #define CSYNC_TIMER_PERIOD_LSB_PTR        ((reg8 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__D0_REG )
        #define CSYNC_TIMER_COUNTER_LSB         (* (reg8 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__A0_REG )
        #define CSYNC_TIMER_COUNTER_LSB_PTR       ((reg8 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__A0_REG )
    #elif(CSYNC_TIMER_Resolution <= 16u) /* 8-bit Timer */
        #if(CY_PSOC3) /* 8-bit addres space */
            #define CSYNC_TIMER_CAPTURE_LSB         (* (reg16 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__F0_REG )
            #define CSYNC_TIMER_CAPTURE_LSB_PTR       ((reg16 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__F0_REG )
            #define CSYNC_TIMER_PERIOD_LSB          (* (reg16 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__D0_REG )
            #define CSYNC_TIMER_PERIOD_LSB_PTR        ((reg16 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__D0_REG )
            #define CSYNC_TIMER_COUNTER_LSB         (* (reg16 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__A0_REG )
            #define CSYNC_TIMER_COUNTER_LSB_PTR       ((reg16 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__A0_REG )
        #else /* 16-bit address space */
            #define CSYNC_TIMER_CAPTURE_LSB         (* (reg16 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__16BIT_F0_REG )
            #define CSYNC_TIMER_CAPTURE_LSB_PTR       ((reg16 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__16BIT_F0_REG )
            #define CSYNC_TIMER_PERIOD_LSB          (* (reg16 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__16BIT_D0_REG )
            #define CSYNC_TIMER_PERIOD_LSB_PTR        ((reg16 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__16BIT_D0_REG )
            #define CSYNC_TIMER_COUNTER_LSB         (* (reg16 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__16BIT_A0_REG )
            #define CSYNC_TIMER_COUNTER_LSB_PTR       ((reg16 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__16BIT_A0_REG )
        #endif /* CY_PSOC3 */
    #elif(CSYNC_TIMER_Resolution <= 24u)/* 24-bit Timer */
        #define CSYNC_TIMER_CAPTURE_LSB         (* (reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__F0_REG )
        #define CSYNC_TIMER_CAPTURE_LSB_PTR       ((reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__F0_REG )
        #define CSYNC_TIMER_PERIOD_LSB          (* (reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__D0_REG )
        #define CSYNC_TIMER_PERIOD_LSB_PTR        ((reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__D0_REG )
        #define CSYNC_TIMER_COUNTER_LSB         (* (reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__A0_REG )
        #define CSYNC_TIMER_COUNTER_LSB_PTR       ((reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__A0_REG )
    #else /* 32-bit Timer */
        #if(CY_PSOC3 || CY_PSOC5) /* 8-bit address space */
            #define CSYNC_TIMER_CAPTURE_LSB         (* (reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__F0_REG )
            #define CSYNC_TIMER_CAPTURE_LSB_PTR       ((reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__F0_REG )
            #define CSYNC_TIMER_PERIOD_LSB          (* (reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__D0_REG )
            #define CSYNC_TIMER_PERIOD_LSB_PTR        ((reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__D0_REG )
            #define CSYNC_TIMER_COUNTER_LSB         (* (reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__A0_REG )
            #define CSYNC_TIMER_COUNTER_LSB_PTR       ((reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__A0_REG )
        #else /* 32-bit address space */
            #define CSYNC_TIMER_CAPTURE_LSB         (* (reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__32BIT_F0_REG )
            #define CSYNC_TIMER_CAPTURE_LSB_PTR       ((reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__32BIT_F0_REG )
            #define CSYNC_TIMER_PERIOD_LSB          (* (reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__32BIT_D0_REG )
            #define CSYNC_TIMER_PERIOD_LSB_PTR        ((reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__32BIT_D0_REG )
            #define CSYNC_TIMER_COUNTER_LSB         (* (reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__32BIT_A0_REG )
            #define CSYNC_TIMER_COUNTER_LSB_PTR       ((reg32 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__32BIT_A0_REG )
        #endif /* CY_PSOC3 || CY_PSOC5 */ 
    #endif

    #define CSYNC_TIMER_COUNTER_LSB_PTR_8BIT       ((reg8 *) CSYNC_TIMER_TimerUDB_sT16_timerdp_u0__A0_REG )
    
    #if (CSYNC_TIMER_UsingHWCaptureCounter)
        #define CSYNC_TIMER_CAP_COUNT              (*(reg8 *) CSYNC_TIMER_TimerUDB_sCapCount_counter__PERIOD_REG )
        #define CSYNC_TIMER_CAP_COUNT_PTR          ( (reg8 *) CSYNC_TIMER_TimerUDB_sCapCount_counter__PERIOD_REG )
        #define CSYNC_TIMER_CAPTURE_COUNT_CTRL     (*(reg8 *) CSYNC_TIMER_TimerUDB_sCapCount_counter__CONTROL_AUX_CTL_REG )
        #define CSYNC_TIMER_CAPTURE_COUNT_CTRL_PTR ( (reg8 *) CSYNC_TIMER_TimerUDB_sCapCount_counter__CONTROL_AUX_CTL_REG )
    #endif /* (CSYNC_TIMER_UsingHWCaptureCounter) */


    /***************************************
    *       Register Constants
    ***************************************/

    /* Control Register Bit Locations */
    #define CSYNC_TIMER_CTRL_INTCNT_SHIFT              0x00u       /* As defined by Verilog Implementation */
    #define CSYNC_TIMER_CTRL_TRIG_MODE_SHIFT           0x02u       /* As defined by Verilog Implementation */
    #define CSYNC_TIMER_CTRL_TRIG_EN_SHIFT             0x04u       /* As defined by Verilog Implementation */
    #define CSYNC_TIMER_CTRL_CAP_MODE_SHIFT            0x05u       /* As defined by Verilog Implementation */
    #define CSYNC_TIMER_CTRL_ENABLE_SHIFT              0x07u       /* As defined by Verilog Implementation */

    /* Control Register Bit Masks */
    #define CSYNC_TIMER_CTRL_INTCNT_MASK               ((uint8)((uint8)0x03u << CSYNC_TIMER_CTRL_INTCNT_SHIFT))
    #define CSYNC_TIMER_CTRL_TRIG_MODE_MASK            ((uint8)((uint8)0x03u << CSYNC_TIMER_CTRL_TRIG_MODE_SHIFT))
    #define CSYNC_TIMER_CTRL_TRIG_EN                   ((uint8)((uint8)0x01u << CSYNC_TIMER_CTRL_TRIG_EN_SHIFT))
    #define CSYNC_TIMER_CTRL_CAP_MODE_MASK             ((uint8)((uint8)0x03u << CSYNC_TIMER_CTRL_CAP_MODE_SHIFT))
    #define CSYNC_TIMER_CTRL_ENABLE                    ((uint8)((uint8)0x01u << CSYNC_TIMER_CTRL_ENABLE_SHIFT))

    /* Bit Counter (7-bit) Control Register Bit Definitions */
    /* As defined by the Register map for the AUX Control Register */
    #define CSYNC_TIMER_CNTR_ENABLE                    0x20u

    /* Status Register Bit Locations */
    #define CSYNC_TIMER_STATUS_TC_SHIFT                0x00u  /* As defined by Verilog Implementation */
    #define CSYNC_TIMER_STATUS_CAPTURE_SHIFT           0x01u  /* As defined by Verilog Implementation */
    #define CSYNC_TIMER_STATUS_TC_INT_MASK_SHIFT       CSYNC_TIMER_STATUS_TC_SHIFT
    #define CSYNC_TIMER_STATUS_CAPTURE_INT_MASK_SHIFT  CSYNC_TIMER_STATUS_CAPTURE_SHIFT
    #define CSYNC_TIMER_STATUS_FIFOFULL_SHIFT          0x02u  /* As defined by Verilog Implementation */
    #define CSYNC_TIMER_STATUS_FIFONEMP_SHIFT          0x03u  /* As defined by Verilog Implementation */
    #define CSYNC_TIMER_STATUS_FIFOFULL_INT_MASK_SHIFT CSYNC_TIMER_STATUS_FIFOFULL_SHIFT

    /* Status Register Bit Masks */
    /* Sticky TC Event Bit-Mask */
    #define CSYNC_TIMER_STATUS_TC                      ((uint8)((uint8)0x01u << CSYNC_TIMER_STATUS_TC_SHIFT))
    /* Sticky Capture Event Bit-Mask */
    #define CSYNC_TIMER_STATUS_CAPTURE                 ((uint8)((uint8)0x01u << CSYNC_TIMER_STATUS_CAPTURE_SHIFT))
    /* Interrupt Enable Bit-Mask */
    #define CSYNC_TIMER_STATUS_TC_INT_MASK             ((uint8)((uint8)0x01u << CSYNC_TIMER_STATUS_TC_SHIFT))
    /* Interrupt Enable Bit-Mask */
    #define CSYNC_TIMER_STATUS_CAPTURE_INT_MASK        ((uint8)((uint8)0x01u << CSYNC_TIMER_STATUS_CAPTURE_SHIFT))
    /* NOT-Sticky FIFO Full Bit-Mask */
    #define CSYNC_TIMER_STATUS_FIFOFULL                ((uint8)((uint8)0x01u << CSYNC_TIMER_STATUS_FIFOFULL_SHIFT))
    /* NOT-Sticky FIFO Not Empty Bit-Mask */
    #define CSYNC_TIMER_STATUS_FIFONEMP                ((uint8)((uint8)0x01u << CSYNC_TIMER_STATUS_FIFONEMP_SHIFT))
    /* Interrupt Enable Bit-Mask */
    #define CSYNC_TIMER_STATUS_FIFOFULL_INT_MASK       ((uint8)((uint8)0x01u << CSYNC_TIMER_STATUS_FIFOFULL_SHIFT))

    #define CSYNC_TIMER_STATUS_ACTL_INT_EN             0x10u   /* As defined for the ACTL Register */

    /* Datapath Auxillary Control Register definitions */
    #define CSYNC_TIMER_AUX_CTRL_FIFO0_CLR             0x01u   /* As defined by Register map */
    #define CSYNC_TIMER_AUX_CTRL_FIFO1_CLR             0x02u   /* As defined by Register map */
    #define CSYNC_TIMER_AUX_CTRL_FIFO0_LVL             0x04u   /* As defined by Register map */
    #define CSYNC_TIMER_AUX_CTRL_FIFO1_LVL             0x08u   /* As defined by Register map */
    #define CSYNC_TIMER_STATUS_ACTL_INT_EN_MASK        0x10u   /* As defined for the ACTL Register */

#endif /* Implementation Specific Registers and Register Constants */

#endif  /* CY_TIMER_CSYNC_TIMER_H */


/* [] END OF FILE */
