/*******************************************************************************
* File Name: SERVO_PWM.h
* Version 3.30
*
* Description:
*  Contains the prototypes and constants for the functions available to the
*  PWM user module.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#if !defined(CY_PWM_SERVO_PWM_H)
#define CY_PWM_SERVO_PWM_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */

extern uint8 SERVO_PWM_initVar;


/***************************************
* Conditional Compilation Parameters
***************************************/
#define SERVO_PWM_Resolution                     (16u)
#define SERVO_PWM_UsingFixedFunction             (0u)
#define SERVO_PWM_DeadBandMode                   (0u)
#define SERVO_PWM_KillModeMinTime                (0u)
#define SERVO_PWM_KillMode                       (0u)
#define SERVO_PWM_PWMMode                        (0u)
#define SERVO_PWM_PWMModeIsCenterAligned         (0u)
#define SERVO_PWM_DeadBandUsed                   (0u)
#define SERVO_PWM_DeadBand2_4                    (0u)

#if !defined(SERVO_PWM_PWMUDB_genblk8_stsreg__REMOVED)
    #define SERVO_PWM_UseStatus                  (1u)
#else
    #define SERVO_PWM_UseStatus                  (0u)
#endif /* !defined(SERVO_PWM_PWMUDB_genblk8_stsreg__REMOVED) */

#if !defined(SERVO_PWM_PWMUDB_genblk1_ctrlreg__REMOVED)
    #define SERVO_PWM_UseControl                 (1u)
#else
    #define SERVO_PWM_UseControl                 (0u)
#endif /* !defined(SERVO_PWM_PWMUDB_genblk1_ctrlreg__REMOVED) */

#define SERVO_PWM_UseOneCompareMode              (1u)
#define SERVO_PWM_MinimumKillTime                (1u)
#define SERVO_PWM_EnableMode                     (0u)

#define SERVO_PWM_CompareMode1SW                 (0u)
#define SERVO_PWM_CompareMode2SW                 (0u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component PWM_v3_30 requires cy_boot v3.0 or later
#endif /* (CY_ PSOC5LP) */

/* Use Kill Mode Enumerated Types */
#define SERVO_PWM__B_PWM__DISABLED 0
#define SERVO_PWM__B_PWM__ASYNCHRONOUS 1
#define SERVO_PWM__B_PWM__SINGLECYCLE 2
#define SERVO_PWM__B_PWM__LATCHED 3
#define SERVO_PWM__B_PWM__MINTIME 4


/* Use Dead Band Mode Enumerated Types */
#define SERVO_PWM__B_PWM__DBMDISABLED 0
#define SERVO_PWM__B_PWM__DBM_2_4_CLOCKS 1
#define SERVO_PWM__B_PWM__DBM_256_CLOCKS 2


/* Used PWM Mode Enumerated Types */
#define SERVO_PWM__B_PWM__ONE_OUTPUT 0
#define SERVO_PWM__B_PWM__TWO_OUTPUTS 1
#define SERVO_PWM__B_PWM__DUAL_EDGE 2
#define SERVO_PWM__B_PWM__CENTER_ALIGN 3
#define SERVO_PWM__B_PWM__DITHER 5
#define SERVO_PWM__B_PWM__HARDWARESELECT 4


/* Used PWM Compare Mode Enumerated Types */
#define SERVO_PWM__B_PWM__LESS_THAN 1
#define SERVO_PWM__B_PWM__LESS_THAN_OR_EQUAL 2
#define SERVO_PWM__B_PWM__GREATER_THAN 3
#define SERVO_PWM__B_PWM__GREATER_THAN_OR_EQUAL_TO 4
#define SERVO_PWM__B_PWM__EQUAL 0
#define SERVO_PWM__B_PWM__FIRMWARE 5



/***************************************
* Data Struct Definition
***************************************/


/**************************************************************************
 * Sleep Wakeup Backup structure for PWM Component
 *************************************************************************/
typedef struct
{

    uint8 PWMEnableState;

    #if(!SERVO_PWM_UsingFixedFunction)
        uint16 PWMUdb;               /* PWM Current Counter value  */
        #if(!SERVO_PWM_PWMModeIsCenterAligned)
            uint16 PWMPeriod;
        #endif /* (!SERVO_PWM_PWMModeIsCenterAligned) */
        #if (SERVO_PWM_UseStatus)
            uint8 InterruptMaskValue;   /* PWM Current Interrupt Mask */
        #endif /* (SERVO_PWM_UseStatus) */

        /* Backup for Deadband parameters */
        #if(SERVO_PWM_DeadBandMode == SERVO_PWM__B_PWM__DBM_256_CLOCKS || \
            SERVO_PWM_DeadBandMode == SERVO_PWM__B_PWM__DBM_2_4_CLOCKS)
            uint8 PWMdeadBandValue; /* Dead Band Counter Current Value */
        #endif /* deadband count is either 2-4 clocks or 256 clocks */

        /* Backup Kill Mode Counter*/
        #if(SERVO_PWM_KillModeMinTime)
            uint8 PWMKillCounterPeriod; /* Kill Mode period value */
        #endif /* (SERVO_PWM_KillModeMinTime) */

        /* Backup control register */
        #if(SERVO_PWM_UseControl)
            uint8 PWMControlRegister; /* PWM Control Register value */
        #endif /* (SERVO_PWM_UseControl) */

    #endif /* (!SERVO_PWM_UsingFixedFunction) */

}SERVO_PWM_backupStruct;


/***************************************
*        Function Prototypes
 **************************************/

void    SERVO_PWM_Start(void) ;
void    SERVO_PWM_Stop(void) ;

#if (SERVO_PWM_UseStatus || SERVO_PWM_UsingFixedFunction)
    void  SERVO_PWM_SetInterruptMode(uint8 interruptMode) ;
    uint8 SERVO_PWM_ReadStatusRegister(void) ;
#endif /* (SERVO_PWM_UseStatus || SERVO_PWM_UsingFixedFunction) */

#define SERVO_PWM_GetInterruptSource() SERVO_PWM_ReadStatusRegister()

#if (SERVO_PWM_UseControl)
    uint8 SERVO_PWM_ReadControlRegister(void) ;
    void  SERVO_PWM_WriteControlRegister(uint8 control)
          ;
#endif /* (SERVO_PWM_UseControl) */

#if (SERVO_PWM_UseOneCompareMode)
   #if (SERVO_PWM_CompareMode1SW)
       void    SERVO_PWM_SetCompareMode(uint8 comparemode)
               ;
   #endif /* (SERVO_PWM_CompareMode1SW) */
#else
    #if (SERVO_PWM_CompareMode1SW)
        void    SERVO_PWM_SetCompareMode1(uint8 comparemode)
                ;
    #endif /* (SERVO_PWM_CompareMode1SW) */
    #if (SERVO_PWM_CompareMode2SW)
        void    SERVO_PWM_SetCompareMode2(uint8 comparemode)
                ;
    #endif /* (SERVO_PWM_CompareMode2SW) */
#endif /* (SERVO_PWM_UseOneCompareMode) */

#if (!SERVO_PWM_UsingFixedFunction)
    uint16   SERVO_PWM_ReadCounter(void) ;
    uint16 SERVO_PWM_ReadCapture(void) ;

    #if (SERVO_PWM_UseStatus)
            void SERVO_PWM_ClearFIFO(void) ;
    #endif /* (SERVO_PWM_UseStatus) */

    void    SERVO_PWM_WriteCounter(uint16 counter)
            ;
#endif /* (!SERVO_PWM_UsingFixedFunction) */

void    SERVO_PWM_WritePeriod(uint16 period)
        ;
uint16 SERVO_PWM_ReadPeriod(void) ;

#if (SERVO_PWM_UseOneCompareMode)
    void    SERVO_PWM_WriteCompare(uint16 compare)
            ;
    uint16 SERVO_PWM_ReadCompare(void) ;
#else
    void    SERVO_PWM_WriteCompare1(uint16 compare)
            ;
    uint16 SERVO_PWM_ReadCompare1(void) ;
    void    SERVO_PWM_WriteCompare2(uint16 compare)
            ;
    uint16 SERVO_PWM_ReadCompare2(void) ;
#endif /* (SERVO_PWM_UseOneCompareMode) */


#if (SERVO_PWM_DeadBandUsed)
    void    SERVO_PWM_WriteDeadTime(uint8 deadtime) ;
    uint8   SERVO_PWM_ReadDeadTime(void) ;
#endif /* (SERVO_PWM_DeadBandUsed) */

#if ( SERVO_PWM_KillModeMinTime)
    void SERVO_PWM_WriteKillTime(uint8 killtime) ;
    uint8 SERVO_PWM_ReadKillTime(void) ;
#endif /* ( SERVO_PWM_KillModeMinTime) */

void SERVO_PWM_Init(void) ;
void SERVO_PWM_Enable(void) ;
void SERVO_PWM_Sleep(void) ;
void SERVO_PWM_Wakeup(void) ;
void SERVO_PWM_SaveConfig(void) ;
void SERVO_PWM_RestoreConfig(void) ;


/***************************************
*         Initialization Values
**************************************/
#define SERVO_PWM_INIT_PERIOD_VALUE          (20000u)
#define SERVO_PWM_INIT_COMPARE_VALUE1        (1500u)
#define SERVO_PWM_INIT_COMPARE_VALUE2        (63u)
#define SERVO_PWM_INIT_INTERRUPTS_MODE       (uint8)(((uint8)(0u <<   \
                                                    SERVO_PWM_STATUS_TC_INT_EN_MASK_SHIFT)) | \
                                                    (uint8)((uint8)(0u <<  \
                                                    SERVO_PWM_STATUS_CMP2_INT_EN_MASK_SHIFT)) | \
                                                    (uint8)((uint8)(0u <<  \
                                                    SERVO_PWM_STATUS_CMP1_INT_EN_MASK_SHIFT )) | \
                                                    (uint8)((uint8)(0u <<  \
                                                    SERVO_PWM_STATUS_KILL_INT_EN_MASK_SHIFT )))
#define SERVO_PWM_DEFAULT_COMPARE2_MODE      (uint8)((uint8)1u <<  SERVO_PWM_CTRL_CMPMODE2_SHIFT)
#define SERVO_PWM_DEFAULT_COMPARE1_MODE      (uint8)((uint8)1u <<  SERVO_PWM_CTRL_CMPMODE1_SHIFT)
#define SERVO_PWM_INIT_DEAD_TIME             (1u)


/********************************
*         Registers
******************************** */

#if (SERVO_PWM_UsingFixedFunction)
   #define SERVO_PWM_PERIOD_LSB              (*(reg16 *) SERVO_PWM_PWMHW__PER0)
   #define SERVO_PWM_PERIOD_LSB_PTR          ( (reg16 *) SERVO_PWM_PWMHW__PER0)
   #define SERVO_PWM_COMPARE1_LSB            (*(reg16 *) SERVO_PWM_PWMHW__CNT_CMP0)
   #define SERVO_PWM_COMPARE1_LSB_PTR        ( (reg16 *) SERVO_PWM_PWMHW__CNT_CMP0)
   #define SERVO_PWM_COMPARE2_LSB            (0x00u)
   #define SERVO_PWM_COMPARE2_LSB_PTR        (0x00u)
   #define SERVO_PWM_COUNTER_LSB             (*(reg16 *) SERVO_PWM_PWMHW__CNT_CMP0)
   #define SERVO_PWM_COUNTER_LSB_PTR         ( (reg16 *) SERVO_PWM_PWMHW__CNT_CMP0)
   #define SERVO_PWM_CAPTURE_LSB             (*(reg16 *) SERVO_PWM_PWMHW__CAP0)
   #define SERVO_PWM_CAPTURE_LSB_PTR         ( (reg16 *) SERVO_PWM_PWMHW__CAP0)
   #define SERVO_PWM_RT1                     (*(reg8 *)  SERVO_PWM_PWMHW__RT1)
   #define SERVO_PWM_RT1_PTR                 ( (reg8 *)  SERVO_PWM_PWMHW__RT1)

#else
   #if (SERVO_PWM_Resolution == 8u) /* 8bit - PWM */

       #if(SERVO_PWM_PWMModeIsCenterAligned)
           #define SERVO_PWM_PERIOD_LSB      (*(reg8 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__D1_REG)
           #define SERVO_PWM_PERIOD_LSB_PTR  ((reg8 *)   SERVO_PWM_PWMUDB_sP16_pwmdp_u0__D1_REG)
       #else
           #define SERVO_PWM_PERIOD_LSB      (*(reg8 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__F0_REG)
           #define SERVO_PWM_PERIOD_LSB_PTR  ((reg8 *)   SERVO_PWM_PWMUDB_sP16_pwmdp_u0__F0_REG)
       #endif /* (SERVO_PWM_PWMModeIsCenterAligned) */

       #define SERVO_PWM_COMPARE1_LSB        (*(reg8 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__D0_REG)
       #define SERVO_PWM_COMPARE1_LSB_PTR    ((reg8 *)   SERVO_PWM_PWMUDB_sP16_pwmdp_u0__D0_REG)
       #define SERVO_PWM_COMPARE2_LSB        (*(reg8 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__D1_REG)
       #define SERVO_PWM_COMPARE2_LSB_PTR    ((reg8 *)   SERVO_PWM_PWMUDB_sP16_pwmdp_u0__D1_REG)
       #define SERVO_PWM_COUNTERCAP_LSB      (*(reg8 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__A1_REG)
       #define SERVO_PWM_COUNTERCAP_LSB_PTR  ((reg8 *)   SERVO_PWM_PWMUDB_sP16_pwmdp_u0__A1_REG)
       #define SERVO_PWM_COUNTER_LSB         (*(reg8 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__A0_REG)
       #define SERVO_PWM_COUNTER_LSB_PTR     ((reg8 *)   SERVO_PWM_PWMUDB_sP16_pwmdp_u0__A0_REG)
       #define SERVO_PWM_CAPTURE_LSB         (*(reg8 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__F1_REG)
       #define SERVO_PWM_CAPTURE_LSB_PTR     ((reg8 *)   SERVO_PWM_PWMUDB_sP16_pwmdp_u0__F1_REG)

   #else
        #if(CY_PSOC3) /* 8-bit address space */
            #if(SERVO_PWM_PWMModeIsCenterAligned)
               #define SERVO_PWM_PERIOD_LSB      (*(reg16 *) SERVO_PWM_PWMUDB_sP16_pwmdp_u0__D1_REG)
               #define SERVO_PWM_PERIOD_LSB_PTR  ((reg16 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__D1_REG)
            #else
               #define SERVO_PWM_PERIOD_LSB      (*(reg16 *) SERVO_PWM_PWMUDB_sP16_pwmdp_u0__F0_REG)
               #define SERVO_PWM_PERIOD_LSB_PTR  ((reg16 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__F0_REG)
            #endif /* (SERVO_PWM_PWMModeIsCenterAligned) */

            #define SERVO_PWM_COMPARE1_LSB       (*(reg16 *) SERVO_PWM_PWMUDB_sP16_pwmdp_u0__D0_REG)
            #define SERVO_PWM_COMPARE1_LSB_PTR   ((reg16 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__D0_REG)
            #define SERVO_PWM_COMPARE2_LSB       (*(reg16 *) SERVO_PWM_PWMUDB_sP16_pwmdp_u0__D1_REG)
            #define SERVO_PWM_COMPARE2_LSB_PTR   ((reg16 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__D1_REG)
            #define SERVO_PWM_COUNTERCAP_LSB     (*(reg16 *) SERVO_PWM_PWMUDB_sP16_pwmdp_u0__A1_REG)
            #define SERVO_PWM_COUNTERCAP_LSB_PTR ((reg16 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__A1_REG)
            #define SERVO_PWM_COUNTER_LSB        (*(reg16 *) SERVO_PWM_PWMUDB_sP16_pwmdp_u0__A0_REG)
            #define SERVO_PWM_COUNTER_LSB_PTR    ((reg16 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__A0_REG)
            #define SERVO_PWM_CAPTURE_LSB        (*(reg16 *) SERVO_PWM_PWMUDB_sP16_pwmdp_u0__F1_REG)
            #define SERVO_PWM_CAPTURE_LSB_PTR    ((reg16 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__F1_REG)
        #else
            #if(SERVO_PWM_PWMModeIsCenterAligned)
               #define SERVO_PWM_PERIOD_LSB      (*(reg16 *) SERVO_PWM_PWMUDB_sP16_pwmdp_u0__16BIT_D1_REG)
               #define SERVO_PWM_PERIOD_LSB_PTR  ((reg16 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__16BIT_D1_REG)
            #else
               #define SERVO_PWM_PERIOD_LSB      (*(reg16 *) SERVO_PWM_PWMUDB_sP16_pwmdp_u0__16BIT_F0_REG)
               #define SERVO_PWM_PERIOD_LSB_PTR  ((reg16 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__16BIT_F0_REG)
            #endif /* (SERVO_PWM_PWMModeIsCenterAligned) */

            #define SERVO_PWM_COMPARE1_LSB       (*(reg16 *) SERVO_PWM_PWMUDB_sP16_pwmdp_u0__16BIT_D0_REG)
            #define SERVO_PWM_COMPARE1_LSB_PTR   ((reg16 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__16BIT_D0_REG)
            #define SERVO_PWM_COMPARE2_LSB       (*(reg16 *) SERVO_PWM_PWMUDB_sP16_pwmdp_u0__16BIT_D1_REG)
            #define SERVO_PWM_COMPARE2_LSB_PTR   ((reg16 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__16BIT_D1_REG)
            #define SERVO_PWM_COUNTERCAP_LSB     (*(reg16 *) SERVO_PWM_PWMUDB_sP16_pwmdp_u0__16BIT_A1_REG)
            #define SERVO_PWM_COUNTERCAP_LSB_PTR ((reg16 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__16BIT_A1_REG)
            #define SERVO_PWM_COUNTER_LSB        (*(reg16 *) SERVO_PWM_PWMUDB_sP16_pwmdp_u0__16BIT_A0_REG)
            #define SERVO_PWM_COUNTER_LSB_PTR    ((reg16 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__16BIT_A0_REG)
            #define SERVO_PWM_CAPTURE_LSB        (*(reg16 *) SERVO_PWM_PWMUDB_sP16_pwmdp_u0__16BIT_F1_REG)
            #define SERVO_PWM_CAPTURE_LSB_PTR    ((reg16 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__16BIT_F1_REG)
        #endif /* (CY_PSOC3) */

       #define SERVO_PWM_AUX_CONTROLDP1          (*(reg8 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u1__DP_AUX_CTL_REG)
       #define SERVO_PWM_AUX_CONTROLDP1_PTR      ((reg8 *)   SERVO_PWM_PWMUDB_sP16_pwmdp_u1__DP_AUX_CTL_REG)

   #endif /* (SERVO_PWM_Resolution == 8) */

   #define SERVO_PWM_COUNTERCAP_LSB_PTR_8BIT ( (reg8 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__A1_REG)
   #define SERVO_PWM_AUX_CONTROLDP0          (*(reg8 *)  SERVO_PWM_PWMUDB_sP16_pwmdp_u0__DP_AUX_CTL_REG)
   #define SERVO_PWM_AUX_CONTROLDP0_PTR      ((reg8 *)   SERVO_PWM_PWMUDB_sP16_pwmdp_u0__DP_AUX_CTL_REG)

#endif /* (SERVO_PWM_UsingFixedFunction) */

#if(SERVO_PWM_KillModeMinTime )
    #define SERVO_PWM_KILLMODEMINTIME        (*(reg8 *)  SERVO_PWM_PWMUDB_sKM_killmodecounterdp_u0__D0_REG)
    #define SERVO_PWM_KILLMODEMINTIME_PTR    ((reg8 *)   SERVO_PWM_PWMUDB_sKM_killmodecounterdp_u0__D0_REG)
    /* Fixed Function Block has no Kill Mode parameters because it is Asynchronous only */
#endif /* (SERVO_PWM_KillModeMinTime ) */

#if(SERVO_PWM_DeadBandMode == SERVO_PWM__B_PWM__DBM_256_CLOCKS)
    #define SERVO_PWM_DEADBAND_COUNT         (*(reg8 *)  SERVO_PWM_PWMUDB_sDB255_deadbandcounterdp_u0__D0_REG)
    #define SERVO_PWM_DEADBAND_COUNT_PTR     ((reg8 *)   SERVO_PWM_PWMUDB_sDB255_deadbandcounterdp_u0__D0_REG)
    #define SERVO_PWM_DEADBAND_LSB_PTR       ((reg8 *)   SERVO_PWM_PWMUDB_sDB255_deadbandcounterdp_u0__A0_REG)
    #define SERVO_PWM_DEADBAND_LSB           (*(reg8 *)  SERVO_PWM_PWMUDB_sDB255_deadbandcounterdp_u0__A0_REG)
#elif(SERVO_PWM_DeadBandMode == SERVO_PWM__B_PWM__DBM_2_4_CLOCKS)
    
    /* In Fixed Function Block these bits are in the control blocks control register */
    #if (SERVO_PWM_UsingFixedFunction)
        #define SERVO_PWM_DEADBAND_COUNT         (*(reg8 *)  SERVO_PWM_PWMHW__CFG0)
        #define SERVO_PWM_DEADBAND_COUNT_PTR     ((reg8 *)   SERVO_PWM_PWMHW__CFG0)
        #define SERVO_PWM_DEADBAND_COUNT_MASK    (uint8)((uint8)0x03u << SERVO_PWM_DEADBAND_COUNT_SHIFT)

        /* As defined by the Register Map as DEADBAND_PERIOD[1:0] in CFG0 */
        #define SERVO_PWM_DEADBAND_COUNT_SHIFT   (0x06u)
    #else
        /* Lower two bits of the added control register define the count 1-3 */
        #define SERVO_PWM_DEADBAND_COUNT         (*(reg8 *)  SERVO_PWM_PWMUDB_genblk7_dbctrlreg__CONTROL_REG)
        #define SERVO_PWM_DEADBAND_COUNT_PTR     ((reg8 *)   SERVO_PWM_PWMUDB_genblk7_dbctrlreg__CONTROL_REG)
        #define SERVO_PWM_DEADBAND_COUNT_MASK    (uint8)((uint8)0x03u << SERVO_PWM_DEADBAND_COUNT_SHIFT)

        /* As defined by the verilog implementation of the Control Register */
        #define SERVO_PWM_DEADBAND_COUNT_SHIFT   (0x00u)
    #endif /* (SERVO_PWM_UsingFixedFunction) */
#endif /* (SERVO_PWM_DeadBandMode == SERVO_PWM__B_PWM__DBM_256_CLOCKS) */



#if (SERVO_PWM_UsingFixedFunction)
    #define SERVO_PWM_STATUS                 (*(reg8 *) SERVO_PWM_PWMHW__SR0)
    #define SERVO_PWM_STATUS_PTR             ((reg8 *) SERVO_PWM_PWMHW__SR0)
    #define SERVO_PWM_STATUS_MASK            (*(reg8 *) SERVO_PWM_PWMHW__SR0)
    #define SERVO_PWM_STATUS_MASK_PTR        ((reg8 *) SERVO_PWM_PWMHW__SR0)
    #define SERVO_PWM_CONTROL                (*(reg8 *) SERVO_PWM_PWMHW__CFG0)
    #define SERVO_PWM_CONTROL_PTR            ((reg8 *) SERVO_PWM_PWMHW__CFG0)
    #define SERVO_PWM_CONTROL2               (*(reg8 *) SERVO_PWM_PWMHW__CFG1)
    #define SERVO_PWM_CONTROL3               (*(reg8 *) SERVO_PWM_PWMHW__CFG2)
    #define SERVO_PWM_GLOBAL_ENABLE          (*(reg8 *) SERVO_PWM_PWMHW__PM_ACT_CFG)
    #define SERVO_PWM_GLOBAL_ENABLE_PTR      ( (reg8 *) SERVO_PWM_PWMHW__PM_ACT_CFG)
    #define SERVO_PWM_GLOBAL_STBY_ENABLE     (*(reg8 *) SERVO_PWM_PWMHW__PM_STBY_CFG)
    #define SERVO_PWM_GLOBAL_STBY_ENABLE_PTR ( (reg8 *) SERVO_PWM_PWMHW__PM_STBY_CFG)


    /***********************************
    *          Constants
    ***********************************/

    /* Fixed Function Block Chosen */
    #define SERVO_PWM_BLOCK_EN_MASK          (SERVO_PWM_PWMHW__PM_ACT_MSK)
    #define SERVO_PWM_BLOCK_STBY_EN_MASK     (SERVO_PWM_PWMHW__PM_STBY_MSK)
    
    /* Control Register definitions */
    #define SERVO_PWM_CTRL_ENABLE_SHIFT      (0x00u)

    /* As defined by Register map as MODE_CFG bits in CFG2*/
    #define SERVO_PWM_CTRL_CMPMODE1_SHIFT    (0x04u)

    /* As defined by Register map */
    #define SERVO_PWM_CTRL_DEAD_TIME_SHIFT   (0x06u)  

    /* Fixed Function Block Only CFG register bit definitions */
    /*  Set to compare mode */
    #define SERVO_PWM_CFG0_MODE              (0x02u)   

    /* Enable the block to run */
    #define SERVO_PWM_CFG0_ENABLE            (0x01u)   
    
    /* As defined by Register map as DB bit in CFG0 */
    #define SERVO_PWM_CFG0_DB                (0x20u)   

    /* Control Register Bit Masks */
    #define SERVO_PWM_CTRL_ENABLE            (uint8)((uint8)0x01u << SERVO_PWM_CTRL_ENABLE_SHIFT)
    #define SERVO_PWM_CTRL_RESET             (uint8)((uint8)0x01u << SERVO_PWM_CTRL_RESET_SHIFT)
    #define SERVO_PWM_CTRL_CMPMODE2_MASK     (uint8)((uint8)0x07u << SERVO_PWM_CTRL_CMPMODE2_SHIFT)
    #define SERVO_PWM_CTRL_CMPMODE1_MASK     (uint8)((uint8)0x07u << SERVO_PWM_CTRL_CMPMODE1_SHIFT)

    /* Control2 Register Bit Masks */
    /* As defined in Register Map, Part of the TMRX_CFG1 register */
    #define SERVO_PWM_CTRL2_IRQ_SEL_SHIFT    (0x00u)
    #define SERVO_PWM_CTRL2_IRQ_SEL          (uint8)((uint8)0x01u << SERVO_PWM_CTRL2_IRQ_SEL_SHIFT)

    /* Status Register Bit Locations */
    /* As defined by Register map as TC in SR0 */
    #define SERVO_PWM_STATUS_TC_SHIFT        (0x07u)   
    
    /* As defined by the Register map as CAP_CMP in SR0 */
    #define SERVO_PWM_STATUS_CMP1_SHIFT      (0x06u)   

    /* Status Register Interrupt Enable Bit Locations */
    #define SERVO_PWM_STATUS_KILL_INT_EN_MASK_SHIFT          (0x00u)
    #define SERVO_PWM_STATUS_TC_INT_EN_MASK_SHIFT            (SERVO_PWM_STATUS_TC_SHIFT - 4u)
    #define SERVO_PWM_STATUS_CMP2_INT_EN_MASK_SHIFT          (0x00u)
    #define SERVO_PWM_STATUS_CMP1_INT_EN_MASK_SHIFT          (SERVO_PWM_STATUS_CMP1_SHIFT - 4u)

    /* Status Register Bit Masks */
    #define SERVO_PWM_STATUS_TC              (uint8)((uint8)0x01u << SERVO_PWM_STATUS_TC_SHIFT)
    #define SERVO_PWM_STATUS_CMP1            (uint8)((uint8)0x01u << SERVO_PWM_STATUS_CMP1_SHIFT)

    /* Status Register Interrupt Bit Masks */
    #define SERVO_PWM_STATUS_TC_INT_EN_MASK              (uint8)((uint8)SERVO_PWM_STATUS_TC >> 4u)
    #define SERVO_PWM_STATUS_CMP1_INT_EN_MASK            (uint8)((uint8)SERVO_PWM_STATUS_CMP1 >> 4u)

    /*RT1 Synch Constants */
    #define SERVO_PWM_RT1_SHIFT             (0x04u)

    /* Sync TC and CMP bit masks */
    #define SERVO_PWM_RT1_MASK              (uint8)((uint8)0x03u << SERVO_PWM_RT1_SHIFT)
    #define SERVO_PWM_SYNC                  (uint8)((uint8)0x03u << SERVO_PWM_RT1_SHIFT)
    #define SERVO_PWM_SYNCDSI_SHIFT         (0x00u)

    /* Sync all DSI inputs */
    #define SERVO_PWM_SYNCDSI_MASK          (uint8)((uint8)0x0Fu << SERVO_PWM_SYNCDSI_SHIFT)

    /* Sync all DSI inputs */
    #define SERVO_PWM_SYNCDSI_EN            (uint8)((uint8)0x0Fu << SERVO_PWM_SYNCDSI_SHIFT)


#else
    #define SERVO_PWM_STATUS                (*(reg8 *)   SERVO_PWM_PWMUDB_genblk8_stsreg__STATUS_REG )
    #define SERVO_PWM_STATUS_PTR            ((reg8 *)    SERVO_PWM_PWMUDB_genblk8_stsreg__STATUS_REG )
    #define SERVO_PWM_STATUS_MASK           (*(reg8 *)   SERVO_PWM_PWMUDB_genblk8_stsreg__MASK_REG)
    #define SERVO_PWM_STATUS_MASK_PTR       ((reg8 *)    SERVO_PWM_PWMUDB_genblk8_stsreg__MASK_REG)
    #define SERVO_PWM_STATUS_AUX_CTRL       (*(reg8 *)   SERVO_PWM_PWMUDB_genblk8_stsreg__STATUS_AUX_CTL_REG)
    #define SERVO_PWM_CONTROL               (*(reg8 *)   SERVO_PWM_PWMUDB_genblk1_ctrlreg__CONTROL_REG)
    #define SERVO_PWM_CONTROL_PTR           ((reg8 *)    SERVO_PWM_PWMUDB_genblk1_ctrlreg__CONTROL_REG)


    /***********************************
    *          Constants
    ***********************************/

    /* Control Register bit definitions */
    #define SERVO_PWM_CTRL_ENABLE_SHIFT      (0x07u)
    #define SERVO_PWM_CTRL_RESET_SHIFT       (0x06u)
    #define SERVO_PWM_CTRL_CMPMODE2_SHIFT    (0x03u)
    #define SERVO_PWM_CTRL_CMPMODE1_SHIFT    (0x00u)
    #define SERVO_PWM_CTRL_DEAD_TIME_SHIFT   (0x00u)   /* No Shift Needed for UDB block */
    
    /* Control Register Bit Masks */
    #define SERVO_PWM_CTRL_ENABLE            (uint8)((uint8)0x01u << SERVO_PWM_CTRL_ENABLE_SHIFT)
    #define SERVO_PWM_CTRL_RESET             (uint8)((uint8)0x01u << SERVO_PWM_CTRL_RESET_SHIFT)
    #define SERVO_PWM_CTRL_CMPMODE2_MASK     (uint8)((uint8)0x07u << SERVO_PWM_CTRL_CMPMODE2_SHIFT)
    #define SERVO_PWM_CTRL_CMPMODE1_MASK     (uint8)((uint8)0x07u << SERVO_PWM_CTRL_CMPMODE1_SHIFT)

    /* Status Register Bit Locations */
    #define SERVO_PWM_STATUS_KILL_SHIFT          (0x05u)
    #define SERVO_PWM_STATUS_FIFONEMPTY_SHIFT    (0x04u)
    #define SERVO_PWM_STATUS_FIFOFULL_SHIFT      (0x03u)
    #define SERVO_PWM_STATUS_TC_SHIFT            (0x02u)
    #define SERVO_PWM_STATUS_CMP2_SHIFT          (0x01u)
    #define SERVO_PWM_STATUS_CMP1_SHIFT          (0x00u)

    /* Status Register Interrupt Enable Bit Locations - UDB Status Interrupt Mask match Status Bit Locations*/
    #define SERVO_PWM_STATUS_KILL_INT_EN_MASK_SHIFT          (SERVO_PWM_STATUS_KILL_SHIFT)
    #define SERVO_PWM_STATUS_FIFONEMPTY_INT_EN_MASK_SHIFT    (SERVO_PWM_STATUS_FIFONEMPTY_SHIFT)
    #define SERVO_PWM_STATUS_FIFOFULL_INT_EN_MASK_SHIFT      (SERVO_PWM_STATUS_FIFOFULL_SHIFT)
    #define SERVO_PWM_STATUS_TC_INT_EN_MASK_SHIFT            (SERVO_PWM_STATUS_TC_SHIFT)
    #define SERVO_PWM_STATUS_CMP2_INT_EN_MASK_SHIFT          (SERVO_PWM_STATUS_CMP2_SHIFT)
    #define SERVO_PWM_STATUS_CMP1_INT_EN_MASK_SHIFT          (SERVO_PWM_STATUS_CMP1_SHIFT)

    /* Status Register Bit Masks */
    #define SERVO_PWM_STATUS_KILL            (uint8)((uint8)0x00u << SERVO_PWM_STATUS_KILL_SHIFT )
    #define SERVO_PWM_STATUS_FIFOFULL        (uint8)((uint8)0x01u << SERVO_PWM_STATUS_FIFOFULL_SHIFT)
    #define SERVO_PWM_STATUS_FIFONEMPTY      (uint8)((uint8)0x01u << SERVO_PWM_STATUS_FIFONEMPTY_SHIFT)
    #define SERVO_PWM_STATUS_TC              (uint8)((uint8)0x01u << SERVO_PWM_STATUS_TC_SHIFT)
    #define SERVO_PWM_STATUS_CMP2            (uint8)((uint8)0x01u << SERVO_PWM_STATUS_CMP2_SHIFT)
    #define SERVO_PWM_STATUS_CMP1            (uint8)((uint8)0x01u << SERVO_PWM_STATUS_CMP1_SHIFT)

    /* Status Register Interrupt Bit Masks  - UDB Status Interrupt Mask match Status Bit Locations */
    #define SERVO_PWM_STATUS_KILL_INT_EN_MASK            (SERVO_PWM_STATUS_KILL)
    #define SERVO_PWM_STATUS_FIFOFULL_INT_EN_MASK        (SERVO_PWM_STATUS_FIFOFULL)
    #define SERVO_PWM_STATUS_FIFONEMPTY_INT_EN_MASK      (SERVO_PWM_STATUS_FIFONEMPTY)
    #define SERVO_PWM_STATUS_TC_INT_EN_MASK              (SERVO_PWM_STATUS_TC)
    #define SERVO_PWM_STATUS_CMP2_INT_EN_MASK            (SERVO_PWM_STATUS_CMP2)
    #define SERVO_PWM_STATUS_CMP1_INT_EN_MASK            (SERVO_PWM_STATUS_CMP1)

    /* Datapath Auxillary Control Register bit definitions */
    #define SERVO_PWM_AUX_CTRL_FIFO0_CLR         (0x01u)
    #define SERVO_PWM_AUX_CTRL_FIFO1_CLR         (0x02u)
    #define SERVO_PWM_AUX_CTRL_FIFO0_LVL         (0x04u)
    #define SERVO_PWM_AUX_CTRL_FIFO1_LVL         (0x08u)
    #define SERVO_PWM_STATUS_ACTL_INT_EN_MASK    (0x10u) /* As defined for the ACTL Register */
#endif /* SERVO_PWM_UsingFixedFunction */

#endif  /* CY_PWM_SERVO_PWM_H */


/* [] END OF FILE */
