/*******************************************************************************
* File Name: motor.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_motor_H) /* Pins motor_H */
#define CY_PINS_motor_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "motor_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 motor__PORT == 15 && ((motor__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    motor_Write(uint8 value);
void    motor_SetDriveMode(uint8 mode);
uint8   motor_ReadDataReg(void);
uint8   motor_Read(void);
void    motor_SetInterruptMode(uint16 position, uint16 mode);
uint8   motor_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the motor_SetDriveMode() function.
     *  @{
     */
        #define motor_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define motor_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define motor_DM_RES_UP          PIN_DM_RES_UP
        #define motor_DM_RES_DWN         PIN_DM_RES_DWN
        #define motor_DM_OD_LO           PIN_DM_OD_LO
        #define motor_DM_OD_HI           PIN_DM_OD_HI
        #define motor_DM_STRONG          PIN_DM_STRONG
        #define motor_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define motor_MASK               motor__MASK
#define motor_SHIFT              motor__SHIFT
#define motor_WIDTH              1u

/* Interrupt constants */
#if defined(motor__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in motor_SetInterruptMode() function.
     *  @{
     */
        #define motor_INTR_NONE      (uint16)(0x0000u)
        #define motor_INTR_RISING    (uint16)(0x0001u)
        #define motor_INTR_FALLING   (uint16)(0x0002u)
        #define motor_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define motor_INTR_MASK      (0x01u) 
#endif /* (motor__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define motor_PS                     (* (reg8 *) motor__PS)
/* Data Register */
#define motor_DR                     (* (reg8 *) motor__DR)
/* Port Number */
#define motor_PRT_NUM                (* (reg8 *) motor__PRT) 
/* Connect to Analog Globals */                                                  
#define motor_AG                     (* (reg8 *) motor__AG)                       
/* Analog MUX bux enable */
#define motor_AMUX                   (* (reg8 *) motor__AMUX) 
/* Bidirectional Enable */                                                        
#define motor_BIE                    (* (reg8 *) motor__BIE)
/* Bit-mask for Aliased Register Access */
#define motor_BIT_MASK               (* (reg8 *) motor__BIT_MASK)
/* Bypass Enable */
#define motor_BYP                    (* (reg8 *) motor__BYP)
/* Port wide control signals */                                                   
#define motor_CTL                    (* (reg8 *) motor__CTL)
/* Drive Modes */
#define motor_DM0                    (* (reg8 *) motor__DM0) 
#define motor_DM1                    (* (reg8 *) motor__DM1)
#define motor_DM2                    (* (reg8 *) motor__DM2) 
/* Input Buffer Disable Override */
#define motor_INP_DIS                (* (reg8 *) motor__INP_DIS)
/* LCD Common or Segment Drive */
#define motor_LCD_COM_SEG            (* (reg8 *) motor__LCD_COM_SEG)
/* Enable Segment LCD */
#define motor_LCD_EN                 (* (reg8 *) motor__LCD_EN)
/* Slew Rate Control */
#define motor_SLW                    (* (reg8 *) motor__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define motor_PRTDSI__CAPS_SEL       (* (reg8 *) motor__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define motor_PRTDSI__DBL_SYNC_IN    (* (reg8 *) motor__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define motor_PRTDSI__OE_SEL0        (* (reg8 *) motor__PRTDSI__OE_SEL0) 
#define motor_PRTDSI__OE_SEL1        (* (reg8 *) motor__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define motor_PRTDSI__OUT_SEL0       (* (reg8 *) motor__PRTDSI__OUT_SEL0) 
#define motor_PRTDSI__OUT_SEL1       (* (reg8 *) motor__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define motor_PRTDSI__SYNC_OUT       (* (reg8 *) motor__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(motor__SIO_CFG)
    #define motor_SIO_HYST_EN        (* (reg8 *) motor__SIO_HYST_EN)
    #define motor_SIO_REG_HIFREQ     (* (reg8 *) motor__SIO_REG_HIFREQ)
    #define motor_SIO_CFG            (* (reg8 *) motor__SIO_CFG)
    #define motor_SIO_DIFF           (* (reg8 *) motor__SIO_DIFF)
#endif /* (motor__SIO_CFG) */

/* Interrupt Registers */
#if defined(motor__INTSTAT)
    #define motor_INTSTAT            (* (reg8 *) motor__INTSTAT)
    #define motor_SNAP               (* (reg8 *) motor__SNAP)
    
	#define motor_0_INTTYPE_REG 		(* (reg8 *) motor__0__INTTYPE)
#endif /* (motor__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_motor_H */


/* [] END OF FILE */
