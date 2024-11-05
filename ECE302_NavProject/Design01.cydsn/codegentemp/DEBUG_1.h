/*******************************************************************************
* File Name: DEBUG_1.h  
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

#if !defined(CY_PINS_DEBUG_1_H) /* Pins DEBUG_1_H */
#define CY_PINS_DEBUG_1_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "DEBUG_1_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 DEBUG_1__PORT == 15 && ((DEBUG_1__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    DEBUG_1_Write(uint8 value);
void    DEBUG_1_SetDriveMode(uint8 mode);
uint8   DEBUG_1_ReadDataReg(void);
uint8   DEBUG_1_Read(void);
void    DEBUG_1_SetInterruptMode(uint16 position, uint16 mode);
uint8   DEBUG_1_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the DEBUG_1_SetDriveMode() function.
     *  @{
     */
        #define DEBUG_1_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define DEBUG_1_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define DEBUG_1_DM_RES_UP          PIN_DM_RES_UP
        #define DEBUG_1_DM_RES_DWN         PIN_DM_RES_DWN
        #define DEBUG_1_DM_OD_LO           PIN_DM_OD_LO
        #define DEBUG_1_DM_OD_HI           PIN_DM_OD_HI
        #define DEBUG_1_DM_STRONG          PIN_DM_STRONG
        #define DEBUG_1_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define DEBUG_1_MASK               DEBUG_1__MASK
#define DEBUG_1_SHIFT              DEBUG_1__SHIFT
#define DEBUG_1_WIDTH              1u

/* Interrupt constants */
#if defined(DEBUG_1__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in DEBUG_1_SetInterruptMode() function.
     *  @{
     */
        #define DEBUG_1_INTR_NONE      (uint16)(0x0000u)
        #define DEBUG_1_INTR_RISING    (uint16)(0x0001u)
        #define DEBUG_1_INTR_FALLING   (uint16)(0x0002u)
        #define DEBUG_1_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define DEBUG_1_INTR_MASK      (0x01u) 
#endif /* (DEBUG_1__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define DEBUG_1_PS                     (* (reg8 *) DEBUG_1__PS)
/* Data Register */
#define DEBUG_1_DR                     (* (reg8 *) DEBUG_1__DR)
/* Port Number */
#define DEBUG_1_PRT_NUM                (* (reg8 *) DEBUG_1__PRT) 
/* Connect to Analog Globals */                                                  
#define DEBUG_1_AG                     (* (reg8 *) DEBUG_1__AG)                       
/* Analog MUX bux enable */
#define DEBUG_1_AMUX                   (* (reg8 *) DEBUG_1__AMUX) 
/* Bidirectional Enable */                                                        
#define DEBUG_1_BIE                    (* (reg8 *) DEBUG_1__BIE)
/* Bit-mask for Aliased Register Access */
#define DEBUG_1_BIT_MASK               (* (reg8 *) DEBUG_1__BIT_MASK)
/* Bypass Enable */
#define DEBUG_1_BYP                    (* (reg8 *) DEBUG_1__BYP)
/* Port wide control signals */                                                   
#define DEBUG_1_CTL                    (* (reg8 *) DEBUG_1__CTL)
/* Drive Modes */
#define DEBUG_1_DM0                    (* (reg8 *) DEBUG_1__DM0) 
#define DEBUG_1_DM1                    (* (reg8 *) DEBUG_1__DM1)
#define DEBUG_1_DM2                    (* (reg8 *) DEBUG_1__DM2) 
/* Input Buffer Disable Override */
#define DEBUG_1_INP_DIS                (* (reg8 *) DEBUG_1__INP_DIS)
/* LCD Common or Segment Drive */
#define DEBUG_1_LCD_COM_SEG            (* (reg8 *) DEBUG_1__LCD_COM_SEG)
/* Enable Segment LCD */
#define DEBUG_1_LCD_EN                 (* (reg8 *) DEBUG_1__LCD_EN)
/* Slew Rate Control */
#define DEBUG_1_SLW                    (* (reg8 *) DEBUG_1__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define DEBUG_1_PRTDSI__CAPS_SEL       (* (reg8 *) DEBUG_1__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define DEBUG_1_PRTDSI__DBL_SYNC_IN    (* (reg8 *) DEBUG_1__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define DEBUG_1_PRTDSI__OE_SEL0        (* (reg8 *) DEBUG_1__PRTDSI__OE_SEL0) 
#define DEBUG_1_PRTDSI__OE_SEL1        (* (reg8 *) DEBUG_1__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define DEBUG_1_PRTDSI__OUT_SEL0       (* (reg8 *) DEBUG_1__PRTDSI__OUT_SEL0) 
#define DEBUG_1_PRTDSI__OUT_SEL1       (* (reg8 *) DEBUG_1__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define DEBUG_1_PRTDSI__SYNC_OUT       (* (reg8 *) DEBUG_1__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(DEBUG_1__SIO_CFG)
    #define DEBUG_1_SIO_HYST_EN        (* (reg8 *) DEBUG_1__SIO_HYST_EN)
    #define DEBUG_1_SIO_REG_HIFREQ     (* (reg8 *) DEBUG_1__SIO_REG_HIFREQ)
    #define DEBUG_1_SIO_CFG            (* (reg8 *) DEBUG_1__SIO_CFG)
    #define DEBUG_1_SIO_DIFF           (* (reg8 *) DEBUG_1__SIO_DIFF)
#endif /* (DEBUG_1__SIO_CFG) */

/* Interrupt Registers */
#if defined(DEBUG_1__INTSTAT)
    #define DEBUG_1_INTSTAT            (* (reg8 *) DEBUG_1__INTSTAT)
    #define DEBUG_1_SNAP               (* (reg8 *) DEBUG_1__SNAP)
    
	#define DEBUG_1_0_INTTYPE_REG 		(* (reg8 *) DEBUG_1__0__INTTYPE)
#endif /* (DEBUG_1__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_DEBUG_1_H */


/* [] END OF FILE */
