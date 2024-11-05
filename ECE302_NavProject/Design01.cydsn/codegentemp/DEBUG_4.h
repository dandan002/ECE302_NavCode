/*******************************************************************************
* File Name: DEBUG_4.h  
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

#if !defined(CY_PINS_DEBUG_4_H) /* Pins DEBUG_4_H */
#define CY_PINS_DEBUG_4_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "DEBUG_4_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 DEBUG_4__PORT == 15 && ((DEBUG_4__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    DEBUG_4_Write(uint8 value);
void    DEBUG_4_SetDriveMode(uint8 mode);
uint8   DEBUG_4_ReadDataReg(void);
uint8   DEBUG_4_Read(void);
void    DEBUG_4_SetInterruptMode(uint16 position, uint16 mode);
uint8   DEBUG_4_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the DEBUG_4_SetDriveMode() function.
     *  @{
     */
        #define DEBUG_4_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define DEBUG_4_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define DEBUG_4_DM_RES_UP          PIN_DM_RES_UP
        #define DEBUG_4_DM_RES_DWN         PIN_DM_RES_DWN
        #define DEBUG_4_DM_OD_LO           PIN_DM_OD_LO
        #define DEBUG_4_DM_OD_HI           PIN_DM_OD_HI
        #define DEBUG_4_DM_STRONG          PIN_DM_STRONG
        #define DEBUG_4_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define DEBUG_4_MASK               DEBUG_4__MASK
#define DEBUG_4_SHIFT              DEBUG_4__SHIFT
#define DEBUG_4_WIDTH              1u

/* Interrupt constants */
#if defined(DEBUG_4__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in DEBUG_4_SetInterruptMode() function.
     *  @{
     */
        #define DEBUG_4_INTR_NONE      (uint16)(0x0000u)
        #define DEBUG_4_INTR_RISING    (uint16)(0x0001u)
        #define DEBUG_4_INTR_FALLING   (uint16)(0x0002u)
        #define DEBUG_4_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define DEBUG_4_INTR_MASK      (0x01u) 
#endif /* (DEBUG_4__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define DEBUG_4_PS                     (* (reg8 *) DEBUG_4__PS)
/* Data Register */
#define DEBUG_4_DR                     (* (reg8 *) DEBUG_4__DR)
/* Port Number */
#define DEBUG_4_PRT_NUM                (* (reg8 *) DEBUG_4__PRT) 
/* Connect to Analog Globals */                                                  
#define DEBUG_4_AG                     (* (reg8 *) DEBUG_4__AG)                       
/* Analog MUX bux enable */
#define DEBUG_4_AMUX                   (* (reg8 *) DEBUG_4__AMUX) 
/* Bidirectional Enable */                                                        
#define DEBUG_4_BIE                    (* (reg8 *) DEBUG_4__BIE)
/* Bit-mask for Aliased Register Access */
#define DEBUG_4_BIT_MASK               (* (reg8 *) DEBUG_4__BIT_MASK)
/* Bypass Enable */
#define DEBUG_4_BYP                    (* (reg8 *) DEBUG_4__BYP)
/* Port wide control signals */                                                   
#define DEBUG_4_CTL                    (* (reg8 *) DEBUG_4__CTL)
/* Drive Modes */
#define DEBUG_4_DM0                    (* (reg8 *) DEBUG_4__DM0) 
#define DEBUG_4_DM1                    (* (reg8 *) DEBUG_4__DM1)
#define DEBUG_4_DM2                    (* (reg8 *) DEBUG_4__DM2) 
/* Input Buffer Disable Override */
#define DEBUG_4_INP_DIS                (* (reg8 *) DEBUG_4__INP_DIS)
/* LCD Common or Segment Drive */
#define DEBUG_4_LCD_COM_SEG            (* (reg8 *) DEBUG_4__LCD_COM_SEG)
/* Enable Segment LCD */
#define DEBUG_4_LCD_EN                 (* (reg8 *) DEBUG_4__LCD_EN)
/* Slew Rate Control */
#define DEBUG_4_SLW                    (* (reg8 *) DEBUG_4__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define DEBUG_4_PRTDSI__CAPS_SEL       (* (reg8 *) DEBUG_4__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define DEBUG_4_PRTDSI__DBL_SYNC_IN    (* (reg8 *) DEBUG_4__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define DEBUG_4_PRTDSI__OE_SEL0        (* (reg8 *) DEBUG_4__PRTDSI__OE_SEL0) 
#define DEBUG_4_PRTDSI__OE_SEL1        (* (reg8 *) DEBUG_4__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define DEBUG_4_PRTDSI__OUT_SEL0       (* (reg8 *) DEBUG_4__PRTDSI__OUT_SEL0) 
#define DEBUG_4_PRTDSI__OUT_SEL1       (* (reg8 *) DEBUG_4__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define DEBUG_4_PRTDSI__SYNC_OUT       (* (reg8 *) DEBUG_4__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(DEBUG_4__SIO_CFG)
    #define DEBUG_4_SIO_HYST_EN        (* (reg8 *) DEBUG_4__SIO_HYST_EN)
    #define DEBUG_4_SIO_REG_HIFREQ     (* (reg8 *) DEBUG_4__SIO_REG_HIFREQ)
    #define DEBUG_4_SIO_CFG            (* (reg8 *) DEBUG_4__SIO_CFG)
    #define DEBUG_4_SIO_DIFF           (* (reg8 *) DEBUG_4__SIO_DIFF)
#endif /* (DEBUG_4__SIO_CFG) */

/* Interrupt Registers */
#if defined(DEBUG_4__INTSTAT)
    #define DEBUG_4_INTSTAT            (* (reg8 *) DEBUG_4__INTSTAT)
    #define DEBUG_4_SNAP               (* (reg8 *) DEBUG_4__SNAP)
    
	#define DEBUG_4_0_INTTYPE_REG 		(* (reg8 *) DEBUG_4__0__INTTYPE)
#endif /* (DEBUG_4__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_DEBUG_4_H */


/* [] END OF FILE */
