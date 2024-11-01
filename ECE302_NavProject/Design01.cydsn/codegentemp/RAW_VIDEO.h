/*******************************************************************************
* File Name: RAW_VIDEO.h  
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

#if !defined(CY_PINS_RAW_VIDEO_H) /* Pins RAW_VIDEO_H */
#define CY_PINS_RAW_VIDEO_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "RAW_VIDEO_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 RAW_VIDEO__PORT == 15 && ((RAW_VIDEO__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    RAW_VIDEO_Write(uint8 value);
void    RAW_VIDEO_SetDriveMode(uint8 mode);
uint8   RAW_VIDEO_ReadDataReg(void);
uint8   RAW_VIDEO_Read(void);
void    RAW_VIDEO_SetInterruptMode(uint16 position, uint16 mode);
uint8   RAW_VIDEO_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the RAW_VIDEO_SetDriveMode() function.
     *  @{
     */
        #define RAW_VIDEO_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define RAW_VIDEO_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define RAW_VIDEO_DM_RES_UP          PIN_DM_RES_UP
        #define RAW_VIDEO_DM_RES_DWN         PIN_DM_RES_DWN
        #define RAW_VIDEO_DM_OD_LO           PIN_DM_OD_LO
        #define RAW_VIDEO_DM_OD_HI           PIN_DM_OD_HI
        #define RAW_VIDEO_DM_STRONG          PIN_DM_STRONG
        #define RAW_VIDEO_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define RAW_VIDEO_MASK               RAW_VIDEO__MASK
#define RAW_VIDEO_SHIFT              RAW_VIDEO__SHIFT
#define RAW_VIDEO_WIDTH              1u

/* Interrupt constants */
#if defined(RAW_VIDEO__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in RAW_VIDEO_SetInterruptMode() function.
     *  @{
     */
        #define RAW_VIDEO_INTR_NONE      (uint16)(0x0000u)
        #define RAW_VIDEO_INTR_RISING    (uint16)(0x0001u)
        #define RAW_VIDEO_INTR_FALLING   (uint16)(0x0002u)
        #define RAW_VIDEO_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define RAW_VIDEO_INTR_MASK      (0x01u) 
#endif /* (RAW_VIDEO__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define RAW_VIDEO_PS                     (* (reg8 *) RAW_VIDEO__PS)
/* Data Register */
#define RAW_VIDEO_DR                     (* (reg8 *) RAW_VIDEO__DR)
/* Port Number */
#define RAW_VIDEO_PRT_NUM                (* (reg8 *) RAW_VIDEO__PRT) 
/* Connect to Analog Globals */                                                  
#define RAW_VIDEO_AG                     (* (reg8 *) RAW_VIDEO__AG)                       
/* Analog MUX bux enable */
#define RAW_VIDEO_AMUX                   (* (reg8 *) RAW_VIDEO__AMUX) 
/* Bidirectional Enable */                                                        
#define RAW_VIDEO_BIE                    (* (reg8 *) RAW_VIDEO__BIE)
/* Bit-mask for Aliased Register Access */
#define RAW_VIDEO_BIT_MASK               (* (reg8 *) RAW_VIDEO__BIT_MASK)
/* Bypass Enable */
#define RAW_VIDEO_BYP                    (* (reg8 *) RAW_VIDEO__BYP)
/* Port wide control signals */                                                   
#define RAW_VIDEO_CTL                    (* (reg8 *) RAW_VIDEO__CTL)
/* Drive Modes */
#define RAW_VIDEO_DM0                    (* (reg8 *) RAW_VIDEO__DM0) 
#define RAW_VIDEO_DM1                    (* (reg8 *) RAW_VIDEO__DM1)
#define RAW_VIDEO_DM2                    (* (reg8 *) RAW_VIDEO__DM2) 
/* Input Buffer Disable Override */
#define RAW_VIDEO_INP_DIS                (* (reg8 *) RAW_VIDEO__INP_DIS)
/* LCD Common or Segment Drive */
#define RAW_VIDEO_LCD_COM_SEG            (* (reg8 *) RAW_VIDEO__LCD_COM_SEG)
/* Enable Segment LCD */
#define RAW_VIDEO_LCD_EN                 (* (reg8 *) RAW_VIDEO__LCD_EN)
/* Slew Rate Control */
#define RAW_VIDEO_SLW                    (* (reg8 *) RAW_VIDEO__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define RAW_VIDEO_PRTDSI__CAPS_SEL       (* (reg8 *) RAW_VIDEO__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define RAW_VIDEO_PRTDSI__DBL_SYNC_IN    (* (reg8 *) RAW_VIDEO__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define RAW_VIDEO_PRTDSI__OE_SEL0        (* (reg8 *) RAW_VIDEO__PRTDSI__OE_SEL0) 
#define RAW_VIDEO_PRTDSI__OE_SEL1        (* (reg8 *) RAW_VIDEO__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define RAW_VIDEO_PRTDSI__OUT_SEL0       (* (reg8 *) RAW_VIDEO__PRTDSI__OUT_SEL0) 
#define RAW_VIDEO_PRTDSI__OUT_SEL1       (* (reg8 *) RAW_VIDEO__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define RAW_VIDEO_PRTDSI__SYNC_OUT       (* (reg8 *) RAW_VIDEO__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(RAW_VIDEO__SIO_CFG)
    #define RAW_VIDEO_SIO_HYST_EN        (* (reg8 *) RAW_VIDEO__SIO_HYST_EN)
    #define RAW_VIDEO_SIO_REG_HIFREQ     (* (reg8 *) RAW_VIDEO__SIO_REG_HIFREQ)
    #define RAW_VIDEO_SIO_CFG            (* (reg8 *) RAW_VIDEO__SIO_CFG)
    #define RAW_VIDEO_SIO_DIFF           (* (reg8 *) RAW_VIDEO__SIO_DIFF)
#endif /* (RAW_VIDEO__SIO_CFG) */

/* Interrupt Registers */
#if defined(RAW_VIDEO__INTSTAT)
    #define RAW_VIDEO_INTSTAT            (* (reg8 *) RAW_VIDEO__INTSTAT)
    #define RAW_VIDEO_SNAP               (* (reg8 *) RAW_VIDEO__SNAP)
    
	#define RAW_VIDEO_0_INTTYPE_REG 		(* (reg8 *) RAW_VIDEO__0__INTTYPE)
#endif /* (RAW_VIDEO__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_RAW_VIDEO_H */


/* [] END OF FILE */
