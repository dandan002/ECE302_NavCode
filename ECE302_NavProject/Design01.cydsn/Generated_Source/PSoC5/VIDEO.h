/*******************************************************************************
* File Name: VIDEO.h  
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

#if !defined(CY_PINS_VIDEO_H) /* Pins VIDEO_H */
#define CY_PINS_VIDEO_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "VIDEO_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 VIDEO__PORT == 15 && ((VIDEO__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    VIDEO_Write(uint8 value);
void    VIDEO_SetDriveMode(uint8 mode);
uint8   VIDEO_ReadDataReg(void);
uint8   VIDEO_Read(void);
void    VIDEO_SetInterruptMode(uint16 position, uint16 mode);
uint8   VIDEO_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the VIDEO_SetDriveMode() function.
     *  @{
     */
        #define VIDEO_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define VIDEO_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define VIDEO_DM_RES_UP          PIN_DM_RES_UP
        #define VIDEO_DM_RES_DWN         PIN_DM_RES_DWN
        #define VIDEO_DM_OD_LO           PIN_DM_OD_LO
        #define VIDEO_DM_OD_HI           PIN_DM_OD_HI
        #define VIDEO_DM_STRONG          PIN_DM_STRONG
        #define VIDEO_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define VIDEO_MASK               VIDEO__MASK
#define VIDEO_SHIFT              VIDEO__SHIFT
#define VIDEO_WIDTH              1u

/* Interrupt constants */
#if defined(VIDEO__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in VIDEO_SetInterruptMode() function.
     *  @{
     */
        #define VIDEO_INTR_NONE      (uint16)(0x0000u)
        #define VIDEO_INTR_RISING    (uint16)(0x0001u)
        #define VIDEO_INTR_FALLING   (uint16)(0x0002u)
        #define VIDEO_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define VIDEO_INTR_MASK      (0x01u) 
#endif /* (VIDEO__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define VIDEO_PS                     (* (reg8 *) VIDEO__PS)
/* Data Register */
#define VIDEO_DR                     (* (reg8 *) VIDEO__DR)
/* Port Number */
#define VIDEO_PRT_NUM                (* (reg8 *) VIDEO__PRT) 
/* Connect to Analog Globals */                                                  
#define VIDEO_AG                     (* (reg8 *) VIDEO__AG)                       
/* Analog MUX bux enable */
#define VIDEO_AMUX                   (* (reg8 *) VIDEO__AMUX) 
/* Bidirectional Enable */                                                        
#define VIDEO_BIE                    (* (reg8 *) VIDEO__BIE)
/* Bit-mask for Aliased Register Access */
#define VIDEO_BIT_MASK               (* (reg8 *) VIDEO__BIT_MASK)
/* Bypass Enable */
#define VIDEO_BYP                    (* (reg8 *) VIDEO__BYP)
/* Port wide control signals */                                                   
#define VIDEO_CTL                    (* (reg8 *) VIDEO__CTL)
/* Drive Modes */
#define VIDEO_DM0                    (* (reg8 *) VIDEO__DM0) 
#define VIDEO_DM1                    (* (reg8 *) VIDEO__DM1)
#define VIDEO_DM2                    (* (reg8 *) VIDEO__DM2) 
/* Input Buffer Disable Override */
#define VIDEO_INP_DIS                (* (reg8 *) VIDEO__INP_DIS)
/* LCD Common or Segment Drive */
#define VIDEO_LCD_COM_SEG            (* (reg8 *) VIDEO__LCD_COM_SEG)
/* Enable Segment LCD */
#define VIDEO_LCD_EN                 (* (reg8 *) VIDEO__LCD_EN)
/* Slew Rate Control */
#define VIDEO_SLW                    (* (reg8 *) VIDEO__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define VIDEO_PRTDSI__CAPS_SEL       (* (reg8 *) VIDEO__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define VIDEO_PRTDSI__DBL_SYNC_IN    (* (reg8 *) VIDEO__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define VIDEO_PRTDSI__OE_SEL0        (* (reg8 *) VIDEO__PRTDSI__OE_SEL0) 
#define VIDEO_PRTDSI__OE_SEL1        (* (reg8 *) VIDEO__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define VIDEO_PRTDSI__OUT_SEL0       (* (reg8 *) VIDEO__PRTDSI__OUT_SEL0) 
#define VIDEO_PRTDSI__OUT_SEL1       (* (reg8 *) VIDEO__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define VIDEO_PRTDSI__SYNC_OUT       (* (reg8 *) VIDEO__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(VIDEO__SIO_CFG)
    #define VIDEO_SIO_HYST_EN        (* (reg8 *) VIDEO__SIO_HYST_EN)
    #define VIDEO_SIO_REG_HIFREQ     (* (reg8 *) VIDEO__SIO_REG_HIFREQ)
    #define VIDEO_SIO_CFG            (* (reg8 *) VIDEO__SIO_CFG)
    #define VIDEO_SIO_DIFF           (* (reg8 *) VIDEO__SIO_DIFF)
#endif /* (VIDEO__SIO_CFG) */

/* Interrupt Registers */
#if defined(VIDEO__INTSTAT)
    #define VIDEO_INTSTAT            (* (reg8 *) VIDEO__INTSTAT)
    #define VIDEO_SNAP               (* (reg8 *) VIDEO__SNAP)
    
	#define VIDEO_0_INTTYPE_REG 		(* (reg8 *) VIDEO__0__INTTYPE)
#endif /* (VIDEO__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_VIDEO_H */


/* [] END OF FILE */
