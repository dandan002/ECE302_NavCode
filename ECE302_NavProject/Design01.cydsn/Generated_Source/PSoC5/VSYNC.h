/*******************************************************************************
* File Name: VSYNC.h  
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

#if !defined(CY_PINS_VSYNC_H) /* Pins VSYNC_H */
#define CY_PINS_VSYNC_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "VSYNC_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 VSYNC__PORT == 15 && ((VSYNC__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    VSYNC_Write(uint8 value);
void    VSYNC_SetDriveMode(uint8 mode);
uint8   VSYNC_ReadDataReg(void);
uint8   VSYNC_Read(void);
void    VSYNC_SetInterruptMode(uint16 position, uint16 mode);
uint8   VSYNC_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the VSYNC_SetDriveMode() function.
     *  @{
     */
        #define VSYNC_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define VSYNC_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define VSYNC_DM_RES_UP          PIN_DM_RES_UP
        #define VSYNC_DM_RES_DWN         PIN_DM_RES_DWN
        #define VSYNC_DM_OD_LO           PIN_DM_OD_LO
        #define VSYNC_DM_OD_HI           PIN_DM_OD_HI
        #define VSYNC_DM_STRONG          PIN_DM_STRONG
        #define VSYNC_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define VSYNC_MASK               VSYNC__MASK
#define VSYNC_SHIFT              VSYNC__SHIFT
#define VSYNC_WIDTH              1u

/* Interrupt constants */
#if defined(VSYNC__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in VSYNC_SetInterruptMode() function.
     *  @{
     */
        #define VSYNC_INTR_NONE      (uint16)(0x0000u)
        #define VSYNC_INTR_RISING    (uint16)(0x0001u)
        #define VSYNC_INTR_FALLING   (uint16)(0x0002u)
        #define VSYNC_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define VSYNC_INTR_MASK      (0x01u) 
#endif /* (VSYNC__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define VSYNC_PS                     (* (reg8 *) VSYNC__PS)
/* Data Register */
#define VSYNC_DR                     (* (reg8 *) VSYNC__DR)
/* Port Number */
#define VSYNC_PRT_NUM                (* (reg8 *) VSYNC__PRT) 
/* Connect to Analog Globals */                                                  
#define VSYNC_AG                     (* (reg8 *) VSYNC__AG)                       
/* Analog MUX bux enable */
#define VSYNC_AMUX                   (* (reg8 *) VSYNC__AMUX) 
/* Bidirectional Enable */                                                        
#define VSYNC_BIE                    (* (reg8 *) VSYNC__BIE)
/* Bit-mask for Aliased Register Access */
#define VSYNC_BIT_MASK               (* (reg8 *) VSYNC__BIT_MASK)
/* Bypass Enable */
#define VSYNC_BYP                    (* (reg8 *) VSYNC__BYP)
/* Port wide control signals */                                                   
#define VSYNC_CTL                    (* (reg8 *) VSYNC__CTL)
/* Drive Modes */
#define VSYNC_DM0                    (* (reg8 *) VSYNC__DM0) 
#define VSYNC_DM1                    (* (reg8 *) VSYNC__DM1)
#define VSYNC_DM2                    (* (reg8 *) VSYNC__DM2) 
/* Input Buffer Disable Override */
#define VSYNC_INP_DIS                (* (reg8 *) VSYNC__INP_DIS)
/* LCD Common or Segment Drive */
#define VSYNC_LCD_COM_SEG            (* (reg8 *) VSYNC__LCD_COM_SEG)
/* Enable Segment LCD */
#define VSYNC_LCD_EN                 (* (reg8 *) VSYNC__LCD_EN)
/* Slew Rate Control */
#define VSYNC_SLW                    (* (reg8 *) VSYNC__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define VSYNC_PRTDSI__CAPS_SEL       (* (reg8 *) VSYNC__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define VSYNC_PRTDSI__DBL_SYNC_IN    (* (reg8 *) VSYNC__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define VSYNC_PRTDSI__OE_SEL0        (* (reg8 *) VSYNC__PRTDSI__OE_SEL0) 
#define VSYNC_PRTDSI__OE_SEL1        (* (reg8 *) VSYNC__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define VSYNC_PRTDSI__OUT_SEL0       (* (reg8 *) VSYNC__PRTDSI__OUT_SEL0) 
#define VSYNC_PRTDSI__OUT_SEL1       (* (reg8 *) VSYNC__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define VSYNC_PRTDSI__SYNC_OUT       (* (reg8 *) VSYNC__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(VSYNC__SIO_CFG)
    #define VSYNC_SIO_HYST_EN        (* (reg8 *) VSYNC__SIO_HYST_EN)
    #define VSYNC_SIO_REG_HIFREQ     (* (reg8 *) VSYNC__SIO_REG_HIFREQ)
    #define VSYNC_SIO_CFG            (* (reg8 *) VSYNC__SIO_CFG)
    #define VSYNC_SIO_DIFF           (* (reg8 *) VSYNC__SIO_DIFF)
#endif /* (VSYNC__SIO_CFG) */

/* Interrupt Registers */
#if defined(VSYNC__INTSTAT)
    #define VSYNC_INTSTAT            (* (reg8 *) VSYNC__INTSTAT)
    #define VSYNC_SNAP               (* (reg8 *) VSYNC__SNAP)
    
	#define VSYNC_0_INTTYPE_REG 		(* (reg8 *) VSYNC__0__INTTYPE)
#endif /* (VSYNC__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_VSYNC_H */


/* [] END OF FILE */
