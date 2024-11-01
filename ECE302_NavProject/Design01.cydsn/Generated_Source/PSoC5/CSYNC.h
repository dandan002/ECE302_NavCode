/*******************************************************************************
* File Name: CSYNC.h  
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

#if !defined(CY_PINS_CSYNC_H) /* Pins CSYNC_H */
#define CY_PINS_CSYNC_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CSYNC_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 CSYNC__PORT == 15 && ((CSYNC__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CSYNC_Write(uint8 value);
void    CSYNC_SetDriveMode(uint8 mode);
uint8   CSYNC_ReadDataReg(void);
uint8   CSYNC_Read(void);
void    CSYNC_SetInterruptMode(uint16 position, uint16 mode);
uint8   CSYNC_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CSYNC_SetDriveMode() function.
     *  @{
     */
        #define CSYNC_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define CSYNC_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define CSYNC_DM_RES_UP          PIN_DM_RES_UP
        #define CSYNC_DM_RES_DWN         PIN_DM_RES_DWN
        #define CSYNC_DM_OD_LO           PIN_DM_OD_LO
        #define CSYNC_DM_OD_HI           PIN_DM_OD_HI
        #define CSYNC_DM_STRONG          PIN_DM_STRONG
        #define CSYNC_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CSYNC_MASK               CSYNC__MASK
#define CSYNC_SHIFT              CSYNC__SHIFT
#define CSYNC_WIDTH              1u

/* Interrupt constants */
#if defined(CSYNC__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CSYNC_SetInterruptMode() function.
     *  @{
     */
        #define CSYNC_INTR_NONE      (uint16)(0x0000u)
        #define CSYNC_INTR_RISING    (uint16)(0x0001u)
        #define CSYNC_INTR_FALLING   (uint16)(0x0002u)
        #define CSYNC_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define CSYNC_INTR_MASK      (0x01u) 
#endif /* (CSYNC__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CSYNC_PS                     (* (reg8 *) CSYNC__PS)
/* Data Register */
#define CSYNC_DR                     (* (reg8 *) CSYNC__DR)
/* Port Number */
#define CSYNC_PRT_NUM                (* (reg8 *) CSYNC__PRT) 
/* Connect to Analog Globals */                                                  
#define CSYNC_AG                     (* (reg8 *) CSYNC__AG)                       
/* Analog MUX bux enable */
#define CSYNC_AMUX                   (* (reg8 *) CSYNC__AMUX) 
/* Bidirectional Enable */                                                        
#define CSYNC_BIE                    (* (reg8 *) CSYNC__BIE)
/* Bit-mask for Aliased Register Access */
#define CSYNC_BIT_MASK               (* (reg8 *) CSYNC__BIT_MASK)
/* Bypass Enable */
#define CSYNC_BYP                    (* (reg8 *) CSYNC__BYP)
/* Port wide control signals */                                                   
#define CSYNC_CTL                    (* (reg8 *) CSYNC__CTL)
/* Drive Modes */
#define CSYNC_DM0                    (* (reg8 *) CSYNC__DM0) 
#define CSYNC_DM1                    (* (reg8 *) CSYNC__DM1)
#define CSYNC_DM2                    (* (reg8 *) CSYNC__DM2) 
/* Input Buffer Disable Override */
#define CSYNC_INP_DIS                (* (reg8 *) CSYNC__INP_DIS)
/* LCD Common or Segment Drive */
#define CSYNC_LCD_COM_SEG            (* (reg8 *) CSYNC__LCD_COM_SEG)
/* Enable Segment LCD */
#define CSYNC_LCD_EN                 (* (reg8 *) CSYNC__LCD_EN)
/* Slew Rate Control */
#define CSYNC_SLW                    (* (reg8 *) CSYNC__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CSYNC_PRTDSI__CAPS_SEL       (* (reg8 *) CSYNC__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CSYNC_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CSYNC__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CSYNC_PRTDSI__OE_SEL0        (* (reg8 *) CSYNC__PRTDSI__OE_SEL0) 
#define CSYNC_PRTDSI__OE_SEL1        (* (reg8 *) CSYNC__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CSYNC_PRTDSI__OUT_SEL0       (* (reg8 *) CSYNC__PRTDSI__OUT_SEL0) 
#define CSYNC_PRTDSI__OUT_SEL1       (* (reg8 *) CSYNC__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CSYNC_PRTDSI__SYNC_OUT       (* (reg8 *) CSYNC__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CSYNC__SIO_CFG)
    #define CSYNC_SIO_HYST_EN        (* (reg8 *) CSYNC__SIO_HYST_EN)
    #define CSYNC_SIO_REG_HIFREQ     (* (reg8 *) CSYNC__SIO_REG_HIFREQ)
    #define CSYNC_SIO_CFG            (* (reg8 *) CSYNC__SIO_CFG)
    #define CSYNC_SIO_DIFF           (* (reg8 *) CSYNC__SIO_DIFF)
#endif /* (CSYNC__SIO_CFG) */

/* Interrupt Registers */
#if defined(CSYNC__INTSTAT)
    #define CSYNC_INTSTAT            (* (reg8 *) CSYNC__INTSTAT)
    #define CSYNC_SNAP               (* (reg8 *) CSYNC__SNAP)
    
	#define CSYNC_0_INTTYPE_REG 		(* (reg8 *) CSYNC__0__INTTYPE)
#endif /* (CSYNC__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_CSYNC_H */


/* [] END OF FILE */
