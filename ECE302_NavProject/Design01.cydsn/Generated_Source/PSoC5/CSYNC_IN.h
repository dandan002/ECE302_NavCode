/*******************************************************************************
* File Name: CSYNC_IN.h  
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

#if !defined(CY_PINS_CSYNC_IN_H) /* Pins CSYNC_IN_H */
#define CY_PINS_CSYNC_IN_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CSYNC_IN_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 CSYNC_IN__PORT == 15 && ((CSYNC_IN__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CSYNC_IN_Write(uint8 value);
void    CSYNC_IN_SetDriveMode(uint8 mode);
uint8   CSYNC_IN_ReadDataReg(void);
uint8   CSYNC_IN_Read(void);
void    CSYNC_IN_SetInterruptMode(uint16 position, uint16 mode);
uint8   CSYNC_IN_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CSYNC_IN_SetDriveMode() function.
     *  @{
     */
        #define CSYNC_IN_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define CSYNC_IN_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define CSYNC_IN_DM_RES_UP          PIN_DM_RES_UP
        #define CSYNC_IN_DM_RES_DWN         PIN_DM_RES_DWN
        #define CSYNC_IN_DM_OD_LO           PIN_DM_OD_LO
        #define CSYNC_IN_DM_OD_HI           PIN_DM_OD_HI
        #define CSYNC_IN_DM_STRONG          PIN_DM_STRONG
        #define CSYNC_IN_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CSYNC_IN_MASK               CSYNC_IN__MASK
#define CSYNC_IN_SHIFT              CSYNC_IN__SHIFT
#define CSYNC_IN_WIDTH              1u

/* Interrupt constants */
#if defined(CSYNC_IN__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CSYNC_IN_SetInterruptMode() function.
     *  @{
     */
        #define CSYNC_IN_INTR_NONE      (uint16)(0x0000u)
        #define CSYNC_IN_INTR_RISING    (uint16)(0x0001u)
        #define CSYNC_IN_INTR_FALLING   (uint16)(0x0002u)
        #define CSYNC_IN_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define CSYNC_IN_INTR_MASK      (0x01u) 
#endif /* (CSYNC_IN__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CSYNC_IN_PS                     (* (reg8 *) CSYNC_IN__PS)
/* Data Register */
#define CSYNC_IN_DR                     (* (reg8 *) CSYNC_IN__DR)
/* Port Number */
#define CSYNC_IN_PRT_NUM                (* (reg8 *) CSYNC_IN__PRT) 
/* Connect to Analog Globals */                                                  
#define CSYNC_IN_AG                     (* (reg8 *) CSYNC_IN__AG)                       
/* Analog MUX bux enable */
#define CSYNC_IN_AMUX                   (* (reg8 *) CSYNC_IN__AMUX) 
/* Bidirectional Enable */                                                        
#define CSYNC_IN_BIE                    (* (reg8 *) CSYNC_IN__BIE)
/* Bit-mask for Aliased Register Access */
#define CSYNC_IN_BIT_MASK               (* (reg8 *) CSYNC_IN__BIT_MASK)
/* Bypass Enable */
#define CSYNC_IN_BYP                    (* (reg8 *) CSYNC_IN__BYP)
/* Port wide control signals */                                                   
#define CSYNC_IN_CTL                    (* (reg8 *) CSYNC_IN__CTL)
/* Drive Modes */
#define CSYNC_IN_DM0                    (* (reg8 *) CSYNC_IN__DM0) 
#define CSYNC_IN_DM1                    (* (reg8 *) CSYNC_IN__DM1)
#define CSYNC_IN_DM2                    (* (reg8 *) CSYNC_IN__DM2) 
/* Input Buffer Disable Override */
#define CSYNC_IN_INP_DIS                (* (reg8 *) CSYNC_IN__INP_DIS)
/* LCD Common or Segment Drive */
#define CSYNC_IN_LCD_COM_SEG            (* (reg8 *) CSYNC_IN__LCD_COM_SEG)
/* Enable Segment LCD */
#define CSYNC_IN_LCD_EN                 (* (reg8 *) CSYNC_IN__LCD_EN)
/* Slew Rate Control */
#define CSYNC_IN_SLW                    (* (reg8 *) CSYNC_IN__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CSYNC_IN_PRTDSI__CAPS_SEL       (* (reg8 *) CSYNC_IN__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CSYNC_IN_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CSYNC_IN__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CSYNC_IN_PRTDSI__OE_SEL0        (* (reg8 *) CSYNC_IN__PRTDSI__OE_SEL0) 
#define CSYNC_IN_PRTDSI__OE_SEL1        (* (reg8 *) CSYNC_IN__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CSYNC_IN_PRTDSI__OUT_SEL0       (* (reg8 *) CSYNC_IN__PRTDSI__OUT_SEL0) 
#define CSYNC_IN_PRTDSI__OUT_SEL1       (* (reg8 *) CSYNC_IN__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CSYNC_IN_PRTDSI__SYNC_OUT       (* (reg8 *) CSYNC_IN__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CSYNC_IN__SIO_CFG)
    #define CSYNC_IN_SIO_HYST_EN        (* (reg8 *) CSYNC_IN__SIO_HYST_EN)
    #define CSYNC_IN_SIO_REG_HIFREQ     (* (reg8 *) CSYNC_IN__SIO_REG_HIFREQ)
    #define CSYNC_IN_SIO_CFG            (* (reg8 *) CSYNC_IN__SIO_CFG)
    #define CSYNC_IN_SIO_DIFF           (* (reg8 *) CSYNC_IN__SIO_DIFF)
#endif /* (CSYNC_IN__SIO_CFG) */

/* Interrupt Registers */
#if defined(CSYNC_IN__INTSTAT)
    #define CSYNC_IN_INTSTAT            (* (reg8 *) CSYNC_IN__INTSTAT)
    #define CSYNC_IN_SNAP               (* (reg8 *) CSYNC_IN__SNAP)
    
	#define CSYNC_IN_0_INTTYPE_REG 		(* (reg8 *) CSYNC_IN__0__INTTYPE)
#endif /* (CSYNC_IN__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_CSYNC_IN_H */


/* [] END OF FILE */
