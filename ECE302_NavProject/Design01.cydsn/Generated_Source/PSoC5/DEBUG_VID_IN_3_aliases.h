/*******************************************************************************
* File Name: DEBUG_VID_IN_3.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_DEBUG_VID_IN_3_ALIASES_H) /* Pins DEBUG_VID_IN_3_ALIASES_H */
#define CY_PINS_DEBUG_VID_IN_3_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"


/***************************************
*              Constants        
***************************************/
#define DEBUG_VID_IN_3_0			(DEBUG_VID_IN_3__0__PC)
#define DEBUG_VID_IN_3_0_INTR	((uint16)((uint16)0x0001u << DEBUG_VID_IN_3__0__SHIFT))

#define DEBUG_VID_IN_3_INTR_ALL	 ((uint16)(DEBUG_VID_IN_3_0_INTR))

#endif /* End Pins DEBUG_VID_IN_3_ALIASES_H */


/* [] END OF FILE */
