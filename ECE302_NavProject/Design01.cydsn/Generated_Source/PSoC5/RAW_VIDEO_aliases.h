/*******************************************************************************
* File Name: RAW_VIDEO.h  
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

#if !defined(CY_PINS_RAW_VIDEO_ALIASES_H) /* Pins RAW_VIDEO_ALIASES_H */
#define CY_PINS_RAW_VIDEO_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"


/***************************************
*              Constants        
***************************************/
#define RAW_VIDEO_0			(RAW_VIDEO__0__PC)
#define RAW_VIDEO_0_INTR	((uint16)((uint16)0x0001u << RAW_VIDEO__0__SHIFT))

#define RAW_VIDEO_INTR_ALL	 ((uint16)(RAW_VIDEO_0_INTR))

#endif /* End Pins RAW_VIDEO_ALIASES_H */


/* [] END OF FILE */
