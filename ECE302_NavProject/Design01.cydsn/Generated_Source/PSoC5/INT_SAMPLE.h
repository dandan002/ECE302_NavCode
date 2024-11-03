/*******************************************************************************
* File Name: INT_SAMPLE.h
* Version 1.70
*
*  Description:
*   Provides the function definitions for the Interrupt Controller.
*
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/
#if !defined(CY_ISR_INT_SAMPLE_H)
#define CY_ISR_INT_SAMPLE_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void INT_SAMPLE_Start(void);
void INT_SAMPLE_StartEx(cyisraddress address);
void INT_SAMPLE_Stop(void);

CY_ISR_PROTO(INT_SAMPLE_Interrupt);

void INT_SAMPLE_SetVector(cyisraddress address);
cyisraddress INT_SAMPLE_GetVector(void);

void INT_SAMPLE_SetPriority(uint8 priority);
uint8 INT_SAMPLE_GetPriority(void);

void INT_SAMPLE_Enable(void);
uint8 INT_SAMPLE_GetState(void);
void INT_SAMPLE_Disable(void);

void INT_SAMPLE_SetPending(void);
void INT_SAMPLE_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the INT_SAMPLE ISR. */
#define INT_SAMPLE_INTC_VECTOR            ((reg32 *) INT_SAMPLE__INTC_VECT)

/* Address of the INT_SAMPLE ISR priority. */
#define INT_SAMPLE_INTC_PRIOR             ((reg8 *) INT_SAMPLE__INTC_PRIOR_REG)

/* Priority of the INT_SAMPLE interrupt. */
#define INT_SAMPLE_INTC_PRIOR_NUMBER      INT_SAMPLE__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable INT_SAMPLE interrupt. */
#define INT_SAMPLE_INTC_SET_EN            ((reg32 *) INT_SAMPLE__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the INT_SAMPLE interrupt. */
#define INT_SAMPLE_INTC_CLR_EN            ((reg32 *) INT_SAMPLE__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the INT_SAMPLE interrupt state to pending. */
#define INT_SAMPLE_INTC_SET_PD            ((reg32 *) INT_SAMPLE__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the INT_SAMPLE interrupt. */
#define INT_SAMPLE_INTC_CLR_PD            ((reg32 *) INT_SAMPLE__INTC_CLR_PD_REG)


#endif /* CY_ISR_INT_SAMPLE_H */


/* [] END OF FILE */
