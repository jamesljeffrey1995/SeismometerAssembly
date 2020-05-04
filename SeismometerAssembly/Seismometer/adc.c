/*-----------------------------------------------------------------------------
 * Name:    ADC.c
 * Purpose: Low level ADC functions
 * Note(s): 
 *-----------------------------------------------------------------------------
 * Alan Holloway 9/9/2014  adapted from Keil examples 
 *----------------------------------------------------------------------------*/
#include <stm32f4xx.h>                  /* STM32F4xx Definitions              */
#include "ADC.h"

#define ADC_BITS      12                /* Number of A/D converter bits       */
#define ADC_TOUT  200000                /* Approx. A/D conversion timeout     */

/*-----------------------------------------------------------------------------
  Function that initializes ADC
*----------------------------------------------------------------------------*/
void ADC_Init (void)  {

  RCC->APB2ENR |= (1UL <<  10);         /* Enable ADC3 clock                  */
	RCC->AHB1ENR |= (1<<0);                  // enable peripheral clock for GPIOA
	GPIOA->MODER |= (3UL << 3*2);         			  /* PA3 is Analog mode  -  IN3  */
	
  ADC3->SQR1   =   0;
  ADC3->SQR2   =   0;
  ADC3->SQR3   =  (3UL <<  0);          /* SQ1 = channel 3                    */
	
  ADC3->SMPR1  =  (7UL <<  6);          /* Channel 3 sample time is 480 cyc. */
  ADC3->SMPR2  =   0;                   /* Clear register                     */
		
  ADC3->CR1   |=  ( 1UL <<  5);         /* enable EOC interrupt               */
  ADC3->CR2   |=  ( 1UL <<  0);         /* ADC enable                         */	
}

/*-----------------------------------------------------------------------------
 *      ADC_GetVal:  Get converted value from the ADC
 *
 * Parameters:  (none)
 * Return:      converted value or -1 if conversion in progress/failed
 *----------------------------------------------------------------------------*/
int32_t ADC_GetVal (void) {
  int32_t i;

  for (i = ADC_TOUT; i; i--) {
					
		if (ADC3->SR & (1<<1))  {                // ADC1 EOC interrupt?
    ADC3->SR &= ~(1<<1);                   // clear EOC interrupt
		return(ADC3->DR) ;          // Read Conversion Result					
    }
  }
  return -1;                            /* Conversion timeout expired         */
}

/*-----------------------------------------------------------------------------
 *      ADC_StartCnv:  Start A/D conversion 
 *
 * Parameters:  (none)
 * Return:      (none)
 *----------------------------------------------------------------------------*/
void ADC_StartCnv (void) {
	
	ADC3->CR2 |= (1UL << 30);               /* Start conversion                   */
}

/*-----------------------------------------------------------------------------
 * End of file
 *----------------------------------------------------------------------------*/
