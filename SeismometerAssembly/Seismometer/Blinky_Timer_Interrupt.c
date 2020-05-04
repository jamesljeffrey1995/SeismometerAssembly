/*----------------------------------------------------------------------------
 * Name:    Blinky_Timer_Interrupt.c
 * Purpose: Seisometre 
 * Note(s): 
 *----------------------------------------------------------------------------
 * Test Program for STM32F4 Discovery Board
 * Version: 1.0 
 * Author: James Jeffrey 
 * 7/01/2017
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include "STM32F4xx.h"
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include <math.h>
#include "ADC.h"
/*----------------------------------------------------------------------------
  USER function prototypes
 *----------------------------------------------------------------------------*/

void LED_Init_1(void); 
void Delay (uint32_t dlyTicks);
void SystemClock_Config(void);
void Timer_Interrupt_Init(void);
void Basic_Timer_Init(void);
void Soft_Delay(void);


/*----------------------------------------------------------------------------
  Global variables
 *----------------------------------------------------------------------------*/
volatile uint32_t msTicks;                      /* counts 1ms timeTicks       */
TIM_HandleTypeDef TIM2_Handle;
volatile uint32_t adcVal;
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	/*Initialise the variables within main*/
	float a;
	double d;
	float ml;
	double timer_add;
	double b;
  
  /* Setup to use the STM32 HAL libraries */
  SystemClock_Config();                     /* Configure the System Clock     */
	SystemCoreClockUpdate();
	HAL_Init();                               /* Initialize the HAL Library     */

	/* Call user initialation functions*/
			LED_Init_1();
			Basic_Timer_Init();
			ADC_Init();
			Timer_Interrupt_Init();
  while(1)  	/* Loop forever */
		{     
			Delay(1000);
			a=((adcVal/20.475)-100);
			b=(4095/3);
			printf("\nThe displacement, measured in microns = %f",a);/*Displays the displacement in the debug menue*/
			printf("\nThe digital value = %d",adcVal); /*Displays the digital value in the debug menue*/
			d=(timer_add*100e-6*8); /*This converts the timer into seconds and then is multiplied by the speed*/
			ml = (log(a)+2.56*d-1.67);/*equation given to work out the richter scale*/
			
			if (((a>-50)&&(a<-5))|(((a<50)&&(a>5)))){ /*this means that the microns have to be less than -50 and more than -5, or less 50 and more than 5*/
				GPIOD->ODR = GPIOD->ODR & 0xEFFF; /*0x1000 = 1110 1111 1111 1111 */
				GPIOD->ODR = GPIOD->ODR | 0x4000;
				printf("\nEarthquake occuring");
				printf("\nThe magnitude of the earthquake is at %f",ml);
			}
			
			else if((a>-10)&&(a<10)){/*So when its between -10 and 10, the clock resets*/
				TIM6->CNT=0; /*This resets the clock*/
				TIM6->CR1|=TIM_CR1_CEN; /*This enables the clock*/
				GPIOD->ODR = GPIOD->ODR &~ 0x4000;
			}
			
			else if ((a<-50)|(a>50)){
				TIM6->CR1 |= TIM_CR1_UDIS; /*This disables TIM6*/
				timer_add = TIM6->CNT;
				printf("\nThe magnitude of the earthquake is at %f",ml);
				Delay(500);
			}
			
			if (adcVal<=b){/*green LED flashes if less or equal to 1v*/
				GPIOD->ODR = GPIOD->ODR | 0x1000; /*0x1000 = 0001 0000 0000 0000 */
				Soft_Delay();
				GPIOD->ODR = GPIOD->ODR &~ 0x1000; /*0x1000 = 0001 0000 0000 0000 */	
    }
			if ((adcVal>b)&&(adcVal<=(2*b))){/*orange LED flashes if over 1V but below or equal to 2V*/
			GPIOD->ODR = GPIOD->ODR | 0x2000; /*0x2000 = 0010 0000 0000 0000 */
			Soft_Delay();
			GPIOD->ODR = GPIOD->ODR &~ 0x2000; /*0x2000 = 0010 0000 0000 0000 */
			}
			if ((adcVal>(2*b))&&(adcVal<=(3*b))){/*green and orange flash if over 2V*/
			GPIOD->ODR = GPIOD->ODR | 0x2000; /*0x2000 = 0010 0000 0000 0000 */
			GPIOD->ODR = GPIOD->ODR | 0x1000; /*0x1000 = 0001 0000 0000 0000 */
			Soft_Delay();
			GPIOD->ODR = GPIOD->ODR &~ 0x2000; /*0x2000 = 0010 0000 0000 0000 */
			GPIOD->ODR = GPIOD->ODR &~ 0x1000; /*0x1000 = 0001 0000 0000 0000 */
			}
	}
}

/*----------------------------------------------------------------------------
  USER functions
 *----------------------------------------------------------------------------*/


/*Inititialise GPIOD bit 12 as output connected to the green (left) LED on discovery board */

void LED_Init_1 (void) {

  RCC->AHB1ENR 	  |= ((1UL <<  3) );        	/* Enable GPIOD clock                */
  GPIOD->MODER    &= ~((3UL << 2*12));  			/* PD.12  is output               */
  GPIOD->MODER    |=  ((1UL << 2*12)  ); 
  GPIOD->OTYPER   &= ~((1UL <<   12) );  			/* PD.12 is output Push-Pull     */
  GPIOD->OSPEEDR  &= ~((3UL << 2*12) );   		/* PD.12 is 50MHz Fast Speed     */
  GPIOD->OSPEEDR  |=  ((2UL << 2*12) ); 
  GPIOD->PUPDR    &= ~((3UL << 2*12) );  			/* PD.12 is Pull up              */
  GPIOD->PUPDR    |=  ((1UL << 2*12) ); 
	
	RCC->AHB1ENR 	  |= ((1UL <<  3) );        	/* Enable GPIOD clock                */
  GPIOD->MODER    &= ~((3UL << 2*13));  			/* PD.13  is output               */
  GPIOD->MODER    |=  ((1UL << 2*13)  ); 
  GPIOD->OTYPER   &= ~((1UL <<   13) );  			/* PD.13 is output Push-Pull     */
  GPIOD->OSPEEDR  &= ~((3UL << 2*13) );   		/* PD.13 is 50MHz Fast Speed     */
  GPIOD->OSPEEDR  |=  ((2UL << 2*13) ); 
  GPIOD->PUPDR    &= ~((3UL << 2*13) );  			/* PD.13 is Pull up              */
  GPIOD->PUPDR    |=  ((1UL << 2*13) ); 
	
	RCC->AHB1ENR 	  |= ((1UL <<  3) );        	/* Enable GPIOD clock                */
  GPIOD->MODER    &= ~((3UL << 2*14));  			/* PD.14  is output               */
  GPIOD->MODER    |=  ((1UL << 2*14)  ); 
  GPIOD->OTYPER   &= ~((1UL <<   14) );  			/* PD.14 is output Push-Pull     */
  GPIOD->OSPEEDR  &= ~((3UL << 2*14) );   		/* PD.14 is 50MHz Fast Speed     */
  GPIOD->OSPEEDR  |=  ((2UL << 2*14) ); 
  GPIOD->PUPDR    &= ~((3UL << 2*14) );  			/* PD.14 is Pull up              */
  GPIOD->PUPDR    |=  ((1UL << 2*14) ); 

}


/*Inititialise Timer 2 using HAL Library */
/*Timer 2 has a 16 bit prescaler & 32bit Timer/counter */
/*Prescaler of 8400 gives a timer incremeent every 100us (10kHz) */
/* Timnr set to 100000 gives 10s delay note Timer 2 is 32 bit hence >65536 */

void Timer_Interrupt_Init(void)
{
	  __TIM2_CLK_ENABLE();
    TIM2_Handle.Init.Prescaler = 8400;
    TIM2_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM2_Handle.Init.Period = 1000;
    TIM2_Handle.Instance = TIM2;   //Same timer whose clocks we enabled
    HAL_TIM_Base_Init(&TIM2_Handle);     // Init timer
    HAL_TIM_Base_Start_IT(&TIM2_Handle); // start timer interrupts
	
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);   //  Set the priority
    HAL_NVIC_EnableIRQ(TIM2_IRQn);					//  Enable the interrupt

}



void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&TIM2_Handle, TIM_FLAG_UPDATE) != RESET)      //In case other interrupts are also running
    {
        if (__HAL_TIM_GET_ITSTATUS(&TIM2_Handle, TIM_IT_UPDATE) != RESET)   
        {
            __HAL_TIM_CLEAR_FLAG(&TIM2_Handle, TIM_FLAG_UPDATE);      //Clear the interrupt
				ADC_StartCnv();
				adcVal = ADC_GetVal();
        }
    }
}

void Soft_Delay(void){
	uint32_t i=10000000;
	
	while(i>0){
		i--;
	}
}
	
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;
}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {                                              
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}

/**----------------------------------------------------------------------------
  * System Clock Configuration
  *----------------------------------------------------------------------------*/
void Basic_Timer_Init(void)
{
	   
	uint16_t PrescalerValue = 0;
	
	/* -----------------------------------------------------------------------
    TIM6 Configuration: 
    
    In this example TIM6 input clock (TIM6CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1 (4 in this example).   
      
			core clock = (HCLK)  = 168MHz  
			PCLK1 = HCLK / 4  
	    TIM6CLK = 2 * PCLK1 
      => TIM6CLK = HCLK / 2 = SystemCoreClock /2 = 168MHz/2 = 84MHz
          
    To get TIM6 counter clock at 10 kHz (100us), the prescaler is computed as follows:
       Prescaler = (TIM6CLK / TIM6 counter clock) - 1
       Prescaler = ((SystemCoreClock /2) /10 k Hz) - 1
                                            
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  ----------------------------------------------------------------------- */  
	  
	  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 10000) - 1;
	  //printf(" \nSystemCoreClock = %d, Prescale Value  = %d",  SystemCoreClock,PrescalerValue);
	  
		RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;   // Enable TIM6 clock
    TIM6->DIER |= TIM_DIER_UIE;   // Enable interrupt on update event
	  TIM6->PSC = PrescalerValue;     
	
}

void Hardware_Delay(void)
{
	
	TIM6->CNT=65535- 10000;    //Timer counts up and generates an interrupt flag on overflow
	TIM6->CR1 |= TIM_CR1_CEN;   // Enable TIM6 counter
	while((TIM6->SR&0x0001)==0)    //Poll the timer Update Interrupt Flag (UIF) on the Timer Status Register (SR)
	{
		; //wait for timer overflow
	}
  TIM6->SR &= ~1;   // clear UIF 

}



void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the
     device is clocked below the maximum system frequency (see datasheet). */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}
