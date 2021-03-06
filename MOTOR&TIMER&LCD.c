#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_lcd.h"
#include "stm32l152_glass_lcd.h"
#include "stdio.h"
#include "string.h"
#include "dwt_delay.h"
#include "stm32l1xx_ll_exti.h"

void SystemClock_Config(void);
void TIMBase_Config(void);
void PIN_MOTOR_Config(void);
uint16_t cnt = 0;
uint16_t cnt2 = 0;
char disp_str[7];

int main()
{
	SystemClock_Config();
	TIMBase_Config();
	PIN_MOTOR_Config();
	LCD_GLASS_Init();
	LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_7);
	LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_6);
	while(1)
	{
			cnt = LL_TIM_GetCounter(TIM2);
			if(cnt >= 1000-1)
			{
				if (cnt2 != 10)
				{
					LL_LCD_Clear();
					sprintf(disp_str,"TMR %d",cnt2);
					LCD_GLASS_DisplayString((uint8_t*)disp_str);
					LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_12);
					cnt2 +=1;
				}
				else
				{
					LL_LCD_Clear();
					sprintf(disp_str,"TMR");
					LCD_GLASS_DisplayString((uint8_t*)disp_str);
					cnt2=0;
					LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_12);
					LL_mDelay(10000);
				}
			}
	}
	
}

void TIMBase_Config(void)
{
	LL_TIM_InitTypeDef timbase_initstructure;

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2) ;

	timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	timbase_initstructure.Autoreload = (1000 - 1);
	timbase_initstructure.Prescaler = 32000 - 1;

	LL_TIM_Init(TIM2, &timbase_initstructure);
	LL_TIM_EnableCounter(TIM2) ;
}

void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}


void PIN_MOTOR_Config(void)
{
	LL_GPIO_InitTypeDef gpio_initstructure,PC12_init,pb7_init;
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	pb7_init.Mode = LL_GPIO_MODE_OUTPUT;
	pb7_init.Pull = LL_GPIO_PULL_NO;
	pb7_init.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
	pb7_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
	pb7_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(GPIOB,&pb7_init);
	
	PC12_init.Mode = LL_GPIO_MODE_OUTPUT;
	PC12_init.Pull = LL_GPIO_PULL_NO;
	PC12_init.Pin = LL_GPIO_PIN_12;
	PC12_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
	PC12_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(GPIOA,&PC12_init);
}

