#include "g431timer.h"

void pinModeAF(int ulPin, uint32_t Alternate)
{
   int pn = digitalPinToPinName(ulPin);

   if (STM_PIN(pn) < 8) {
      LL_GPIO_SetAFPin_0_7( get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), Alternate);
   } else {
      LL_GPIO_SetAFPin_8_15(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), Alternate);
   }

   LL_GPIO_SetPinMode(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), LL_GPIO_MODE_ALTERNATE);
}

TIM_HandleTypeDef EncoderInit(uint32_t period)
{
   // use standaard Arduino function for basic setup
   pinMode(PB6, INPUT);  // TIM4
   pinMode(PB7, INPUT);  // TIM4

   // use custom function to switch to AlternateFuction.
   pinModeAF(PB6, GPIO_AF2_TIM4);
   pinModeAF(PB7, GPIO_AF2_TIM4);

   TIM_HandleTypeDef       Encoder_Handle;
   TIM_Encoder_InitTypeDef sEncoderConfig;

   Encoder_Handle.Init.Period             = period;
   Encoder_Handle.Init.Prescaler          = 0;
   Encoder_Handle.Init.ClockDivision      = 0;
   Encoder_Handle.Init.CounterMode        = TIM_COUNTERMODE_UP;
   Encoder_Handle.Init.RepetitionCounter  = 0;
   Encoder_Handle.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
   sEncoderConfig.EncoderMode             = TIM_ENCODERMODE_TI12;

   sEncoderConfig.IC1Polarity             = TIM_ICPOLARITY_RISING;
   sEncoderConfig.IC1Selection            = TIM_ICSELECTION_DIRECTTI;
   sEncoderConfig.IC1Prescaler            = TIM_ICPSC_DIV1;
   sEncoderConfig.IC1Filter               = 0;

   sEncoderConfig.IC2Polarity             = TIM_ICPOLARITY_RISING;
   sEncoderConfig.IC2Selection            = TIM_ICSELECTION_DIRECTTI;
   sEncoderConfig.IC2Prescaler            = TIM_ICPSC_DIV1;
   sEncoderConfig.IC2Filter               = 0;

   Encoder_Handle.Instance = TIM4;
   enableTimerClock(&Encoder_Handle);
   if(HAL_TIM_Encoder_Init(&Encoder_Handle, &sEncoderConfig) != HAL_OK) printf("Init error of TIM4");
   HAL_TIM_Encoder_Start(&Encoder_Handle, TIM_CHANNEL_ALL);
   
   return Encoder_Handle;
}

