#ifndef __G431TIMER_H__
#define __G431TIMER_H__

#include <Arduino.h>
#include <SimpleFOC.h>
#include <stm32g4xx_hal.h>

#ifdef __cplusplus
extern "C" {
#endif

void pinModeAF(int ulPin, uint32_t Alternate);

//void EncoderInit(void);
TIM_HandleTypeDef EncoderInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __MATHER_H__ */