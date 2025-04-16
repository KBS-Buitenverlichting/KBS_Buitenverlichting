#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wlxx_hal.h"

#ifdef __cplusplus
}
#endif

void SystemClock_Config(void);
void Gpio_Init(void);
void Error_Handler(void);
