#include "stm32f1xx_hal.h"

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    while (1) {
        __WFI();
    }
}
