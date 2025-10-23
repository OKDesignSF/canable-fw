#include "lock.h"

#include "stm32f0xx_hal.h"

bool lock_enter(void)
{
    bool interrupts_enabled = (__get_PRIMASK() == 0);
    if (interrupts_enabled) {
        __disable_irq();
    }

    return interrupts_enabled;
}

void lock_exit(bool interrupts_were_enabled)
{
    if (interrupts_were_enabled) {
        __enable_irq();
    }
}
