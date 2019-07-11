#include "stm32f4xx_gpio.h"

class Motor
{
public:

    virtual void init(const GPIO_InitTypeDef& gpioSettings, const GPIO_TypeDef *gpioPort) = 0;
    virtual

private:
    
};