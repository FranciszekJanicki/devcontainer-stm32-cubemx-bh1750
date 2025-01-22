#include "main.h"
#include "bh1750.hpp"
#include "gpio.h"
#include "i2c.h"
#include "i2c_device.hpp"
#include "system_clock.h"
#include "usart.h"
#include <cstdio>

int main()
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    using BH1750 = BH1750::BH1750;
    using namespace Utility;

    I2CDevice i2c_device{&hi2c1, std::to_underlying(BH1750::DevAddress::AD0_LOW)};

    BH1750
    bh1750{std::move(i2c_device), BH1750::MTREG_DEFAULT, BH1750::Mode::ONETIME_HIGH_RES_MODE};

    while (true) {
        printf("Light: %f\n\r", bh1750.get_light_scaled().value());
        HAL_Delay(50);
    }

    return 0;
}
