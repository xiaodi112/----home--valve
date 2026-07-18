/**
 * Bootloader 用的最小 libopt: 只启用 BL 真正用到的外设,
 * 缩小代码体积 (FMC + RCU + GPIO + MISC).
 */
#ifndef GD32L23X_LIBOPT_H
#define GD32L23X_LIBOPT_H

#include "gd32l23x_fmc.h"
#include "gd32l23x_gpio.h"
#include "gd32l23x_misc.h"
#include "gd32l23x_rcu.h"
#include "gd32l23x_usart.h"

#endif /* GD32L23X_LIBOPT_H */
