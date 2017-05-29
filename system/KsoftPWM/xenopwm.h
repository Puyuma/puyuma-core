/**
 * Access RTDM device for software PWM from user space.
 * Copyright (c) 2017 Shao-Hua Wang.
 */

#ifndef _XENOPWM_H_
#define _XENOPWM_H_

#include <rtdm/rtdm.h>

#define PWM_DRIVER "xenoPWM"
#define XENOPWM_VERSION 1

#define PIN_SET _IOW(RTDM_CLASS_EXPERIMENTAL, 0, int *)
#define GPIO_SET _IOW(RTDM_CLASS_EXPERIMENTAL, 1, int *)
#define PWM_INIT _IOW(RTDM_CLASS_EXPERIMENTAL, 2, int *)
#define PWM_SET _IOW(RTDM_CLASS_EXPERIMENTAL, 3, int *)

#define PWM_FREQ 100
#define PERIOD_NS (1000000000 / PWM_FREQ)
#define PERIOD_US (1000000 / PWM_FREQ)

#define MAX_PWM_NUM 5

typedef struct duty_cycle {
	int pin;
	int duty_cycle;
	int min;
	int max;
} Dutycycle;

#endif
