#ifndef __SUPER_CAP_H__
#define __SUPER_CAP_H__

#include "main.h"
//#define GPIO_SetBits(GPIOH, GPIO_Pin_12),GPIO_SetBits(GPIOH, GPIO_Pin_12) supercap_off
extern void super_cap_configuration();
extern void super_cap_on(void);
extern void super_cap_off(void);
extern void super_cap_none(void);
#endif
