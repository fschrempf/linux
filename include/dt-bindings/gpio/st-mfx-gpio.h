/*
 * This header provides constants for binding st,mfx-gpio.
 *
 * The first cell in ST MFX GPIO specifier is the gpio number.
 *
 * The second cell contains standard flag values specified in gpio.h and
 * specific flag values described here.
 */

#ifndef _DT_BINDINGS_GPIO_ST_MFX_GPIO_H
#define _DT_BINDINGS_GPIO_ST_MFX_GPIO_H

#include <dt-bindings/gpio/gpio.h>

#define GPIO_IN_NO_PULL		0
#define GPIO_IN_PUSH_PULL	2
#define GPIO_OUT_PUSH_PULL	GPIO_PUSH_PULL
#define GPIO_OUT_OPEN_DRAIN	GPIO_SINGLE_ENDED

#define GPIO_NO_PULL		0
#define GPIO_PULL_DOWN		0
#define GPIO_PULL_UP		4

#endif
