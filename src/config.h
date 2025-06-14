

#ifndef CONFIG_H
#define CONFIG_H

#define PMW3389_SPI_DEV DEVICE_DT_GET(DT_NODELABEL(spi2))
#define PMW3389_CS_PIN 24

// doesn't account for gpio port WIP
#define LEFT_BUTTON_PIN  11
#define RIGHT_BUTTON_PIN 12
#define MIDDLE_BUTTON_PIN 13
#define MIDDLE_LEFT_BUTTON_PIN 14
#define MIDDLE_RIGHT_BUTTON_PIN 15
#define SCROLL_WHEEL_PIN 16

#include "sensors/pmw3389/pmw3389.h"



#endif // CONFIG_H
