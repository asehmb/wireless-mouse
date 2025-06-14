/* https://github.com/mrjohnk/PMW3389DM/blob/master/Arduino%20Examples/PMW3389DM-polling/SROM.ino */

// the firmeware that is uploaded in the ADNS each time it boots
#ifndef SROM_H
#define SROM_H
#include <stdint.h>

extern const unsigned short firmware_length;

extern const uint8_t firmware_data[];

#endif // SRC_SROM_H