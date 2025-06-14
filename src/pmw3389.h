
#ifndef PMW3389_H
#define PMW3389_H

#include "config.h"

#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/kernel.h>
#include "srom.h"

// Pins
#ifndef PMW3389_CS_PIN
#define PMW3389_CS_PIN 24 // Default Chip Select pin, can be overridden in the build configuration
#endif

#ifndef PMW3389_SPI_DEV
#define PMW3389_SPI_DEV spi0 // Default SPI device, can be overridden in the build configuration
#endif



// Registers
#define Product_ID  0x00
#define Revision_ID 0x01
#define Motion  0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Raw_Data_Sum  0x08
#define Maximum_Raw_data  0x09
#define Minimum_Raw_data  0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Control 0x0D
#define Config1 0x0F
#define Config2 0x10
#define Angle_Tune  0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower  0x15
#define Rest1_Rate_Upper  0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower  0x18
#define Rest2_Rate_Upper  0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower  0x1B
#define Rest3_Rate_Upper  0x1C
#define Observation 0x24
#define Data_Out_Lower  0x25
#define Data_Out_Upper  0x26
#define Raw_Data_Dump 0x29
#define SROM_ID 0x2A
#define Min_SQ_Run  0x2B
#define Raw_Data_Threshold  0x2C
#define Config5 0x2F
#define Power_Up_Reset  0x3A
#define Shutdown  0x3B
#define Inverse_Product_ID  0x3F
#define LiftCutoff_Tune3  0x41
#define Angle_Snap  0x42
#define LiftCutoff_Tune1  0x4A
#define Motion_Burst  0x50
#define LiftCutoff_Tune_Timeout 0x58
#define LiftCutoff_Tune_Min_Length  0x5A
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst  0x64
#define LiftCutoff_Tune2  0x65

#define constrain(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

#define R_CHECK(func) do { \
    int ret = (func); \
    if (ret) { \
        printk("Error in %s: %d\n", #func, ret); \
        return ret; \
    } \
} while (0)

void begin_ncs(void);
void end_ncs(void);

int send_byte(uint8_t data);
int receive_byte(uint8_t *data);
int read_register(uint8_t reg);
int write_register(uint8_t reg, uint8_t value);
int srom_download(const uint8_t *firmware_data, size_t firmware_length);
int init_pmw3389(void);
int fetch_burst_xy(volatile short *x, volatile short *y);

static const struct device *pmw3389_spi_dev;
static struct spi_config pmw3389_spi_cfg;



#endif // PMW3389_H