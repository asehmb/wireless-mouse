#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/kernel.h>
#include "srom.h"
#include "zephyr/sys/printk.h"

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


static const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));
static const struct gpio_dt_spec cs = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 24,
};

#define SPI_MODE_3 (SPI_MODE_CPOL | SPI_MODE_CPHA)


struct spi_config spi_cfg = {
    .frequency = 2000000, // 4 MHz
    .operation =  SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_3 | SPI_OP_MODE_MASTER,
    .slave = 10,
};

bool init_complete = false;
volatile int16_t delta_x = 0;
volatile int16_t delta_y = 0;
bool motion_detected = false;

unsigned long last_motion_time = 0;
unsigned long curr_time = 0;

#define R_CHECK(func) do { \
    int ret = (func); \
    if (ret) { \
        printk("Error in %s: %d\n", #func, ret); \
        return ret; \
    } \
} while (0)

void begin_ncs(void) {
    gpio_pin_set_dt(&cs, 0);
}
 void end_ncs(void) {
    gpio_pin_set_dt(&cs, 1);
}

int send_byte(uint8_t data) {
    uint8_t tx_buf[1] = {data};
    struct spi_buf tx_buffer = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_buffer_set = {.buffers = &tx_buffer, .count = 1};

    int ret = spi_write(spi_dev, &spi_cfg, &tx_buffer_set);
    if (ret) {
        printk("SPI write error: %d\n", ret);
    }
    return ret;
}

int receive_byte(uint8_t *data) {
    uint8_t tx_buf[1] = {0x00};
    uint8_t rx_buf[1] = {0};

    struct spi_buf tx = {.buf = tx_buf, .len = 1};
    struct spi_buf rx = {.buf = rx_buf, .len = 1};

    struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

    int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    if (ret) {
        printk("SPI receive error: %d\n", ret);
        return ret;
    }

    *data = rx_buf[0];
    return 0;
}


int read_register(uint8_t reg) {
    begin_ncs();
    send_byte(reg & 0x7F); // MSB = 0 for read
    k_busy_wait(100);      // T_SRAD

    uint8_t data = 0;
    int ret = receive_byte(&data);

    k_busy_wait(1);  // T_SRX
    end_ncs();
    k_busy_wait(120); // T_BEXIT

    if (ret) {
        printk("SPI read failed for reg 0x%02X\n", reg);
        return -1;
    }

    return data;
}


int write_register(uint8_t reg, uint8_t data) {
    begin_ncs();
    uint8_t tx_buf[2] = {reg | 0x80, data};  // MSB = 1 for write
    struct spi_buf tx_buf_struct = {.buf = tx_buf, .len = 2};
    struct spi_buf_set tx_buf_set = {.buffers = &tx_buf_struct, .count = 1};

    int ret = spi_write(spi_dev, &spi_cfg, &tx_buf_set);
    if (ret) {
        printk("SPI write error: %d\n", ret);
    }

    k_busy_wait(20);  // Wait at least 20 us after writing
    end_ncs();
    k_busy_wait(120); // Wait at least 100 us after NCS deasserted
    return ret;
}

int read_burst(uint8_t *data, int len) {
    begin_ncs();
    send_byte(Motion_Burst); // Do not OR with 0x80, it's a read register
    k_busy_wait(35); // Per datasheet: 35us after Motion_Burst write

    struct spi_buf rx = {.buf = data, .len = len};
    struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

    int ret = spi_read(spi_dev, &spi_cfg, &rx_set);
    end_ncs();
    k_busy_wait(120); // Post-burst wait

    return ret;
}

int fetch_burst_xy(volatile short *x, volatile short *y) {
    uint8_t data[12] = {0};
    int ret = read_burst(data, sizeof(data));
    if (ret) {
        printk("Error reading burst data: %d\n", ret);
        return ret;
    }

    // Combine the low and high bytes for X and Y
    if (data[0] & 0x80) { // Check if motion is detected
        motion_detected = true;
        *x = (int16_t)((data[3] << 8) | data[2]);
        *y = (int16_t)((data[5] << 8) | data[4]);
        printk("Motion detected: Delta X: %d, Delta Y: %d\n", delta_x, delta_y);
    } else {
        motion_detected = false;
        *x = 0;
        *y = 0;
    }

    return 0; // Success
}


int init_pmw3389(void) {
    gpio_pin_configure_dt(&cs, GPIO_OUTPUT_HIGH); // Set to HIGH (inactive) by default
    // Reset the PMW3389
    end_ncs();
    begin_ncs();
    end_ncs();

    int ret = write_register(Power_Up_Reset, 0x5A);
    if (ret) {
        printk("Failed to reset PMW3389: %d\n", ret);
        return ret;
    }
    k_msleep(50); // Wait for the sensor to power up

    // read registers 0x02 to 0x06
    uint8_t reg_data[5];
    reg_data[0] = read_register(Motion);
    reg_data[1] = read_register(Delta_X_L);
    reg_data[2] = read_register(Delta_X_H);
    reg_data[3] = read_register(Delta_Y_L);
    reg_data[4] = read_register(Delta_Y_H);

    /* SROM DONWLOAD */
    ret = write_register(Config2, 0x20);
    if (ret) {
        printk("Failed to set Config2: %d\n", ret);
        return ret;
    }
    ret = write_register(SROM_Enable, 0x1D);
    if (ret) {
        printk("Failed to enable SROM: %d\n", ret);
        return ret;
    }
    k_msleep(10); // Wait for SROM to be enabled
    ret = write_register(SROM_Enable, 0x18);
    if (ret) {
        printk("Failed to set SROM Enable: %d\n", ret);
        return ret;
    }
    begin_ncs();
    send_byte(SROM_Load_Burst | 0x80); // Set MSB for SROM load burst
    k_busy_wait(15);

    for (int i = 0; i < firmware_length; i++) {
        uint8_t byte = firmware_data[i];
        struct spi_buf buf = {.buf = &byte, .len = 1};
        struct spi_buf_set tx = {.buffers = &buf, .count = 1};
        spi_write(spi_dev, &spi_cfg, &tx); // Do not re-toggle CS here
        k_busy_wait(15);
    }

    ret = write_register(Config2, 0x00);
    if (ret) {
        printk("Failed to reset Config2: %d\n", ret);
        return ret;
    }
    ret = write_register(Config1, 0x15);
    if (ret) {
        printk("Failed set CPI in Config1: %d\n", ret);
        return ret;
    }
    end_ncs();

    k_msleep(10); // Wait for the sensor to stabilize after SROM load
    uint8_t srom_id = read_register(SROM_ID);
    printk("SROM ID: %02X\n", srom_id);

    reg_data[0] = read_register(Motion);
    reg_data[1] = read_register(Delta_X_L);
    reg_data[2] = read_register(Delta_X_H);
    reg_data[3] = read_register(Delta_Y_L);
    reg_data[4] = read_register(Delta_Y_H);
    printk("Initial Motion: %02X, Delta_X_L: %02X%02X, Delta_X_H: %02X%02X\n ",
           reg_data[0], reg_data[1], reg_data[2], reg_data[3], reg_data[4]);
    k_msleep(5000);

    return 0; // Success
}


int main(void) {
    if (!device_is_ready(spi_dev)) {
        printk("SPI device not ready\n");
        return -1;
    }

    if (!device_is_ready(cs.port)) {
        printk("CS GPIO device not ready\n");
        return -1;
    }

    init_pmw3389();
    init_complete = true;

    while (1) {

        uint8_t motion = read_register(0x02);

        if (motion & 0x80) {
            // Motion detected
            delta_x = (int16_t)((read_register(0x04)) | (read_register(0x05) << 8));
            delta_y = (int16_t)((read_register(0x06)) | (read_register(0x07) << 8));
            printk("Motion: X: %d, Y: %d\n", delta_x, delta_y);
        } else {
            printk("No motion\n");
        }
        k_msleep(5); // Polling interval
    }

    return 0;
}