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

#define constrain(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))


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
    uint8_t tx_buf_data[12] = {0};         // Dummy bytes
    uint8_t rx_buf_data[12] = {0};


    struct spi_buf tx_buf = {
        .buf = tx_buf_data,
        .len = sizeof(tx_buf_data),
    };
    struct spi_buf rx_buf = {
        .buf = rx_buf_data,
        .len = sizeof(rx_buf_data),
    };
    struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };
    struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    // Start burst mode
    begin_ncs();

    if (send_byte(Motion_Burst)) {
        end_ncs();
        printk("Failed to send Motion_Burst command\n");
        return -EIO;
    }

    k_busy_wait(35);  // Datasheet requires at least 35us delay after Motion_Burst command


    // Transceive dummy bytes to get burst data
    int ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
    k_busy_wait(1);
    end_ncs();
    if (ret) return ret;
    /*
       Y+
    X-    X+
       Y-
    */


    *x = -(int16_t)((rx_buf_data[3] << 8) | rx_buf_data[2]);
    *y = (int16_t)((rx_buf_data[5] << 8) | rx_buf_data[4]);


    return 0;
}

int srom_download() {

    R_CHECK(write_register(Config2, 0x00));
    R_CHECK(write_register(SROM_Enable, 0x1D));

    k_msleep(10); // Wait for SROM to be enabled

    R_CHECK(write_register(SROM_Enable, 0x18));

    begin_ncs();
    send_byte(SROM_Load_Burst | 0x80); // Set MSB for SROM load burst
    k_busy_wait(15);

    for (int i = 0; i < firmware_length; i++) {
        send_byte(firmware_data[i]);
        k_busy_wait(15);
    }

    k_msleep(10); // Wait for SROM load to complete

    uint8_t srom_id = read_register(SROM_ID);
    printk("SROM ID: %02X\n", srom_id);

    R_CHECK(write_register(Config2, 0x00));

    R_CHECK(write_register(Config1, 0x15));

    end_ncs();

    k_msleep(10); // Wait for the sensor to stabilize after SROM load
    return 0; // Success
}

int init_pmw3389(void) {
    gpio_pin_configure_dt(&cs, GPIO_OUTPUT_HIGH); // Set to HIGH (inactive) by default
    // Reset the PMW3389
    end_ncs();
    begin_ncs();
    end_ncs();

    R_CHECK(write_register(Shutdown, 0xB6)); // Power up reset
    k_msleep(300); // Wait for the sensor to power up

    begin_ncs();
    k_busy_wait(40);
    end_ncs();
    k_busy_wait(40);

    R_CHECK(write_register(Power_Up_Reset, 0x5A));
    k_msleep(50); // Wait for the sensor to reset

    // read registers 0x02 to 0x06
    uint8_t reg_data[5];
    reg_data[0] = read_register(Motion);
    reg_data[1] = read_register(Delta_X_L);
    reg_data[2] = read_register(Delta_X_H);
    reg_data[3] = read_register(Delta_Y_L);
    reg_data[4] = read_register(Delta_Y_H);

    /* SROM DONWLOAD
       for some reason loading SROM makes it so that the x and y regs are 0
       always 0 so its not needed in this case, YMMV
    END SROM DOWNLOAD */

    reg_data[0] = read_register(Motion);
    reg_data[1] = read_register(Delta_X_L);
    reg_data[2] = read_register(Delta_X_H);
    reg_data[3] = read_register(Delta_Y_L);
    reg_data[4] = read_register(Delta_Y_H);
    printk("Initial Motion: %02X, Delta_X_L: %02X%02X, Delta_X_H: %02X%02X\n ",
           reg_data[0], reg_data[1], reg_data[2], reg_data[3], reg_data[4]);
    k_msleep(10);

    return 0; // Success
}
int convTwosComplement(int16_t val) {
    if (val & 0x8000) { // Check if the sign bit is set
        return val - 65536; // Convert to negative value
    }
    return val; // Positive value, no conversion needed
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
        k_msleep(10); // Polling interval
        fetch_burst_xy(&delta_x, &delta_y);
        if (delta_x|| delta_y) {
            printk("Motion detected: Delta X: %d, Delta Y: %d\n", delta_x, delta_y);
        }
    }

    return 0;
}