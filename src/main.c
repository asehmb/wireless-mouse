#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/kernel.h>
#include "srom.h"

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
    .pin = 10,
};

#define SPI_MODE_3 (SPI_MODE_CPOL | SPI_MODE_CPHA)


struct spi_config spi_cfg = {
    .frequency = 2000000, // 4 MHz
    .operation =  SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_3 | SPI_OP_MODE_MASTER,
    .slave = 0,
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
    gpio_pin_configure_dt(&cs, GPIO_OUTPUT_INACTIVE);
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

int recieve_byte(uint8_t *data) {
    uint8_t rx_buf[1] = {0x00}; // Initialize with 0
    struct spi_buf rx_buffer = {.buf = rx_buf, .len = sizeof(rx_buf)};
    struct spi_buf_set rx_buffer_set = {.buffers = &rx_buffer, .count = 1};

    int ret = spi_read(spi_dev, &spi_cfg, &rx_buffer_set);
    if (ret) {
        printk("SPI read error: %d\n", ret);
        return ret;
    }
    *data = rx_buf[0];
    return 0;
}

int read_register(uint8_t reg) {
    begin_ncs();
    send_byte(reg & 0x7F); // Clear MSB for read
    uint8_t data = 0;
    k_busy_wait(180);
    int ret = recieve_byte(&data);
    end_ncs();
    if (ret) {
        printk("SPI read error: %d\n", ret);
        return -1; // Error in SPI read
    }
    return data;
}

int write_register(uint8_t reg, uint8_t data) {
    begin_ncs();
    uint8_t tx_buf[2] = {reg | 0x80, data}; // Clear MSB for write
    struct spi_buf tx_buffer = {.buf = tx_buf, .len = sizeof(tx_buf)};
	struct spi_buf_set tx_buffer_set = {.buffers = &tx_buffer, .count = 1};

    int ret = spi_write(spi_dev, &spi_cfg, &tx_buffer_set);

    k_busy_wait(120);
    end_ncs();
    k_busy_wait(120);
    if (ret) {
        printk("SPI write error: %d\n", ret);
    }
    return ret;
}

int read_multiple(int addr[], uint8_t *data, int len) {
    begin_ncs();
    for (int i = 0; i < len; i++) {
        send_byte(addr[i] & 0x7F); // Clear MSB for read
        k_busy_wait(180);
        int ret = recieve_byte(&data[i]);
        if (ret) {
            printk("SPI read error at addr %d: %d\n", addr[i], ret);
            end_ncs();
            return ret; // Error in SPI read
        }
    }
    end_ncs();
    k_busy_wait(120); // Wait for 120us after read
    return 0; // Success
}

int read_burst(uint8_t *data, int len) {
    write_register(Motion_Burst, 0x00); // Clear the burst mode

    begin_ncs();
    k_busy_wait(120); // Wait for 120us before burst read
    send_byte(Motion_Burst | 0x80); // Set MSB for burst read
    k_busy_wait(35); // Wait for 160us

    struct spi_buf rx_buffer = {.buf = data, .len = len};
    struct spi_buf_set rx_buffer_set = {.buffers = &rx_buffer, .count = 1};

    int ret = spi_transceive(spi_dev, &spi_cfg, NULL, &rx_buffer_set);
    end_ncs();
    k_busy_wait(120); // Wait for 120us after burst read
    if (ret) {
        printk("SPI burst read error: %d\n", ret);
        return ret; // Error in SPI read
    }
    return 0; // Success
}

int fetch_burst_xy(int16_t *x, int16_t *y) {
    uint8_t data[12] = {0};
    int ret = read_burst(data, sizeof(data));
    if (ret) {
        printk("Error reading burst data: %d\n", ret);
        return ret;
    }

    // Combine the low and high bytes for X and Y
    if (data[0] & 0x01) { // Check if motion is detected
        motion_detected = true;
        *x = (int16_t)((data[3] << 8) | data[2]);
        *y = (int16_t)((data[5] << 8) | data[4]);
    } else {
        motion_detected = false;
        *x = 0;
        *y = 0;
    }

    return 0; // Success
}


int init_pmw3389(void) {
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
    int addr[] = {Motion, Delta_X_L, Delta_X_H, Delta_Y_L, Delta_Y_H};
    ret = read_multiple(addr, reg_data, 5);
    if (ret) {
        printk("Failed to read initial registers: %d\n", ret);
        return ret;
    }
    printk("Initial Motion: %02X, Delta_X: %02X%02X, Delta_Y: %02X%02X\n",
           reg_data[0], reg_data[1], reg_data[2], reg_data[3], reg_data[4]);

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
    k_busy_wait(120); // Wait for 120us before SROM load
    send_byte(SROM_Load_Burst | 0x80); // Set MSB for SROM load burst
    k_busy_wait(15);

    for (int i = 0; i < firmware_length; i++) {
        send_byte(firmware_data[i]);
        k_busy_wait(15); // Wait for 15us between bytes
    }

    uint8_t srom_id = read_register(SROM_ID);
    printk("SROM ID: %02X\n", srom_id);

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

    k_busy_wait(20);
    ret = write_register(0x3D, 0x80); 
    if (ret) {
        printk("Failed to set 0x3D: %d\n", ret);
        return ret;
    }
    k_busy_wait(20);

    printk("waiting for 0x3D");
    while (true) {
        uint8_t status = read_register(0x3D);
        if (status & 0xC0) { // Check if the bit is set
            printk("0x3D is set, exiting loop\n");
            break;
        }
        k_msleep(1); // Wait before checking again
    }

    k_busy_wait(20);
    ret = write_register(0x3D, 0x00); 
    if (ret) {
        printk("Failed to set 0x3D: %d\n", ret);
        return ret;
    }

    return 0; // Success
}

void update_pointer(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if (!init_complete) return;

    write_register(Motion, 0x01);
    read_register(Motion); // Clear the motion register

    delta_x = (int16_t)read_register(Delta_X_L);
    delta_y = (int16_t)read_register(Delta_Y_L);

    motion_detected = true;
}

int conv_twos_complement(int16_t value) {
    if (value & 0x80) { // Check if the sign bit is set
        value = -1 * ((value^0xFF) + 1); // Convert to negative value
    }
    return value; // Positive value
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

        curr_time = k_uptime_get_32();

        if (curr_time > last_motion_time) {
            fetch_burst_xy(&delta_x, &delta_y);
            if (delta_x != 0 || delta_y != 0) {
                printk("Motion detected: Delta X: %d, Delta Y: %d\n", delta_x, delta_y);
            } else {
                printk("No motion detected.\n");
            }
            last_motion_time = curr_time + 20; // Update the last motion time

        }
    }

    return 0;
}