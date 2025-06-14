/* adapted from mrjohnk's https://github.com/mrjohnk/PMW3389DM/blob/master/Arduino%20Examples/PMW3389DM-polling/PMW3389DM-polling.ino
               and dkao's https://github.com/dkao/Kensington_Expert_Mouse_PMW3389_Arduino*/
#include "../../config.h"
#include "pmw3389.h"

static const struct device *pmw3389_spi_dev = PMW3389_SPI_DEV;
static const struct gpio_dt_spec cs = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = PMW3389_CS_PIN,
};

#define SPI_MODE_3 (SPI_MODE_CPOL | SPI_MODE_CPHA)

static struct spi_config pmw3389_spi_cfg = {
    .frequency = 2000000, // 4 MHz
    .operation =  SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_3 | SPI_OP_MODE_MASTER,
    .slave = 10,
};

#define R_CHECK(func) do { \
    int ret = (func); \
    if (ret) { \
        printk("Error in %s: %d\n", #func, ret); \
        return ret; \
    } \
} while (0)

void begin_ncs(void) {
    gpio_pin_set_dt(&cs, 0); // drive ncs low
}
 void end_ncs(void) {
    gpio_pin_set_dt(&cs, 1); // drive ncs high
}

int send_byte(uint8_t data) {
    uint8_t tx_buf[1] = {data};
    struct spi_buf tx_buffer = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_buffer_set = {.buffers = &tx_buffer, .count = 1};

    int ret = spi_write(pmw3389_spi_dev, &pmw3389_spi_cfg, &tx_buffer_set);
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

    R_CHECK(spi_transceive(pmw3389_spi_dev, &pmw3389_spi_cfg, &tx_set, &rx_set));

    // Copy received data to output parameter

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

    int ret = spi_write(pmw3389_spi_dev, &pmw3389_spi_cfg, &tx_buf_set);
    if (ret) {
        printk("SPI write error: %d\n", ret);
    }

    k_busy_wait(20);
    end_ncs();
    k_busy_wait(120);
    return ret;
}

int read_burst(uint8_t *data, int len) {
    begin_ncs();
    send_byte(Motion_Burst);
    k_busy_wait(35);

    struct spi_buf rx = {.buf = data, .len = len};
    struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

    int ret = spi_read(pmw3389_spi_dev, &pmw3389_spi_cfg, &rx_set);
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

    k_busy_wait(35); 


    // Transceive dummy bytes to get burst data
    int ret = spi_transceive(pmw3389_spi_dev, &pmw3389_spi_cfg, &tx, &rx);
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

int srom_download(const uint8_t *firmware_data, size_t firmware_length) {

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

    k_msleep(10);

    return 0; // Success
}
