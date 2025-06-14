
#include "config.h"

#include "pmw3389.h"

volatile short x, y;

int main(void) {
    init_pmw3389();

    while (1) {
        int ret = fetch_burst_xy(&x, &y);
        if (ret) {
            printk("Error fetching burst data: %d\n", ret);
            continue;
        }
        printk("X: %d, Y: %d\n", x, y);
        k_msleep(100); // Adjust the delay as needed
    }
}