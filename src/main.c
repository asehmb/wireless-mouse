
#include "config.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/class/usb_hid.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const uint8_t hid_report_desc[] = HID_MOUSE_REPORT_DESC(2);

enum mouse_report_idx {
	MOUSE_BTN_REPORT_ID = 0,
	MOUSE_X_REPORT_ID = 1,
	MOUSE_Y_REPORT_ID = 2,
	MOUSE_WHEEL_REPORT_ID = 3,
	MOUSE_REPORT_COUNT = 4,
};

const struct device *hid_dev;

static void status_cb(enum usb_dc_status_code status, const uint8_t *param) {
    LOG_INF("USB status: %d", status);
}
volatile short x = 0, y = 0;

int main(void) {
    int ret;

    hid_dev = device_get_binding("HID_0");


    if (!device_is_ready(hid_dev)) {
        LOG_ERR("HID device not ready");
        return -1;
    }

    ret = usb_enable(status_cb);
    if (ret) {
        LOG_ERR("Failed to enable USB");
        return -1;
    }

    usb_hid_register_device(hid_dev, hid_report_desc,
                                  sizeof(hid_report_desc), NULL);


    ret = usb_hid_init(hid_dev);
    if (ret) {
        LOG_ERR("Failed to initialize HID");
        return -1;
    }
    ret = init_pmw3389();

    uint8_t report[4] = {0};

    ret = hid_int_ep_write(hid_dev, report, sizeof(report), NULL);
    if (ret) {
        LOG_ERR("Failed to send mouse report: %d", ret);
    }

    while (1) {
        fetch_burst_xy(&x,  &y);
        report[MOUSE_X_REPORT_ID] = x;
        report[MOUSE_Y_REPORT_ID] = -y;
        hid_int_ep_write(hid_dev, report, sizeof(report), NULL);

        k_msleep(10);
    }

    return 0;
}