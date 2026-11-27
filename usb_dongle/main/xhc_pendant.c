/*
 * Mach3 通信测试。原生 USB 模拟 XHC WHB04B-6（10CE:EB93）。
 * 日志走 UART0（GPIO43/44，板上 UART/COM 口），115200。
 * 原生 USB 口留给 Mach3。手轮增量默认是 0，短按 BOOT（GPIO0）才发一格 X。
 *
 *   idf.py set-target esp32s3
 *   idf.py -p COMx flash monitor
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"

#define USB_VID 0x10CE
#define USB_PID 0xEB93

#define AXIS_X 0x11
#define FEED_0_01 0x0E
#define REPORT_IN 0x04
#define REPORT_FEATURE 0x06
#define FRAME_LEN 21

static const char *TAG = "xhc";

static const uint8_t hid_report_descriptor[] = {
    0x06, 0x00, 0xFF,
    0x09, 0x01,
    0xA1, 0x01,
    0x85, REPORT_IN,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x07,
    0x09, 0x01,
    0x81, 0x02,
    0x85, REPORT_FEATURE,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 64,
    0x09, 0x01,
    0xB1, 0x02,
    0xC0,
};

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

static const uint8_t hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 64, 4),
};

static const tusb_desc_device_t device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

static const char *hid_string_descriptor[] = {
    (char[]){0x09, 0x04},
    "KTURT.LTD",
    "WHB04B-6",
    "mach3-test",
    "XHC HID",
};

static uint8_t frame[64];
static uint8_t frame_len;
static uint8_t host_seed;
static uint32_t frame_count;
static uint32_t reports_sent;
static float last_x, last_y, last_z;
static uint16_t last_feed, last_spindle;
static bool have_last;

static float coordinate(const uint8_t *p)
{
    uint16_t integer = (uint16_t)(p[0] | (p[1] << 8));
    uint16_t fraction_bits = (uint16_t)(p[2] | (p[3] << 8));
    float value = integer + (fraction_bits & 0x7FFF) / 10000.0f;
    if (fraction_bits & 0x8000) {
        value = -value;
    }
    return value;
}

static void decode_frame(const uint8_t *p)
{
    host_seed = p[2];
    uint8_t flags = p[3];
    float x = coordinate(p + 4);
    float y = coordinate(p + 8);
    float z = coordinate(p + 12);
    uint16_t feed = (uint16_t)(p[16] | (p[17] << 8));
    uint16_t spindle = (uint16_t)(p[18] | (p[19] << 8));
    frame_count++;

    bool changed = !have_last || x != last_x || y != last_y || z != last_z || feed != last_feed || spindle != last_spindle;
    if (!changed && (frame_count % 20) != 0) {
        return;
    }
    have_last = true;
    last_x = x;
    last_y = y;
    last_z = z;
    last_feed = feed;
    last_spindle = spindle;

    ESP_LOGI(TAG, "Mach3 %s  X %8.4f  Y %8.4f  Z %8.4f  F %u  S %u  flags 0x%02X  frame %lu",
             (flags & 0x80) ? "工件坐标" : "机械坐标", x, y, z, feed, spindle, flags, (unsigned long)frame_count);
}

static void consume_host_bytes(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (frame_len >= sizeof(frame)) {
            memmove(frame, frame + 1, sizeof(frame) - 1);
            frame_len = sizeof(frame) - 1;
        }
        frame[frame_len++] = data[i];
    }

    while (frame_len >= FRAME_LEN) {
        int start = -1;
        for (int i = 0; i < frame_len - 1; i++) {
            if (frame[i] == 0xFE && frame[i + 1] == 0xFD) {
                start = i;
                break;
            }
        }
        if (start < 0) {
            frame[0] = frame[frame_len - 1];
            frame_len = 1;
            return;
        }
        if (start + FRAME_LEN > frame_len) {
            if (start > 0) {
                memmove(frame, frame + start, frame_len - start);
                frame_len = (uint8_t)(frame_len - start);
            }
            return;
        }
        decode_frame(frame + start);
        uint8_t rest = (uint8_t)(frame_len - (start + FRAME_LEN));
        memmove(frame, frame + start + FRAME_LEN, rest);
        frame_len = rest;
    }
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    if (reqlen == 0) {
        return 0;
    }
    memset(buffer, 0, reqlen);
    return reqlen;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance;
    (void)report_type;
    if (buffer == NULL || bufsize == 0) {
        return;
    }
    if (report_id == 0) {
        report_id = buffer[0];
        buffer++;
        bufsize--;
    }
    if (report_id != REPORT_FEATURE) {
        ESP_LOGI(TAG, "忽略报告 id=%u len=%u", report_id, bufsize);
        return;
    }
    if (bufsize > 0 && buffer[0] == REPORT_FEATURE) {
        buffer++;
        bufsize--;
    }
    consume_host_bytes(buffer, bufsize);
}

static void send_pendant_report(int8_t delta)
{
    if (!tud_mounted() || !tud_hid_ready()) {
        return;
    }
    uint8_t random_byte = (uint8_t)(esp_random() & 0xFF);
    uint8_t packet[7] = {
        random_byte,
        0x00,
        0x00,
        FEED_0_01,
        AXIS_X,
        (uint8_t)delta,
        (uint8_t)(random_byte & host_seed),
    };
    if (tud_hid_report(REPORT_IN, packet, sizeof(packet))) {
        reports_sent++;
    }
}

void app_main(void)
{
    const gpio_config_t boot = {
        .pin_bit_mask = BIT64(GPIO_NUM_0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&boot));

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &device_descriptor,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = hid_configuration_descriptor,
        .hs_configuration_descriptor = hid_configuration_descriptor,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = hid_configuration_descriptor,
#endif
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB %04X:%04X 已启动，等待 Mach3 回传坐标", USB_VID, USB_PID);

    bool boot_was_down = false;
    TickType_t last_wait_log = xTaskGetTickCount();

    while (1) {
        int8_t delta = 0;
        bool boot_down = gpio_get_level(GPIO_NUM_0) == 0;
        if (boot_down && !boot_was_down) {
            delta = 1;
            ESP_LOGW(TAG, "BOOT 按下，发送 X 轴 +1 格");
        }
        boot_was_down = boot_down;

        send_pendant_report(delta);

        if (frame_count == 0 && (xTaskGetTickCount() - last_wait_log) > pdMS_TO_TICKS(2000)) {
            last_wait_log = xTaskGetTickCount();
            ESP_LOGI(TAG, "仍未收到显示数据。已发送 %lu 帧。确认 XHC 插件已启用并重启过 Mach3",
                     (unsigned long)reports_sent);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
