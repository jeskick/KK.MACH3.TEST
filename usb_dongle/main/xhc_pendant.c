/*
 * Mach3 通信测试。原生 USB 模拟 XHC WHB04B-6（10CE:EB93）。
 * 日志走 UART0（GPIO43/44，板上 UART/COM 口），115200。
 * 原生 USB 口留给 Mach3。轴、倍率、拨轮和按键由手轮板发来。
 *
 *   idf.py set-target esp32s3
 *   idf.py -p COMx flash monitor
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "class/hid/hid_device.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "tinyusb.h"

#define USB_VID 0x10CE
#define USB_PID 0xEB93

#define AXIS_OFF 0x06
#define FEED_0_01 0x0E
#define REPORT_IN 0x04
#define REPORT_FEATURE 0x06
#define FRAME_LEN 32

static const char *TAG = "xhc";

/* Mach3 的 ShuttlePro 按集合打开设备：col01 读手轮，col02 写屏幕。
 * 屏幕数据是 HidD_SetFeature，每次 8 字节（报告号 0x06 + 7 字节）。
 * 两个顶层集合必须分开，Windows 才会给出 col01 / col02。 */
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
    0xC0,

    0x06, 0x00, 0xFF,
    0x09, 0x02,
    0xA1, 0x01,
    0x85, REPORT_FEATURE,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x07,
    0x09, 0x02,
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
static float last_mx, last_my, last_mz;
static uint16_t last_feed, last_spindle;
static bool have_last;

#define LINK_CHANNEL 6
#define PIN_RGB 48

static rmt_channel_handle_t rgb_chan;
static rmt_encoder_handle_t rgb_enc;

static void rgb_start(void)
{
    rmt_tx_channel_config_t tx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = PIN_RGB,
        .mem_block_symbols = 64,
        .resolution_hz = 10 * 1000 * 1000,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &rgb_chan));
    rmt_bytes_encoder_config_t enc = {
        .bit0 = {.level0 = 1, .duration0 = 4, .level1 = 0, .duration1 = 8},
        .bit1 = {.level0 = 1, .duration0 = 8, .level1 = 0, .duration1 = 4},
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc, &rgb_enc));
    ESP_ERROR_CHECK(rmt_enable(rgb_chan));
}

static void rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t grb[3] = {g, r, b};
    rmt_transmit_config_t tx = {0};
    rmt_tx_wait_all_done(rgb_chan, pdMS_TO_TICKS(20));
    rmt_transmit(rgb_chan, rgb_enc, grb, sizeof(grb), &tx);
}

static void rgb_show(bool up, bool mach3, bool blink_on)
{
    if (!up) {
        rgb_set(blink_on ? 48 : 0, 0, 0);
    } else if (!mach3) {
        rgb_set(48, 28, 0);
    } else {
        rgb_set(0, 48, 0);
    }
}
static bool link_ready;
static uint8_t link_seq;
static volatile uint32_t wheel_packets;
static volatile uint32_t wheel_last_ms;
static uint8_t in_axis = AXIS_OFF;
static uint8_t in_step = FEED_0_01;
static uint8_t in_btn;
static int16_t wheel_last;
static int32_t wheel_left;
static bool wheel_seen;
static portMUX_TYPE in_mux = portMUX_INITIALIZER_UNLOCKED;

typedef struct __attribute__((packed)) {
    uint8_t magic0;
    uint8_t magic1;
    uint8_t type;
    uint8_t seq;
    uint8_t has_coord;
    uint8_t flags;
    int32_t x;
    int32_t y;
    int32_t z;
    uint16_t feed;
    uint16_t spindle;
    int32_t mx;
    int32_t my;
    int32_t mz;
} link_pkt_t;

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void link_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    (void)info;
    if (data == NULL || len < 4) {
        return;
    }
    if (data[0] != 0xA5 || data[1] != 0x5A || data[2] != 2 || len < 9) {
        return;
    }
    int16_t wheel = (int16_t)(data[7] | (data[8] << 8));
    taskENTER_CRITICAL(&in_mux);
    in_axis = data[4];
    in_step = data[5];
    in_btn = data[6];
    if (!wheel_seen) {
        wheel_last = wheel;
        wheel_seen = true;
    } else {
        wheel_left += (int16_t)(wheel - wheel_last);
        wheel_last = wheel;
    }
    taskEXIT_CRITICAL(&in_mux);
    wheel_packets++;
    wheel_last_ms = now_ms();
}

static void link_send(void)
{
    if (!link_ready) {
        return;
    }
    link_pkt_t pkt = {
        .magic0 = 0xA5,
        .magic1 = 0x5A,
        .type = 1,
        .seq = link_seq++,
        .has_coord = have_last ? 1 : 0,
        .flags = 0,
        .x = (int32_t)(last_x * 10000.0f),
        .y = (int32_t)(last_y * 10000.0f),
        .z = (int32_t)(last_z * 10000.0f),
        .feed = last_feed,
        .spindle = last_spindle,
        .mx = (int32_t)(last_mx * 10000.0f),
        .my = (int32_t)(last_my * 10000.0f),
        .mz = (int32_t)(last_mz * 10000.0f),
    };
    uint8_t mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_send(mac, (const uint8_t *)&pkt, sizeof(pkt));
}

static void link_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs %s", esp_err_to_name(err));
        return;
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(LINK_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(link_recv));
    esp_now_peer_info_t peer = {0};
    memset(peer.peer_addr, 0xFF, 6);
    peer.channel = LINK_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    link_ready = true;
    ESP_LOGI(TAG, "link channel %d", LINK_CHANNEL);
}

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
    float x = coordinate(p + 4);
    float y = coordinate(p + 8);
    float z = coordinate(p + 12);
    uint16_t feed = (uint16_t)(p[16] | (p[17] << 8));
    uint16_t spindle = (uint16_t)(p[18] | (p[19] << 8));
    float mx = coordinate(p + 20);
    float my = coordinate(p + 24);
    float mz = coordinate(p + 28);
    frame_count++;

    bool changed = !have_last || x != last_x || y != last_y || z != last_z ||
                   mx != last_mx || my != last_my || mz != last_mz ||
                   feed != last_feed || spindle != last_spindle;
    if (!changed && (frame_count % 20) != 0) {
        return;
    }
    have_last = true;
    last_x = x;
    last_y = y;
    last_z = z;
    last_mx = mx;
    last_my = my;
    last_mz = mz;
    last_feed = feed;
    last_spindle = spindle;

    ESP_LOGI(TAG, "工件 X %8.4f Y %8.4f Z %8.4f  机械 X %8.4f Y %8.4f Z %8.4f  F %u S %u",
             x, y, z, mx, my, mz, feed, spindle);
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
        int inner = -1;
        for (int j = start + 2; j + 1 < start + FRAME_LEN; j++) {
            if (frame[j] == 0xFE && frame[j + 1] == 0xFD) {
                inner = j;
                break;
            }
        }
        if (inner >= 0) {
            memmove(frame, frame + inner, frame_len - inner);
            frame_len = (uint8_t)(frame_len - inner);
            continue;
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

static void send_pendant_report(void)
{
    if (!tud_mounted() || !tud_hid_ready()) {
        return;
    }
    uint8_t axis;
    uint8_t step;
    uint8_t btn;
    int8_t delta = 0;
    taskENTER_CRITICAL(&in_mux);
    axis = in_axis;
    step = in_step;
    btn = in_btn;
    if (wheel_left > 127) {
        delta = 127;
    } else if (wheel_left < -127) {
        delta = -127;
    } else {
        delta = (int8_t)wheel_left;
    }
    wheel_left -= delta;
    taskEXIT_CRITICAL(&in_mux);
    uint8_t random_byte = (uint8_t)(esp_random() & 0xFF);
    uint8_t packet[7] = {
        random_byte,
        btn,
        0x00,
        step,
        axis,
        (uint8_t)delta,
        (uint8_t)(random_byte & host_seed),
    };
    if (tud_hid_report(REPORT_IN, packet, sizeof(packet))) {
        reports_sent++;
    }
}

void app_main(void)
{
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
    rgb_start();
    link_start();
    ESP_LOGI(TAG, "USB %04X:%04X 已启动，等待 Mach3 回传坐标", USB_VID, USB_PID);

    TickType_t last_wait_log = xTaskGetTickCount();
    uint32_t loops = 0;

    while (1) {
        send_pendant_report();
        loops++;
        if ((loops % 10) == 0) {
            link_send();
            bool wheel_up = wheel_packets > 0 && (now_ms() - wheel_last_ms) < 1000;
            rgb_show(wheel_up, have_last, ((loops / 10) % 2) == 0);
        }
        if ((loops % 50) == 0) {
            bool wheel_up = wheel_packets > 0 && (now_ms() - wheel_last_ms) < 1000;
            ESP_LOGI(TAG, "%s pkts %lu", wheel_up ? "WHEEL LINK OK" : "WHEEL NO LINK",
                     (unsigned long)wheel_packets);
        }

        if (frame_count == 0 && (xTaskGetTickCount() - last_wait_log) > pdMS_TO_TICKS(2000)) {
            last_wait_log = xTaskGetTickCount();
            ESP_LOGI(TAG, "仍未收到显示数据。已发送 %lu 帧。确认 XHC 插件已启用并重启过 Mach3",
                     (unsigned long)reports_sent);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
