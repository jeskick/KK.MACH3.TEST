/*
 * 手轮板。ESP32-S3 N16R8，2 寸 ST7789。
 * 屏幕引脚见 pins.h。编码器、波段开关、按键、急停还没接时，
 * 屏幕显示开路状态，坐标保持 0，并标明还没有和 USB 板通信。
 *
 *   pio run -t upload
 *   pio device monitor
 */

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "pins.h"

#include "../font8x8_basic.h"

#define LCD_W 240
#define LCD_H 320
#define TAG "handwheel"

#define LINK_CHANNEL 6

static esp_lcd_panel_handle_t panel;
static esp_lcd_panel_io_handle_t lcd_io;
static uint16_t line_buf[LCD_W * 32];
static bool link_ready;
static uint8_t link_seq;
static int8_t link_rssi = -127;
static uint32_t link_packets;
static uint32_t link_last_ms;
static bool mach3_ok;
static portMUX_TYPE link_mux = portMUX_INITIALIZER_UNLOCKED;

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

static link_pkt_t link_rx;
static volatile bool link_rx_new;

static float coord_x, coord_y, coord_z;
static float coord_mx, coord_my, coord_mz;
static int feed, spindle;
static int32_t encoder;
static uint8_t enc_prev;
static const int8_t enc_table[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
};

static void lcd_flush(void)
{
    if (lcd_io != NULL) {
        esp_lcd_panel_io_tx_param(lcd_io, 0x00, NULL, 0);
    }
}

static void lcd_fill(uint16_t color)
{
    for (int i = 0; i < LCD_W; i++) {
        line_buf[i] = color;
    }
    for (int y = 0; y < LCD_H; y++) {
        esp_lcd_panel_draw_bitmap(panel, 0, y, LCD_W, y + 1, line_buf);
    }
    lcd_flush();
}

static void fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x >= LCD_W || y >= LCD_H || w <= 0 || h <= 0) {
        return;
    }
    if (x + w > LCD_W) {
        w = LCD_W - x;
    }
    if (y + h > LCD_H) {
        h = LCD_H - y;
    }
    for (int i = 0; i < w; i++) {
        line_buf[i] = color;
    }
    lcd_flush();
    for (int row = 0; row < h; row++) {
        esp_lcd_panel_draw_bitmap(panel, x, y + row, x + w, y + row + 1, line_buf);
    }
    lcd_flush();
}

static void fit_text(char *dst, int cols, const char *src, bool center)
{
    int n = 0;
    while (src[n] != '\0' && n < cols) {
        n++;
    }
    int left = center ? (cols - n) / 2 : 0;
    for (int i = 0; i < cols; i++) {
        dst[i] = ' ';
    }
    for (int i = 0; i < n; i++) {
        dst[left + i] = src[i];
    }
    dst[cols] = '\0';
}

static void draw_glyphs(int x, int y, const char *text, int cols, int cw, int ch,
                        uint16_t fg, uint16_t bg)
{
    const int width = cols * cw;
    if (cols <= 0 || cw <= 0 || ch <= 0 || ch > 32 || width > LCD_W) {
        return;
    }
    lcd_flush();
    for (int i = 0; i < width * ch; i++) {
        line_buf[i] = bg;
    }
    for (int n = 0; n < cols; n++) {
        unsigned char chv = (unsigned char)text[n];
        if (chv > 127) {
            chv = ' ';
        }
        const uint8_t *glyph = (const uint8_t *)font8x8_basic[chv];
        for (int row = 0; row < 8; row++) {
            uint8_t bits = glyph[row];
            int y0 = row * ch / 8;
            int y1 = (row + 1) * ch / 8;
            for (int col = 0; col < 8; col++) {
                if ((bits & (0x01 << col)) == 0) {
                    continue;
                }
                int x0 = col * cw / 8;
                int x1 = (col + 1) * cw / 8;
                for (int py = y0; py < y1; py++) {
                    for (int px = x0; px < x1; px++) {
                        line_buf[py * width + n * cw + px] = fg;
                    }
                }
            }
        }
    }
    esp_lcd_panel_draw_bitmap(panel, x, y, x + width, y + ch, line_buf);
    lcd_flush();
}

static void draw_band(int y, int h, const char *text, int cols, int cw, int ch,
                      uint16_t fg, uint16_t bg, bool center)
{
    char fitted[16];
    if (cols > 15) {
        cols = 15;
    }
    fit_text(fitted, cols, text, center);
    fill_rect(0, y, LCD_W, h, bg);
    int width = cols * cw;
    int x = (LCD_W - width) / 2;
    int ty = y + (h - ch) / 2;
    if (ty < y) {
        ty = y;
    }
    draw_glyphs(x, ty, fitted, cols, cw, ch, fg, bg);
}

static void lcd_start(void)
{
    gpio_config_t bl = {
        .pin_bit_mask = 1ULL << PIN_LCD_BL,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&bl));
    gpio_set_level(PIN_LCD_BL, 1);

    spi_bus_config_t bus = {
        .sclk_io_num = PIN_LCD_SCLK,
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(line_buf),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED));

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = PIN_LCD_CS,
        .dc_gpio_num = PIN_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = 16 * 1000 * 1000,
        .trans_queue_depth = 4,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &lcd_io));

    esp_lcd_panel_dev_config_t cfg = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io, &cfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
    lcd_fill(0x0000);
}

static int switch_pos(int p1, int p2, int p3, int p4)
{
    if (gpio_get_level(p1) == 0) {
        return 1;
    }
    if (gpio_get_level(p2) == 0) {
        return 2;
    }
    if (gpio_get_level(p3) == 0) {
        return 3;
    }
    if (gpio_get_level(p4) == 0) {
        return 4;
    }
    return 0;
}

static void poll_encoder(void)
{
    uint8_t now = (uint8_t)((gpio_get_level(PIN_ENC_A) << 1) | gpio_get_level(PIN_ENC_B));
    encoder += enc_table[(enc_prev << 2) | now];
    enc_prev = now;
}

static void inputs_start(void)
{
    const int pins[] = {
        PIN_ENC_A, PIN_ENC_B,
        PIN_SWA_1, PIN_SWA_2, PIN_SWA_3, PIN_SWA_4,
        PIN_SWB_1, PIN_SWB_2, PIN_SWB_3, PIN_SWB_4,
        PIN_KEY1, PIN_KEY2, PIN_KEY3, PIN_KEY4, PIN_KEY5, PIN_KEY6, PIN_KEY7, PIN_KEY8,
        PIN_ESTOP,
    };
    uint64_t mask = 0;
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        mask |= 1ULL << pins[i];
    }
    gpio_config_t in = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in));
    enc_prev = (uint8_t)((gpio_get_level(PIN_ENC_A) << 1) | gpio_get_level(PIN_ENC_B));
}

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void link_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (data == NULL || len < (int)sizeof(link_pkt_t)) {
        return;
    }
    if (data[0] != 0xA5 || data[1] != 0x5A || data[2] != 1) {
        return;
    }
    link_pkt_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    taskENTER_CRITICAL(&link_mux);
    link_rx = pkt;
    link_rx_new = true;
    if (info != NULL && info->rx_ctrl != NULL) {
        link_rssi = info->rx_ctrl->rssi;
    }
    taskEXIT_CRITICAL(&link_mux);
}

static uint8_t ctrl_axis = 0x06;
static uint8_t ctrl_step = 0x0D;
static uint8_t ctrl_btn;
static const char *ctrl_axis_name = "OFF";
static const char *ctrl_step_name = "0.001";
static const char *ctrl_btn_name = "----";

static void read_controls(void)
{
    int axis = switch_pos(PIN_SWA_1, PIN_SWA_2, PIN_SWA_3, PIN_SWA_4);
    static const uint8_t axis_code[] = {0x06, 0x06, 0x11, 0x12, 0x13};
    static const char *axis_name[] = {"----", "OFF", "X", "Y", "Z"};
    ctrl_axis = axis_code[axis];
    ctrl_axis_name = axis_name[axis];

    int step = switch_pos(PIN_SWB_1, PIN_SWB_2, PIN_SWB_3, PIN_SWB_4);
    static const uint8_t step_code[] = {0x0E, 0x0D, 0x0E, 0x0F, 0x10};
    static const char *step_name[] = {"----", "0.001", "0.01", "0.1", "1"};
    ctrl_step = step_code[step];
    ctrl_step_name = step_name[step];

    static const struct {
        int pin;
        uint8_t code;
        const char *name;
    } keys[] = {
        {PIN_KEY1, 0x01, "RESET"},
        {PIN_KEY2, 0x02, "STOP"},
        {PIN_KEY3, 0x03, "START"},
        {PIN_KEY4, 0x04, "FEED+"},
        {PIN_KEY5, 0x05, "FEED-"},
        {PIN_KEY6, 0x06, "SPD+"},
        {PIN_KEY7, 0x07, "SPD-"},
        {PIN_KEY8, 0x08, "HOME"},
    };
    ctrl_btn = 0;
    ctrl_btn_name = "----";
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
        if (gpio_get_level(keys[i].pin) == 0) {
            ctrl_btn = keys[i].code;
            ctrl_btn_name = keys[i].name;
            break;
        }
    }
}

static void link_send_controls(void)
{
    if (!link_ready) {
        return;
    }
    int16_t detents = (int16_t)(encoder / 4);
    uint8_t pkt[9] = {
        0xA5,
        0x5A,
        2,
        link_seq++,
        ctrl_axis,
        ctrl_step,
        ctrl_btn,
        (uint8_t)detents,
        (uint8_t)(detents >> 8),
    };
    uint8_t mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_send(mac, pkt, sizeof(pkt));
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

static void status_bar(bool up)
{
    uint16_t bg = up ? 0x07E0 : 0xF800;
    uint16_t fg = up ? 0x0000 : 0xFFFF;
    draw_band(0, 20, up ? "LINK OK" : "NO LINK", 7, 24, 16, fg, bg, true);
}

static void fmt_coord(char *dst, size_t n, char axis, float v)
{
    int len = snprintf(dst, n, "%c%+14.4f", axis, v);
    if (len < 0 || len > 15) {
        snprintf(dst, n, "%c%+14.0f", axis, v);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "手轮板启动，点亮背光并初始化 ST7789");
    inputs_start();
    lcd_start();
    rgb_start();
    link_start();
    ESP_LOGI(TAG, "屏幕已初始化 %dx%d", LCD_W, LCD_H);

    char prev[11][24];
    memset(prev, 0, sizeof(prev));
    int shown_link = -1;
    uint32_t ticks = 0;

    while (1) {
        poll_encoder();
        read_controls();
        link_send_controls();
        ticks++;
        if ((ticks % 10) != 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        link_pkt_t pkt;
        bool got = false;
        taskENTER_CRITICAL(&link_mux);
        if (link_rx_new) {
            pkt = link_rx;
            link_rx_new = false;
            got = true;
        }
        taskEXIT_CRITICAL(&link_mux);
        if (got) {
            link_packets++;
            link_last_ms = now_ms();
            mach3_ok = pkt.has_coord != 0;
            if (mach3_ok) {
                coord_x = pkt.x / 10000.0f;
                coord_y = pkt.y / 10000.0f;
                coord_z = pkt.z / 10000.0f;
                coord_mx = pkt.mx / 10000.0f;
                coord_my = pkt.my / 10000.0f;
                coord_mz = pkt.mz / 10000.0f;
                feed = pkt.feed;
                spindle = pkt.spindle;
            }
        }
        bool link_up = link_packets > 0 && (now_ms() - link_last_ms) < 1000;

        bool estop = gpio_get_level(PIN_ESTOP) != 0;
        bool blink_on = ((ticks / 10) % 2) == 0;
        if (shown_link != (int)link_up) {
            status_bar(link_up);
            shown_link = (int)link_up;
            memset(prev, 0, sizeof(prev));
        }
        rgb_show(link_up, mach3_ok, blink_on);

        char row[11][24];
        fmt_coord(row[0], sizeof(row[0]), 'X', coord_x);
        fmt_coord(row[1], sizeof(row[1]), 'X', coord_mx);
        fmt_coord(row[2], sizeof(row[2]), 'Y', coord_y);
        fmt_coord(row[3], sizeof(row[3]), 'Y', coord_my);
        fmt_coord(row[4], sizeof(row[4]), 'Z', coord_z);
        fmt_coord(row[5], sizeof(row[5]), 'Z', coord_mz);
        snprintf(row[6], sizeof(row[6]), "F%6d S%6d", feed, spindle);
        snprintf(row[7], sizeof(row[7]), "AXIS %s", ctrl_axis_name);
        snprintf(row[8], sizeof(row[8]), "STEP %s", ctrl_step_name);
        snprintf(row[9], sizeof(row[9]), "BTN %s", ctrl_btn_name);
        snprintf(row[10], sizeof(row[10]), "ESTOP %s", estop ? "OPEN" : "OK");

        const uint16_t color[11] = {
            0x07E0, 0xF800, 0x07E0, 0xF800, 0x07E0, 0xF800,
            0xFFFF,
            0x07FF, 0x07FF, ctrl_btn ? 0xFFE0 : 0xFFFF,
            estop ? 0xF800 : 0x07E0,
        };
        /* 20 + (28+18)*3 + 32*4 + 34 = 320. 工件绿、大字；机械红、小一号。 */
        static const int band_y[11] = {20, 48, 66, 94, 112, 140, 158, 190, 222, 254, 286};
        static const int band_h[11] = {28, 18, 28, 18, 28, 18, 32, 32, 32, 32, 34};
        static const int band_cols[11] = {15, 15, 15, 15, 15, 15, 15, 10, 10, 10, 10};
        static const int band_cw[11] = {16, 16, 16, 16, 16, 16, 16, 24, 24, 24, 24};
        static const int band_ch[11] = {24, 16, 24, 16, 24, 16, 16, 24, 24, 24, 24};
        static const bool band_center[11] = {
            false, false, false, false, false, false, false, true, true, true, true,
        };
        static uint16_t prev_fg[11];
        for (int i = 0; i < 11; i++) {
            if (strcmp(prev[i], row[i]) != 0 || prev_fg[i] != color[i]) {
                draw_band(band_y[i], band_h[i], row[i], band_cols[i], band_cw[i], band_ch[i],
                          color[i], 0x0000, band_center[i]);
                snprintf(prev[i], sizeof(prev[i]), "%s", row[i]);
                prev_fg[i] = color[i];
            }
        }

        if ((ticks % 50) == 0) {
            ESP_LOGI(TAG, "%s pkts %lu rssi %d mach3 %s  X %.4f Y %.4f Z %.4f",
                     link_up ? "LINK OK" : "NO LINK", (unsigned long)link_packets, link_rssi,
                     mach3_ok ? "YES" : "NO", coord_x, coord_y, coord_z);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
