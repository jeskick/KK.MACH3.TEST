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
} link_pkt_t;

static link_pkt_t link_rx;
static volatile bool link_rx_new;

static float coord_x, coord_y, coord_z;
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
}

static void draw_text(int x, int y, const char *text, uint16_t fg, uint16_t bg, int scale)
{
    lcd_flush();
    const int tw = (int)strlen(text) * 8 * scale;
    const int th = 8 * scale;
    if (tw <= 0 || x >= LCD_W || y >= LCD_H) {
        return;
    }
    int width = tw;
    if (x + width > LCD_W) {
        width = LCD_W - x;
    }
    for (int i = 0; i < width * th; i++) {
        line_buf[i] = bg;
    }
    for (int n = 0; text[n] != '\0'; n++) {
        unsigned char ch = (unsigned char)text[n];
        if (ch > 127) {
            ch = ' ';
        }
        const uint8_t *glyph = (const uint8_t *)font8x8_basic[ch];
        for (int row = 0; row < 8; row++) {
            uint8_t bits = glyph[row];
            for (int col = 0; col < 8; col++) {
                if ((bits & (0x01 << col)) == 0) {
                    continue;
                }
                for (int sy = 0; sy < scale; sy++) {
                    for (int sx = 0; sx < scale; sx++) {
                        int px = n * 8 * scale + col * scale + sx;
                        int py = row * scale + sy;
                        if (x + px < LCD_W && px < width) {
                            line_buf[py * width + px] = fg;
                        }
                    }
                }
            }
        }
    }
    esp_lcd_panel_draw_bitmap(panel, x, y, x + width, y + th, line_buf);
    lcd_flush();
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
        PIN_KEY1, PIN_KEY2, PIN_KEY3, PIN_KEY4, PIN_KEY5, PIN_KEY6,
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

static void link_send_ping(void)
{
    if (!link_ready) {
        return;
    }
    uint8_t pkt[4] = {0xA5, 0x5A, 2, link_seq++};
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
    const char *text = up ? "LINK OK" : "NO LINK";
    const int scale = 3;
    int tw = (int)strlen(text) * 8 * scale;
    int x = (LCD_W - tw) / 2;
    lcd_flush();
    for (int i = 0; i < LCD_W * 32; i++) {
        line_buf[i] = bg;
    }
    esp_lcd_panel_draw_bitmap(panel, 0, 0, LCD_W, 32, line_buf);
    lcd_flush();
    draw_text(x, 4, text, fg, bg, scale);
}

static void show(int y, const char *text, uint16_t fg)
{
    char padded[16];
    snprintf(padded, sizeof(padded), "%-15.15s", text);
    draw_text(0, y, padded, fg, 0x0000, 2);
}

void app_main(void)
{
    ESP_LOGI(TAG, "手轮板启动，点亮背光并初始化 ST7789");
    inputs_start();
    lcd_start();
    rgb_start();
    link_start();
    ESP_LOGI(TAG, "屏幕已初始化 %dx%d", LCD_W, LCD_H);

    char prev[14][20];
    memset(prev, 0, sizeof(prev));
    int shown_link = -1;
    uint32_t ticks = 0;

    while (1) {
        poll_encoder();
        ticks++;
        if (link_ready && (ticks % 25) == 0) {
            link_send_ping();
        }
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
                feed = pkt.feed;
                spindle = pkt.spindle;
            }
        }
        bool link_up = link_packets > 0 && (now_ms() - link_last_ms) < 1000;

        int swa = switch_pos(PIN_SWA_1, PIN_SWA_2, PIN_SWA_3, PIN_SWA_4);
        int swb = switch_pos(PIN_SWB_1, PIN_SWB_2, PIN_SWB_3, PIN_SWB_4);
        bool estop = gpio_get_level(PIN_ESTOP) != 0;
        char keys[7];
        const int key_pins[] = {PIN_KEY1, PIN_KEY2, PIN_KEY3, PIN_KEY4, PIN_KEY5, PIN_KEY6};
        for (int i = 0; i < 6; i++) {
            keys[i] = gpio_get_level(key_pins[i]) == 0 ? '1' + i : '-';
        }
        keys[6] = '\0';

        bool blink_on = ((ticks / 10) % 2) == 0;
        if (shown_link != (int)link_up) {
            status_bar(link_up);
            shown_link = (int)link_up;
            memset(prev, 0, sizeof(prev));
        }
        rgb_show(link_up, mach3_ok, blink_on);

        char row[12][20];
        if (link_rssi < -120) {
            snprintf(row[0], sizeof(row[0]), "PKT %lu", (unsigned long)link_packets);
        } else {
            snprintf(row[0], sizeof(row[0]), "P%lu R%d", (unsigned long)link_packets, link_rssi);
        }
        snprintf(row[1], sizeof(row[1]), "MACH3 %s", mach3_ok ? "YES" : "NO");
        snprintf(row[2], sizeof(row[2]), "X %+9.4f", coord_x);
        snprintf(row[3], sizeof(row[3]), "Y %+9.4f", coord_y);
        snprintf(row[4], sizeof(row[4]), "Z %+9.4f", coord_z);
        snprintf(row[5], sizeof(row[5]), "F %-5d S %-5d", feed, spindle);
        snprintf(row[6], sizeof(row[6]), "ESTOP %s", estop ? "OPEN" : "OK");
        snprintf(row[7], sizeof(row[7]), "ENC %ld", (long)encoder);
        snprintf(row[8], sizeof(row[8]), "SWA %d", swa);
        snprintf(row[9], sizeof(row[9]), "SWB %d", swb);
        snprintf(row[10], sizeof(row[10]), "KEY %s", keys);

        const uint16_t color[12] = {
            link_up ? 0x07E0 : 0xF800,
            mach3_ok ? 0x07E0 : 0xFFE0,
            0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
            estop ? 0xF800 : 0x07E0,
            0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
        };
        const int ypos[12] = {40, 64, 96, 120, 144, 168, 200, 224, 248, 272, 296};
        for (int i = 0; i < 11; i++) {
            if (strcmp(prev[i], row[i]) != 0) {
                show(ypos[i], row[i], color[i]);
                snprintf(prev[i], sizeof(prev[i]), "%s", row[i]);
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
