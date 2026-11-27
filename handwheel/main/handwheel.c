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
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pins.h"

#include "../font8x8_basic.h"

#define LCD_W 240
#define LCD_H 320
#define TAG "handwheel"

static esp_lcd_panel_handle_t panel;
static uint16_t line_buf[LCD_W * 16];

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

static void lcd_fill(uint16_t color)
{
    for (int i = 0; i < LCD_W; i++) {
        line_buf[i] = color;
    }
    for (int y = 0; y < LCD_H; y++) {
        esp_lcd_panel_draw_bitmap(panel, 0, y, LCD_W, y + 1, line_buf);
    }
}

static void draw_text(int x, int y, const char *text, uint16_t fg, uint16_t bg)
{
    const int scale = 2;
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
                if ((bits & (0x80 >> col)) == 0) {
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

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = PIN_LCD_CS,
        .dc_gpio_num = PIN_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = 16 * 1000 * 1000,
        .trans_queue_depth = 1,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io));

    esp_lcd_panel_dev_config_t cfg = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &cfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, true, false));
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

static void show(int y, const char *text, uint16_t fg)
{
    char padded[16];
    snprintf(padded, sizeof(padded), "%-15.15s", text);
    draw_text(0, y, padded, fg, 0x0000);
}

void app_main(void)
{
    ESP_LOGI(TAG, "手轮板启动，点亮背光并初始化 ST7789");
    inputs_start();
    lcd_start();
    ESP_LOGI(TAG, "屏幕已初始化 %dx%d", LCD_W, LCD_H);

    char prev[14][20];
    memset(prev, 0, sizeof(prev));
    uint32_t ticks = 0;

    while (1) {
        poll_encoder();
        ticks++;
        if ((ticks % 10) != 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        int swa = switch_pos(PIN_SWA_1, PIN_SWA_2, PIN_SWA_3, PIN_SWA_4);
        int swb = switch_pos(PIN_SWB_1, PIN_SWB_2, PIN_SWB_3, PIN_SWB_4);
        bool estop = gpio_get_level(PIN_ESTOP) != 0;
        char keys[7];
        const int key_pins[] = {PIN_KEY1, PIN_KEY2, PIN_KEY3, PIN_KEY4, PIN_KEY5, PIN_KEY6};
        for (int i = 0; i < 6; i++) {
            keys[i] = gpio_get_level(key_pins[i]) == 0 ? '1' + i : '-';
        }
        keys[6] = '\0';

        char row[14][20];
        snprintf(row[0], sizeof(row[0]), "HANDWHEEL");
        snprintf(row[1], sizeof(row[1]), "X %+9.4f", coord_x);
        snprintf(row[2], sizeof(row[2]), "Y %+9.4f", coord_y);
        snprintf(row[3], sizeof(row[3]), "Z %+9.4f", coord_z);
        snprintf(row[4], sizeof(row[4]), "F %-5d S %-5d", feed, spindle);
        snprintf(row[5], sizeof(row[5]), "LINK NONE");
        snprintf(row[6], sizeof(row[6]), "ESTOP %s", estop ? "OPEN" : "OK");
        snprintf(row[7], sizeof(row[7]), "ENC %ld", (long)encoder);
        snprintf(row[8], sizeof(row[8]), "SWA %d", swa);
        snprintf(row[9], sizeof(row[9]), "SWB %d", swb);
        snprintf(row[10], sizeof(row[10]), "KEY %s", keys);
        snprintf(row[11], sizeof(row[11]), "RUN %lu", (unsigned long)(ticks / 10));

        const uint16_t color[14] = {
            0x07FF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFE0,
            estop ? 0xF800 : 0x07E0, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x07FF,
        };
        const int ypos[14] = {8, 36, 60, 84, 108, 140, 164, 188, 212, 236, 260, 292};
        for (int i = 0; i < 12; i++) {
            if (strcmp(prev[i], row[i]) != 0) {
                show(ypos[i], row[i], color[i]);
                snprintf(prev[i], sizeof(prev[i]), "%s", row[i]);
            }
        }

        if ((ticks % 50) == 0) {
            ESP_LOGI(TAG, "X %.4f Y %.4f Z %.4f F %d S %d  ENC %ld  SWA %d SWB %d  KEY %s  ESTOP %s",
                     coord_x, coord_y, coord_z, feed, spindle, (long)encoder, swa, swb, keys,
                     estop ? "开路" : "闭合");
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
