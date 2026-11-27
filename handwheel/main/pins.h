#pragma once

/*
 * 手轮板引脚。说明见 handwheel/引脚分配.md。
 * ESP32-S3 N16R8。屏幕已按此连接，其余按此表去接。
 */

/* 2 寸 ST7789，SPI。SCL/SDA 是模块丝印，不是 I2C。 */
#define PIN_LCD_SCLK 12
#define PIN_LCD_MOSI 11
#define PIN_LCD_DC   9
#define PIN_LCD_CS   10
#define PIN_LCD_BL   21
#define PIN_LCD_RST  14

/* 100 PPR 正交编码器。公共端接 GND。 */
#define PIN_ENC_A 4
#define PIN_ENC_B 5

/* 波段开关 A，一刀四段。公共端接 GND，选中为低。建议：X Y Z A。 */
#define PIN_SWA_1 6
#define PIN_SWA_2 7
#define PIN_SWA_3 8
#define PIN_SWA_4 13

/* 波段开关 B，一刀四段。公共端接 GND，选中为低。建议：0.001 0.01 0.1 1。 */
#define PIN_SWB_1 15
#define PIN_SWB_2 16
#define PIN_SWB_3 17
#define PIN_SWB_4 18

/* 六个按键。按下接 GND，内部上拉，按下为低。 */
#define PIN_KEY1 38
#define PIN_KEY2 39
#define PIN_KEY3 40
#define PIN_KEY4 41
#define PIN_KEY5 42
#define PIN_KEY6 47

/* 急停，常闭触点接 GND。闭合为低；按下或断线为高。 */
#define PIN_ESTOP 1

/* 备用：GPIO2。GPIO48 在 DevKitC-1 上是板载 RGB，不分配功能。 */
/* 保留：USB GPIO19/20，UART0 TX GPIO43 / RX GPIO44。 */
