#pragma once

#include <driver/gpio.h>
#include <driver/spi_master.h>

// Board-specific TFT pinout, resolution, driver, and SPI clocks
#ifdef CONFIG_BOARD_TFT144

#define TFT_MOSI GPIO_NUM_4
#define TFT_SCLK GPIO_NUM_3
#define TFT_CS   GPIO_NUM_2
#define TFT_DC   GPIO_NUM_0
#define TFT_RST  GPIO_NUM_5
#define TFT_MISO GPIO_NUM_NC

#define LCD_H_RES 128
#define LCD_V_RES 128

// Use the current driver (ST7735) on TFT144
#define DISPLAY_DRIVER_ST7735 1

// Inversion supported on this panel
#define TFT_HAS_INVERSION 1

// SPI mode for this panel
#define TFT_SPI_MODE 0

#endif // CONFIG_BOARD_TFT144

#ifdef CONFIG_BOARD_TFT071

#define TFT_MOSI GPIO_NUM_7  // SDA
#define TFT_SCLK GPIO_NUM_6  // SCL
#define TFT_CS   GPIO_NUM_5
#define TFT_DC   GPIO_NUM_4
#define TFT_RST  GPIO_NUM_8
#define TFT_MISO GPIO_NUM_NC

// Panel resolution
#define LCD_H_RES 160
#define LCD_V_RES 160

// Use GC9D01 on TFT071
#define DISPLAY_DRIVER_GC9D01 1

// Backlight control (adjust to your board)
#define TFT_BL GPIO_NUM_9
#define TFT_BACKLIGHT_ON 1 // 1 = HIGH turns on; set to 0 if active low

// SPI clocks for this board (more conservative for GC9D01; raise after it works)
#define LCD_SPI_CLOCK_HZ       20000000   // Main
#define LCD_SPI_READ_CLOCK_HZ  20000000  // Read
#define TOUCH_SPI_CLOCK_HZ       250000  // Touch

// No inversion support on this panel
// RGB565 byte ordering and inversion
#define TFT_HAS_INVERSION 0
#ifndef TFT_SWAP_BYTES
#define TFT_SWAP_BYTES 0
#endif

// SPI mode for this panel (try 0 first; some GC9D01 boards require mode 3)
#ifndef TFT_SPI_MODE
#define TFT_SPI_MODE 0
#endif

//#undef TFT_SPI_MODE

#endif // CONFIG_BOARD_TFT071

// Common defaults and helpers
#ifndef LCD_SPI_CLOCK_HZ
#define LCD_SPI_CLOCK_HZ (40 * 1000 * 1000) // 40MHz default
#endif

// Public API of TFT module
#ifdef __cplusplus
extern "C" {
#endif

// Initialize SPI bus, panel IO, panel, and LVGL display/flush
void tft_init_all();

// Initialize LVGL filesystem driver for SPIFFS
void tft_init_lvgl_filesystem();

// LVGL tick callback (called from timer)
void tft_lvgl_tick_inc(uint32_t tick_period_ms);

// Run LVGL task loop - call this from main LVGL task
void tft_lvgl_run_task();

// Set image display duration in seconds
void tft_set_image_display_seconds(int seconds);

#ifdef __cplusplus
}
#endif

#ifndef LCD_SPI_READ_CLOCK_HZ
#define LCD_SPI_READ_CLOCK_HZ LCD_SPI_CLOCK_HZ
#endif

#ifndef TOUCH_SPI_CLOCK_HZ
#define TOUCH_SPI_CLOCK_HZ LCD_SPI_CLOCK_HZ
#endif

#ifndef LCD_HOST
#define LCD_HOST SPI2_HOST
#endif

// Driver-specific flush offsets
#if defined(DISPLAY_DRIVER_ST7735)
// ST7735 panel specifics (GREENTAB3)
#define TFT_SWAP_BYTES 1
#define TFT_OFFSET_X 2
#define TFT_OFFSET_Y 1
#elif defined(DISPLAY_DRIVER_GC9D01)
// GC9D01 typically no offsets for 160x160
#define TFT_OFFSET_X 0
#define TFT_OFFSET_Y 0
#define TFT_SWAP_BYTES 1  // Disable byte swapping for BGR MADCTL
#endif
