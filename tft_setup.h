#pragma once

// Self-contained copy of LilyGO TFT_eSPI Setup210, so Arduino IDE does not
// need to resolve a private User_Setups/ include path.
#define USER_SETUP_ID 210
#define ST7789_DRIVER
#define TFT_WIDTH 240
#define TFT_HEIGHT 320
#define TFT_RGB_ORDER TFT_RGB
#define INIT_SEQUENCE_2
#define TFT_MISO 38
#define TFT_MOSI 41
#define TFT_SCLK 40
#define TFT_CS 12
#define TFT_DC 11
#define TFT_RST -1
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
// This sketch uses only TFT_eSPI's built-in bitmap fonts. Do not enable
// LOAD_GFXFF/SMOOTH_FONT here: FreeFont state is unnecessary and can leave
// gfxFont uninitialized on some TFT_eSPI builds until setTextFont() runs.
#define SPI_FREQUENCY 40000000
#define SPI_READ_FREQUENCY 20000000
#define SPI_TOUCH_FREQUENCY 2500000

// TFT_eSPI 2.5.43 uses FSPI directly as an ESP32-S3 register-port index.
// With Arduino-ESP32 3.x, FSPI is the SPIClass ID 0, while the direct
// register map indexes SPI2 as 2. This avoids a write through address 0x10.
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #undef FSPI
  #define FSPI 2
#endif

#define USER_SETUP_LOADED
