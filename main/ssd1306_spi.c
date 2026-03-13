#include "ssd1306.h"

void spi_clock_speed(int speed) {}
void spi_master_init(SSD1306_t * dev, int16_t mosi, int16_t sclk, int16_t cs, int16_t dc, int16_t reset) {}
void spi_device_add(SSD1306_t * dev, int16_t cs, int16_t dc, int16_t reset) {}
bool spi_master_write_byte(spi_device_handle_t SPIHandle, const uint8_t* Data, size_t DataLength ) { return true; }
bool spi_master_write_commands(SSD1306_t * dev, const uint8_t * Commands, size_t DataLength ) { return true; }
bool spi_master_write_command(SSD1306_t * dev, uint8_t Command ) { return true; }
bool spi_master_write_data(SSD1306_t * dev, const uint8_t* Data, size_t DataLength ) { return true; }
void spi_init(SSD1306_t * dev, int width, int height) {}
void spi_display_image(SSD1306_t * dev, int page, int seg, const uint8_t * images, int width) {}
void spi_contrast(SSD1306_t * dev, int contrast) {}
void spi_hardware_scroll(SSD1306_t * dev, ssd1306_scroll_type_t scroll) {}