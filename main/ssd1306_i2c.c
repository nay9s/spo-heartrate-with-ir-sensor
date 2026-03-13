#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "ssd1306.h"

void i2c_master_init(SSD1306_t * dev, int16_t sda, int16_t scl, int16_t reset)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    dev->_address = I2C_ADDRESS;
    dev->_i2c_num = I2C_NUM_0;
    dev->_flip = false;
}

void i2c_init(SSD1306_t * dev, int width, int height)
{
    dev->_width = width;
    dev->_height = height;
    dev->_pages = height / 8;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_MUX_RATIO, true);
    i2c_master_write_byte(cmd, height - 1, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_OFFSET, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_START_LINE | 0x00, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP_1, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_PIN_MAP, true);
    if (height == 64) i2c_master_write_byte(cmd, 0x12, true);
    else i2c_master_write_byte(cmd, 0x02, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_CONTRAST, true);
    i2c_master_write_byte(cmd, 0xFF, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_RAM, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_VCOMH_DESELCT, true);
    i2c_master_write_byte(cmd, 0x40, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_MEMORY_ADDR_MODE, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_PAGE_ADDR_MODE, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
    i2c_master_write_byte(cmd, 0x14, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(dev->_i2c_num, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void i2c_display_image(SSD1306_t * dev, int page, int seg, const uint8_t * images, int width)
{
    if (page >= dev->_pages || seg >= dev->_width) return;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, 0x00 | (seg & 0x0F), true);
    i2c_master_write_byte(cmd, 0x10 | ((seg & 0xF0) >> 4), true);
    i2c_master_write_byte(cmd, 0xB0 | page, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(dev->_i2c_num, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
    i2c_master_write(cmd, (uint8_t *)images, width, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(dev->_i2c_num, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void i2c_contrast(SSD1306_t * dev, int contrast)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, OLED_CMD_SET_CONTRAST, true);
    i2c_master_write_byte(cmd, contrast, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(dev->_i2c_num, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void i2c_device_add(SSD1306_t * dev, i2c_port_t i2c_num, int16_t reset, uint16_t i2c_address) {}
void i2c_hardware_scroll(SSD1306_t * dev, ssd1306_scroll_type_t scroll) {}