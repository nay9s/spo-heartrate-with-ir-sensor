#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "ssd1306.h"

#define SDA_GPIO 21
#define SCL_GPIO 22

#define MAX30102_ADDR 0x57

static const char *TAG = "MAX30102";

SSD1306_t dev;

//////////////////////////////////////////////////
//// MAX30102 LOW LEVEL
//////////////////////////////////////////////////

void max30102_write(uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,(MAX30102_ADDR<<1)|I2C_MASTER_WRITE,true);
    i2c_master_write_byte(cmd,reg,true);
    i2c_master_write_byte(cmd,value,true);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_NUM_0,cmd,1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

uint8_t max30102_read(uint8_t reg)
{
    uint8_t data;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,(MAX30102_ADDR<<1)|I2C_MASTER_WRITE,true);
    i2c_master_write_byte(cmd,reg,true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,(MAX30102_ADDR<<1)|I2C_MASTER_READ,true);
    i2c_master_read_byte(cmd,&data,I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_NUM_0,cmd,1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return data;
}

//////////////////////////////////////////////////
//// MAX30102 INIT
//////////////////////////////////////////////////

void max30102_init()
{
    max30102_write(0x09,0x40); // reset
    vTaskDelay(pdMS_TO_TICKS(100));

    max30102_write(0x02,0xC0); // FIFO reset
    max30102_write(0x03,0x00);
    max30102_write(0x04,0x00);

    max30102_write(0x09,0x03); // SpO2 mode
    max30102_write(0x0A,0x27); // 100Hz sample
    max30102_write(0x0C,0x24); // LED current

    ESP_LOGI(TAG,"MAX30102 initialized");
}

//////////////////////////////////////////////////
//// READ SENSOR
//////////////////////////////////////////////////

void read_fifo(uint32_t *red, uint32_t *ir)
{
    uint8_t data[6];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,(MAX30102_ADDR<<1)|I2C_MASTER_WRITE,true);
    i2c_master_write_byte(cmd,0x07,true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,(MAX30102_ADDR<<1)|I2C_MASTER_READ,true);

    i2c_master_read(cmd,data,5,I2C_MASTER_ACK);
    i2c_master_read_byte(cmd,data+5,I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_NUM_0,cmd,1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    *red = ((uint32_t)data[0]<<16)|((uint32_t)data[1]<<8)|data[2];
    *ir  = ((uint32_t)data[3]<<16)|((uint32_t)data[4]<<8)|data[5];

    *red &= 0x3FFFF;
    *ir  &= 0x3FFFF;
}

//////////////////////////////////////////////////
//// SIMPLE SPO2 CALC
//////////////////////////////////////////////////

float calc_spo2(uint32_t red, uint32_t ir)
{
    if(ir==0) return 0;

    float ratio = (float)red / (float)ir;
    float spo2 = 110 - (25 * ratio);

    if(spo2 > 100) spo2 = 100;
    if(spo2 < 0) spo2 = 0;

    return spo2;
}

//////////////////////////////////////////////////
//// MAIN
//////////////////////////////////////////////////

void app_main()
{
    i2c_master_init(&dev,SDA_GPIO,SCL_GPIO,-1);

    ssd1306_init(&dev,128,64);
    ssd1306_clear_screen(&dev,false);

    ssd1306_display_text(&dev,0,"MAX30102 START",14,false);

    max30102_init();

    while(1)
    {
        uint32_t red,ir;

        read_fifo(&red,&ir);

        float spo2 = calc_spo2(red,ir);

        ESP_LOGI(TAG,"RED=%lu IR=%lu SpO2=%.1f",red,ir,spo2);

        char line1[20];
        char line2[20];

        sprintf(line1,"IR:%lu",ir);
        sprintf(line2,"SpO2:%.1f%%",spo2);

        ssd1306_clear_screen(&dev,false);

        ssd1306_display_text(&dev,0,"MAX30102",8,false);
        ssd1306_display_text(&dev,2,line1,strlen(line1),false);
        ssd1306_display_text(&dev,4,line2,strlen(line2),false);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}