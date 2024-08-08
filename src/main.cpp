#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/adc_types_legacy.h>
#include <driver/adc.h>

#include "ssd1306.h"

#include <cmath>

extern "C" void app_main();

// IR sensor GP2Y0A51SK0F
constexpr adc1_channel_t adcChannel = ADC1_CHANNEL_3;   // ADC channel for the sensor (GPIO 4)
constexpr adc_atten_t adcAtten = ADC_ATTEN_DB_11;       // Attenuation for the ADC
constexpr adc_bits_width_t adcWidth = ADC_WIDTH_BIT_12; // ADC resolution
constexpr float referenceVoltage = 5.0;                 // Reference voltage of the ADC
constexpr int adcMaxValue = 4096;                       // Maximum ADC value for a 12-bit ADC

// OLED display
constexpr int I2C_SDA = 7;
constexpr int I2C_SCL = 6;

#define I2C_MASTER_SCL_IO I2C_SCL /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO I2C_SDA /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static ssd1306_handle_t ssd1306_dev = NULL;

float read_adc(adc1_channel_t channel)
{
    uint32_t adcValue = 0;
    for (uint16_t c = 0; c < 10; c++)
    {
        adcValue += adc1_get_raw(channel);
    }
    return adcValue / 10.0;
}

float get_voltage(float adcValue)
{
    return (adcValue / adcMaxValue) * referenceVoltage;
}

float get_adc_value(float voltage)
{
    return (voltage / referenceVoltage) * adcMaxValue;
}

float get_distance(float voltage)
{
    // The equation of the line is y = a/x + b
    // and two (x,y) points on the graph:
    // (30mm, 1.68V) and (150mm, 0.39V)
    const float a = 48.375;
    const float b = 0.0675;
    float dist = 0;
    if (voltage > b)
    {
        dist = a / (voltage - b);
    }
    // alternative formula: https://robojax.com/learn/arduino/?vid=robojax_SHARP_0A51SK_IR
    // float dist = 33.9 + -69.5 * (voltage) + 62.3 * pow(voltage, 2) + -25.4 * pow(voltage, 3) + 3.83 * pow(voltage, 4);

    return dist;
}

void app_main()
{

    // ADC
    adc1_config_width(adcWidth);
    adc1_config_channel_atten(adcChannel, adcAtten);

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    ssd1306_dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);

    ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_clear_screen(ssd1306_dev, 0x00);

    char data_str[20] = {0};
    sprintf(data_str, "STARTING!");
    ssd1306_draw_string(ssd1306_dev, 0, 0, (const uint8_t *)data_str, 16, 1);
    ssd1306_refresh_gram(ssd1306_dev);

    while (1)
    {   
        float adcValue = read_adc(adcChannel); // adc1_get_raw(ADC1_CHANNEL_6);
        float voltage = get_voltage(adcValue);
        float distance = get_distance(voltage);
        printf("Value: %f - voltage: %f - distance: %f\n", adcValue, voltage, distance);

        if (voltage > 0.5) {
           sprintf(data_str, "%.2f cm", distance); 
        } else {
            sprintf(data_str, "Out of range");
        }
        ssd1306_clear_screen(ssd1306_dev, 0x00);
        ssd1306_draw_string(ssd1306_dev, 0, 0, (const uint8_t *)data_str, 16, 1);
        esp_err_t error = ssd1306_refresh_gram(ssd1306_dev);
        if (error != ESP_OK)
        {
            printf("Error: %d\n", error);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}