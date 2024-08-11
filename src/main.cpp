#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_log.h"

#include <driver/adc_types_legacy.h>
#include <driver/adc.h>

#include "ssd1306.h"
#include "font8x8_basic.h"


#include <cmath>

#define tag "SSD1306"

extern "C" void app_main();

// IR sensor GP2Y0A51SK0F
constexpr adc1_channel_t adcChannel = ADC1_CHANNEL_3;   // ADC channel for the sensor (GPIO 4)
constexpr adc_atten_t adcAtten = ADC_ATTEN_DB_11;       // Attenuation for the ADC
constexpr adc_bits_width_t adcWidth = ADC_WIDTH_BIT_12; // ADC resolution
constexpr float referenceVoltage = 5.0;                 // Reference voltage of the ADC
constexpr int adcMaxValue = 4096;                       // Maximum ADC value for a 12-bit ADC

// OLED display
SSD1306_t dev;
int center, top, bottom;
char lineChar[20];


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

    // OLED
    ESP_LOGI(tag, "INTERFACE is i2c");
	ESP_LOGI(tag, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
	ESP_LOGI(tag, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
	ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
	i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

	ESP_LOGI(tag, "Panel is 128x32");
	ssd1306_init(&dev, 128, 32);
	ssd1306_clear_screen(&dev, false);
	
    ssd1306_contrast(&dev, 0xff);
	
	ssd1306_display_text(&dev, 0, "STARTING", 9, false);
	vTaskDelay(3000 / portTICK_PERIOD_MS);

    bool currentState = false;
    bool previousState = false;
    while (1)
    {   
        // ssd1306_clear_screen(&dev, false);
        float adcValue = read_adc(adcChannel); // adc1_get_raw(ADC1_CHANNEL_6);
        float voltage = get_voltage(adcValue);
        float distance = get_distance(voltage);
        printf("Value: %f - voltage: %f - distance: %f\n", adcValue, voltage, distance);

        char data_str[20];
        // data_str[0] = '\0';
        if (voltage > 0.5) {
           sprintf(data_str, "%.2f cm", distance); 
           currentState = true;
        } else {
            sprintf(data_str, "Out of range");
            currentState = false;
        }
        if (currentState != previousState) {
            ssd1306_clear_screen(&dev, false);
            previousState = currentState;
        }
        ssd1306_display_text(&dev, 0, data_str, 12, false);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
