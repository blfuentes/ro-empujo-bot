#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/adc_types_legacy.h>
#include <driver/adc.h>
#include <cmath>

extern "C" void app_main();

constexpr float referenceVoltage = 5.0; // Reference voltage of the ADC
constexpr int adcMaxValue = 4096; // Maximum ADC value for a 12-bit ADC

float read_adc(adc1_channel_t channel) {
    uint32_t adcValue = 0;
    for (uint16_t c = 0; c < 10; c++) {
        adcValue += adc1_get_raw(channel);
    }
    return adcValue / 10.0;
}

float get_voltage(float adcValue) {
    return (adcValue / adcMaxValue) * referenceVoltage;
}

float get_adc_value(float voltage) {
    return (voltage / referenceVoltage) * adcMaxValue;
}

float get_distance(float voltage) {
    // The equation of the line is y = a/x + b    
    // and two (x,y) points on the graph:
    // (30mm, 1.68V) and (150mm, 0.39V)
    const float a = 48.375;
    const float b = 0.0675;
    float dist = 0;
    if ( voltage > b ) {
        dist = a / (voltage - b);
    }
    // alternative formula: https://robojax.com/learn/arduino/?vid=robojax_SHARP_0A51SK_IR
    // float dist = 33.9 + -69.5 * (voltage) + 62.3 * pow(voltage, 2) + -25.4 * pow(voltage, 3) + 3.83 * pow(voltage, 4);

    return dist;
}

void app_main() {

    //ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);

    while (1)
    {   
        float adcValue = read_adc(ADC1_CHANNEL_3); //adc1_get_raw(ADC1_CHANNEL_6);
        float voltage = get_voltage(adcValue);
        float distance = get_distance(voltage);
        printf("Value: %f - voltage: %f - distance: %f\n", adcValue, voltage, distance);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }    
}