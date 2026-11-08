#include "vbat.h"
#include "../config.h"

void vbat_init(void) {
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_VBAT_ADC, ADC_11db);
    pinMode(PIN_VBAT_ADC, INPUT);
}

float vbat_read_volts(void) {
    uint32_t adc_mv = analogReadMilliVolts(PIN_VBAT_ADC);
    return ((float)adc_mv * 0.001f) * VBAT_DIVIDER_RATIO;
}
