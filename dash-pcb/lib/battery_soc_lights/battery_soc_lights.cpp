#include "constants.h"
#include "battery_soc_lights.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

static Adafruit_NeoPixel soc_strip = Adafruit_NeoPixel(8, SOC_LIGHTS_PIN, NEO_GRB + NEO_KHZ800);
static uint32_t color_red = soc_strip.Color(100, 0, 0);
static uint32_t color_yellow = soc_strip.Color(100, 50, 0);
static uint32_t color_green = soc_strip.Color(0, 100, 0);
static uint32_t color_blue = soc_strip.Color(0, 0, 100);
static uint32_t color_off = soc_strip.Color(0, 0, 0);

void init_battery_soc_lights() {
    soc_strip.begin();
    for (int i = 0; i < 16; i++) {
        soc_strip.setPixelColor(i, color_blue);
    }
    soc_strip.show();
}

void loop_battery_soc_lights() {
    float state_of_charge = analogRead(SOC_AIN_PIN) / 1023;
    uint32_t color = color_off;
    if (state_of_charge < 0.15) {
        color = color_red;
    } else if (state_of_charge < 0.4) {
        color = color_yellow;
    } else {
        color = color_green;
    }
    for (int i = 0; i < 16; i++) {
        soc_strip.setPixelColor(i, i < (state_of_charge * 16 + 0.5) ? color : color_off);
    }
    soc_strip.show();
}
