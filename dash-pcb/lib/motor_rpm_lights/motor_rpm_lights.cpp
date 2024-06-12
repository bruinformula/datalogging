#include "constants.h"
#include "motor_rpm_lights.h"

#include <Adafruit_NeoPixel.h>

static int rpm = 0;
static int rpm_red = 4000;
static int rpm_max = 5000;

static Adafruit_NeoPixel rpm_strip = Adafruit_NeoPixel(8, RPM_LIGHTS_PIN, NEO_GRB + NEO_KHZ800);
static uint32_t color_red = rpm_strip.Color(100, 0, 0);
static uint32_t color_green = rpm_strip.Color(0, 100, 0);
static uint32_t color_blue = rpm_strip.Color(0, 0, 100);
static uint32_t color_off = rpm_strip.Color(0, 0, 0);

void init_motor_rpm_lights() {
    rpm_strip.begin();
    for (int i = 0; i < 16; i++) {
        rpm_strip.setPixelColor(i, color_blue);
    }
    rpm_strip.show();
}

void loop_motor_rpm_lights() {
    double rpm_per_pixel = rpm_max / 16.0;
    for (int i = 0; i < 16; i++) {
        uint32_t color = i * rpm_per_pixel > rpm_red ? color_red : color_green;
        rpm_strip.setPixelColor(i, (i * rpm_per_pixel < rpm) ? color : color_off);
    }
    rpm_strip.show();
}

void update_motor_rpm(uint8_t* msg, uint8_t len) {
    if (len != 8) {
        return;
    }
    rpm = ((msg[3] << 8) | msg[4]) / 8.0;
}
