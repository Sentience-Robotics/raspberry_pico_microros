#include "ws2812_set_rgb.h"
#include "ws2812.h"

ws2812_rgb_t ws2812_led_buffer[NUM_LEDS];

void ws2812_set_rgb(uint32_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < NUM_LEDS) {
        ws2812_led_buffer[index].r = r;
        ws2812_led_buffer[index].g = g;
        ws2812_led_buffer[index].b = b;
    }
}

void ws2812_update(bool blocking) {
    for (uint32_t i = 0; i < NUM_LEDS; ++i) {
        put_pixel_rgb(ws2812_led_buffer[i].r, ws2812_led_buffer[i].g, ws2812_led_buffer[i].b);
    }
    // Optionally add a delay or blocking logic if needed
}

void ws2812_clear(void) {
    for (uint32_t i = 0; i < NUM_LEDS; ++i) {
        ws2812_set_rgb(i, 0, 0, 0);
    }
    ws2812_update(true);
} 
