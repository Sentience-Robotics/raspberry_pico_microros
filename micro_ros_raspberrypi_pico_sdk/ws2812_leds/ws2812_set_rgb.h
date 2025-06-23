#ifndef WS2812_SET_RGB_H
#define WS2812_SET_RGB_H

#include <stdint.h>
#include <stdbool.h>

#define NUM_LEDS 12

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} ws2812_rgb_t;

extern ws2812_rgb_t ws2812_led_buffer[NUM_LEDS];

void ws2812_set_rgb(uint32_t index, uint8_t r, uint8_t g, uint8_t b);
void ws2812_update(bool blocking);
void ws2812_clear(void);

#endif 
