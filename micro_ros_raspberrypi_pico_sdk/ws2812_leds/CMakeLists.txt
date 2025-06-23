add_library(ws2812_leds STATIC ws2812.c ws2812_set_rgb.c)
target_include_directories(ws2812_leds PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
pico_generate_pio_header(ws2812_leds ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio) 
target_link_libraries(ws2812_leds pico_stdlib hardware_pio)
