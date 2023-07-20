add_library(pico_oled ssd1306.h font.h acme_5_outlines_font.h BMSPA_font.h bubblesstandard_font.h crackers_font.h image.h ssd1306.c)


target_link_libraries(pico_oled pico_stdlib hardware_i2c)

target_include_directories(pico_oled PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
