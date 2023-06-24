add_executable(Folkrace_Car
        main.c
)

target_link_libraries(Folkrace_Car pico_stdlib pico_float freertos hardware_pwm pico_servo hardware_adc pico_ultrasonic)

pico_enable_stdio_usb(Folkrace_Car 1)
pico_enable_stdio_uart(Folkrace_Car 0)

pico_add_extra_outputs(Folkrace_Car)

