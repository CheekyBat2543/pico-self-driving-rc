add_library(haw_mpu6050 INTERFACE)

target_include_directories(haw_mpu6050
    INTERFACE
    ./include)

target_sources(haw_mpu6050
    INTERFACE
    MPU6050.c
)

target_link_libraries(haw_mpu6050
    INTERFACE
    pico_stdlib
    hardware_i2c 
)
