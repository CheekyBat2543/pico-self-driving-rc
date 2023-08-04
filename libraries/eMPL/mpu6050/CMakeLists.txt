add_library(pico_mpu6050 inv_mpu6050.h inv_mpu6050.c)
#add_library(pico_mpu6050 inv_mpu_dmp_motion_driver.h inv_mpu_dmp_motion_driver.c)

target_link_libraries(pico_mpu6050 pico_stdlib hardware_i2c hardware_gpio)

target_include_directories(pico_mpu6050 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")