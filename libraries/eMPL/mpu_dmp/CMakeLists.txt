add_library(pico_mpu_dmp inv_mpu_dmp_motion_driver.h dmpKey.h dmpmap.h inv_mpu_dmp_motion_driver.c)

target_link_libraries(pico_mpu_dmp pico_stdlib hardware_i2c hardware_gpio pico_mpu6050)

target_include_directories(pico_mpu_dmp PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")