#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/float.h"

#include "servo.h"
#include "ultrasonic.h"
#include "MPU6050.h"
#include "dht.h"

#include "ssd1306.h"
#include "image.h"

//  #define FRONT_SENSOR_DEMO 1
//  #define SIDE_SENSOR_DEMO 1

// Trigonometric Macros
/*------------------------------------------------------------*/
#define COS30                               0.8660254f
#define COS45                               0.70710678118f
#define COS60                               0.5f
#define SIN30                               0.8660254f
#define SIN45                               0.5f
#define SIN60                               0.8660254f
/*------------------------------------------------------------*/


// Pin Connections
/*------------------------------------------------------------*/
#define LED_PIN              25

#define OLED_SDA_PIN         12
#define OLED_SCL_PIN         13

#define MPU6050_SDA_PIN      12
#define MPU6050_SCL_PIN      13

#define LEFT_IR_SENSOR_PIN   10
#define RIGHT_IR_SENSOR_PIN  11

#define DHT_PIN              4

#define RIGHT_TRIG_PIN       16
#define RIGHT_ECHO_PIN       17
#define LEFT_TRIG_PIN        19
#define LEFT_ECHO_PIN        18
#define FRONT_TRIG_PIN       21 
#define FRONT_ECHO_PIN       20

#define MOTOR_ESC_PIN        5
#define SERVO_PIN            2

/*------------------------------------------------------------*/


// Config Values
/*------------------------------------------------------------*/
#define ARRAY_SIZE                          5

#define MIN_FRONT_DISTANCE                  28   // CM
#define MAX_FRONT_DISTANCE                  450  // CM
#define MAX_FRONT_DISTANCE_TO_TURN          80   // CM
#define MIN_FRONT_DISTANCE_TO_ACCELERATE    400  // CM

#define MOTOR_DISTANCE_TO_BRAKE             50   // CM
#define MOTOR_BRAKE_PERIOD                  500  // Millisecond

#define MAX_SIDE_SENSOR_DISTANCE            200  // CM
#define MIN_SIDE_DISTANCE_TO_TURN           5   // CM
#define MAX_SIDE_DISTANCE_TO_TURN           60  // CM

#define MIN_SERVO_MICROS                    1000 // Right
#define MAX_SERVO_MICROS                    2000 // Left

#define MOTOR_FORWARD_DIRECTION             1   
#define MOTOR_BACKWARD_DIRECTION            0

#define MOTOR_TURNING_MICROS                1570 // Microseconds                  
#define MIN_MOTOR_FORWARD_MICROS            1575 // Microseconds
#define MAX_MOTOR_FORWARD_MICROS            1585 // Microseconds
#define MOTOR_BRAKE_MICROS                  1500 // Microseconds
#define MOTOR_BACKWARD_MICROS               1375 // Microseconds
#define MOTOR_FORWARD_ACTIVATION_MICROS     2000 // Microseconds
#define MOTOR_BACKWARD_ACTIVATION_MICROS    1000 // Microseconds

#define SERVO_ROUND_INTERVAL                10 

//Time in milliseconds
#define MOTOR_STATE_CHANGE_INTERVAL         800  // Milliseconds
#define FRONT_SENSOR_READ_PERIOD            50   // Milliseconds
#define SIDE_SENSOR_READ_PERIOD             50   // Milliseconds
#define SERVO_UPDATE_PERIOD                 50   // Milliseconds
#define MOTOR_UPDATE_PERIOD                 50   // Milliseconds
#define MPU6050_READ_PERIOD                 50   // Milliseconds
#define LED_BLINK_PERIOD                    1000 // Milliseconds        
#define OLED_REFRESH_PERIOD                 50   // Milliseconds
#define DHT_SENSOR_READ_PERIOD              2000 // Milliseconds

/*------------------------------------------------------------*/


// FreeRTOS queue to send data from side sensors to servo
static QueueHandle_t xDhtQueue       = NULL;
static QueueHandle_t xLeftQueue      = NULL;
static QueueHandle_t xRightQueue     = NULL;
static QueueHandle_t xFrontQueue     = NULL;
static QueueHandle_t xServoQueue     = NULL;
static QueueHandle_t xMotorQueue     = NULL;
static QueueHandle_t xAccelQueue     = NULL;
static QueueHandle_t xGyroQueue      = NULL;

// FreeRTOS task handles
TaskHandle_t xLed_Task_Handle = NULL;
TaskHandle_t xOled_Screen_Task_Handle = NULL;
TaskHandle_t xDht_Sensor_Handle = NULL;
TaskHandle_t xFront_Sensor_Handle = NULL;
TaskHandle_t xLeft_Sensor_Handle = NULL;
TaskHandle_t xRight_Sensor_Handle = NULL;
TaskHandle_t xMpu6050_Sensor_Handle = NULL;
TaskHandle_t xServo_Task_Handle = NULL;
TaskHandle_t xMotor_Task_Handle = NULL;


// Function definitions
int getMedian(const int* arr, int size);;
void changeMotorDirection(uint motorPin, bool * direction, int change_interval);
int roundToIntervalFloat(const float number, int round_interval);
int roundToIntervalInt(const int number, int round_interval);

// FreeRTOS Tasks:
/*-------------------------------------------------------------*/

// Led task that blinks so that we can observe if the board works or not
void led_task() {   
    const uint led_pin = LED_PIN; 
    printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    printf("LED Task is started!\n");
    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);
    while (true) {
        gpio_put(led_pin, 1);
        vTaskDelay((TickType_t)LED_BLINK_PERIOD / portTICK_PERIOD_MS);
        gpio_put(led_pin, 0);
        vTaskDelay((TickType_t)LED_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}

void oled_screen_task(void *pvParameters) {
    const uint i2c_sda_pin = OLED_SDA_PIN;
    const uint i2c_scl_pin = OLED_SCL_PIN;
    printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    printf("OLED Screen task is started!\n");
    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");


    int front_sensor_distance = 0;
    int left_sensor_distance = 0;
    int right_sensor_distance = 0;
    float servo_micros = 0.0f;
    float motor_micros = 0.0f;
    float temperature = 27.0f;

    // Left: 400    Front 400   Right 400
    char front_sensor_text[13];
    char left_sensor_text[13];
    char right_sensor_text[13];
    char servo_text[15];
    char motor_text[15];
    char temperature_text[15];

    // Setup of I2C
    i2c_init(i2c0, 400000);
    /*gpio_set_function(i2c_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c_scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_sda_pin); 
    gpio_pull_up(i2c_scl_pin);*/

    ssd1306_t disp;
    disp.external_vcc = false;
    vTaskSuspendAll();
    bool sensor_is_connected = ssd1306_init(&disp, 128, 64, 0x3C, i2c0);
    xTaskResumeAll();
    // Delete the task if an OLED screen is not connected 
    if(sensor_is_connected != true) {
        vTaskDelete(NULL);
    }
    vTaskSuspendAll();
    ssd1306_clear(&disp);
    xTaskResumeAll();

    /*ssd1306_bmp_show_image(&disp, image_data, image_size);
    ssd1306_show(&disp);*/

    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount();
    while(true) {

        xQueuePeek(xFrontQueue, &front_sensor_distance, portMAX_DELAY);
        xQueuePeek(xRightQueue, &right_sensor_distance, portMAX_DELAY);
        xQueuePeek(xLeftQueue, &left_sensor_distance, portMAX_DELAY);
        xQueuePeek(xMotorQueue, &motor_micros, portMAX_DELAY);
        xQueuePeek(xServoQueue, &servo_micros, portMAX_DELAY);
        xQueuePeek(xDhtQueue, &temperature, portMAX_DELAY);
        vTaskSuspendAll();
        ssd1306_clear(&disp);

        snprintf(front_sensor_text, 12, "Front: %d\0", front_sensor_distance);
        ssd1306_draw_string(&disp, 38, 10, 1, front_sensor_text);


        snprintf(left_sensor_text, 12, "Left: %d\0", left_sensor_distance);
        ssd1306_draw_string(&disp, 10, 24, 1, left_sensor_text);

        snprintf(right_sensor_text, 12, "Right: %d\0", right_sensor_distance);
        ssd1306_draw_string(&disp, 70, 24, 1, right_sensor_text);  

        snprintf(motor_text, 14, "Motor: %.1f\0", motor_micros);
        ssd1306_draw_string(&disp, 30, 38, 1, motor_text);

        snprintf(servo_text, 14, "Servo: %.1f\0", servo_micros);
        ssd1306_draw_string(&disp, 2, 52, 1, servo_text);

        snprintf(temperature_text, 14, "%.1fC\0", temperature);
        ssd1306_draw_string(&disp, 98, 52, 1, temperature_text);

        ssd1306_show(&disp);
        xTaskResumeAll();
        if(disp.status == false) {
            printf("\n-------------------------------------------------\n");
            printf("OLED TASK IS DELETED\n");
            printf("---------------------------------------------------\n\n");
            vTaskDelete(NULL);
        }
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)OLED_REFRESH_PERIOD / portTICK_PERIOD_MS);
    }
}

void dht_sensor_task(void *pvParameters) {
    const uint dht_pin = DHT_PIN;
    static const dht_model_t DHT_MODEL = DHT22;
    printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    printf("DHT22 task is started!\n");
    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");


    float humidity = 0.0f;
    float temperature_c = 27.0f;
    float previous_temperature_c = 27.0f;
    xQueueOverwrite(xDhtQueue, &temperature_c);
    dht_t dht;
    dht_init(&dht, DHT_MODEL, pio0, dht_pin, true);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (true) {
        dht_start_measurement(&dht);
        dht_result_t result = dht_finish_measurement_blocking(&dht, &humidity, &temperature_c);
        if (result == DHT_RESULT_OK) {
            float temperature_to_send = 0.8f * previous_temperature_c + 0.2f * temperature_c;
            xQueueOverwrite(xDhtQueue, &temperature_to_send);
        } 
        if (result == DHT_RESULT_TIMEOUT) {
            temperature_c = previous_temperature_c;
            gpio_deinit(DHT_PIN);
            printf("\n-------------------------------------------------\n");
            printf("DHT SENSOR TASK IS DELETED\n");
            printf("---------------------------------------------------\n\n");            
            vTaskDelete(NULL);
        }
        previous_temperature_c = temperature_c;
        vTaskDelay(DHT_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
    }
}

void front_sensor_task(void *pvParameters) {
    const uint front_trig_pin = FRONT_TRIG_PIN;
    const uint front_echo_pin = FRONT_ECHO_PIN;
    
    printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    printf("Front Sensor Task is started!\n");
    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

    int front_distance_array[ARRAY_SIZE] = {0};
    int front_distance_count = 0;
    float temperature = 27.0f;
    setupUltrasonicPins(front_trig_pin, front_echo_pin);

    for(; front_distance_count < ARRAY_SIZE -1; front_distance_count++){
        int front_distance = getCm(front_trig_pin, front_echo_pin);
        if(front_distance > MAX_FRONT_DISTANCE) front_distance = MAX_FRONT_DISTANCE;
        front_distance_array[front_distance_count] = front_distance;
    }

    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount();
    while(true) {
        xQueuePeek(xDhtQueue, &temperature, 0);
        int front_distance = getCm_with_temperature(front_trig_pin, front_echo_pin, temperature);
        if(front_distance > MAX_FRONT_DISTANCE) front_distance = MAX_FRONT_DISTANCE;
        front_distance_count++;
        if(front_distance_count > ARRAY_SIZE - 1) front_distance_count = 0;

        front_distance_array[front_distance_count] = front_distance;

        #ifdef SIDE_SENSOR_DEMO
        int distance_to_send = 100;
        #else 
        int distance_to_send = getMedian(front_distance_array, ARRAY_SIZE);
        #endif
        
        /*printf("\nFront Distance = %d\n", distance_to_send);*/
        xQueueOverwrite(xFrontQueue, &distance_to_send);

        xTaskDelayUntil(&xNextWaitTime, (TickType_t)FRONT_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
    }
}

void left_sensor_task(void *pvParameters){
    const uint left_trig_pin  = LEFT_TRIG_PIN;
    const uint left_echo_pin  = LEFT_ECHO_PIN;

    printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    printf("Left Sensor Task is started!\n");
    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

    int left_distance_count = 0;
    int left_distance_array[ARRAY_SIZE] = {0};
    float temperature = 27.0f;
    setupUltrasonicPins(left_trig_pin, left_echo_pin);

    for(; left_distance_count < ARRAY_SIZE - 1; left_distance_count++) {
        int left_distance = getCm(left_trig_pin, left_echo_pin);
        if(left_distance > MAX_SIDE_SENSOR_DISTANCE) left_distance = MAX_SIDE_SENSOR_DISTANCE;
        left_distance_array[left_distance_count] = left_distance;
    }
    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount(); 
    while(true) {
        xQueuePeek(xDhtQueue, &temperature, 0);
        int left_distance = getCm_with_temperature(left_trig_pin, left_echo_pin, temperature);
        if(left_distance > MAX_SIDE_SENSOR_DISTANCE) left_distance = MAX_SIDE_SENSOR_DISTANCE;
        left_distance_count++;
        if(left_distance_count > ARRAY_SIZE-1) left_distance_count = 0;

        left_distance_array[left_distance_count] = left_distance;

        #ifdef FRONT_SENSOR_DEMO
        int distance_to_send = MAX_SIDE_DISTANCE_TO_TURN;
        #else
        int distance_to_send = getMedian(left_distance_array, ARRAY_SIZE);
        #endif

        /*printf("left Distance = %d\n", distance_to_send);*/
        xQueueOverwrite(xLeftQueue, &distance_to_send);
        
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)SIDE_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
    }
}

void right_sensor_task(void *pvParameters){
    const uint right_trig_pin  = RIGHT_TRIG_PIN;
    const uint right_echo_pin  = RIGHT_ECHO_PIN;

    printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    printf("Right Sensor Task is started!\n");
    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

    int right_distance_count = 0;
    int right_distance_array[ARRAY_SIZE] = {0};
    float temperature = 27.0f;
    setupUltrasonicPins(right_trig_pin, right_echo_pin);

    for(; right_distance_count < ARRAY_SIZE - 1; right_distance_count++) {
        int right_distance = getCm(right_trig_pin, right_echo_pin);
        if(right_distance > MAX_SIDE_SENSOR_DISTANCE) right_distance = MAX_SIDE_SENSOR_DISTANCE;
        right_distance_array[right_distance_count] = right_distance;
    }
    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount(); 
    while(true) {
        xQueuePeek(xDhtQueue, &temperature, 0);
        int right_distance = getCm_with_temperature(right_trig_pin, right_echo_pin, temperature);
        if(right_distance > MAX_SIDE_SENSOR_DISTANCE) right_distance = MAX_SIDE_SENSOR_DISTANCE;
        right_distance_count++;
        if(right_distance_count > ARRAY_SIZE-1) right_distance_count = 0;

        right_distance_array[right_distance_count] = right_distance;

        #ifdef FRONT_SENSOR_DEMO
        int distance_to_send = MAX_SIDE_DISTANCE_TO_TURN;
        #else
        int distance_to_send = getMedian(right_distance_array, ARRAY_SIZE);
        #endif

        /*printf("Right Distance = %d\n", distance_to_send);*/
        xQueueOverwrite(xRightQueue, &distance_to_send);
        
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)SIDE_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
    }
}

void mpu6050_task(void *pvParameters) {
    const uint mpu6050_sda_pin = MPU6050_SDA_PIN;
    const uint mpu6050_scl_pin = MPU6050_SCL_PIN;

    printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    printf("MPU6050 Task is Started!\n");
    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

    gpio_init(mpu6050_sda_pin);
    gpio_init(mpu6050_scl_pin);
    gpio_set_function(mpu6050_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(mpu6050_scl_pin, GPIO_FUNC_I2C);
    // Don't forget the pull ups! | Or use external ones
    gpio_pull_up(mpu6050_sda_pin);
    gpio_pull_up(mpu6050_scl_pin);
    printf("Setted MPU6050 Pins\n");
    // Pass in the I2C driver (Important for dual-core operations). The second parameter is the address,
    // which can change if you connect pin A0 to GND or to VCC.
    mpu6050_t mpu6050 = mpu6050_init(i2c_default, MPU6050_ADDRESS_A0_GND);
    printf("Initilized MPU6050 sensor\n");
    // Check if the MPU6050 can initialize

    bool mpu6050_state = mpu6050_begin(&mpu6050);
    if (mpu6050_state) {
        sleep_ms(250);
        printf("Beginning MPU6050\n");
        // Set scale of gyroscope
        vTaskSuspendAll();
        mpu6050_set_scale(&mpu6050, MPU6050_SCALE_2000DPS);
        // Set range of accelerometer
        mpu6050_set_range(&mpu6050, MPU6050_RANGE_16G);
        // Enable temperature, gyroscope and accelerometer readings
        mpu6050_set_temperature_measuring(&mpu6050, false);
        mpu6050_set_gyroscope_measuring(&mpu6050, true);
        mpu6050_set_accelerometer_measuring(&mpu6050, true);
        mpu6050_set_dlpf_mode(&mpu6050,  MPU6050_DLPF_3);

        mpu6050_calibrate_gyro(&mpu6050, 1000U);

        mpu6050_find_offset_values(&mpu6050, 1000U);
        xTaskResumeAll();
        // mpu6050_set_dhpf_mode(&mpu6050, MPU6050_DHPF_1_25HZ);
    } 

    vTaskResume(xLed_Task_Handle);
    vTaskResume(xFront_Sensor_Handle);
    vTaskResume(xLeft_Sensor_Handle);
    vTaskResume(xRight_Sensor_Handle);
    vTaskResume(xServo_Task_Handle);
    vTaskResume(xMotor_Task_Handle);
    vTaskResume(xDht_Sensor_Handle);
    vTaskResume(xOled_Screen_Task_Handle);

    if(mpu6050_state == 0) {
        printf("\n\n\n-------------------------------------------------\n");
        printf("MPU6050 Task is deleted!\n");
        printf("---------------------------------------------------\n\n\n");
        vTaskDelete(NULL);
    }
    printf("MPU6050 before while loop\n");
    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount(); 
    while (true) {
        vTaskSuspendAll();
        // Fetch all data from the sensor | I2C is only used here
        mpu6050_event(&mpu6050);

        // Pointers to float vectors with all the results
        mpu6050_vectorf_t * accel = mpu6050_get_accelerometer(&mpu6050);
        mpu6050_vectorf_t * gyro = mpu6050_get_gyroscope(&mpu6050);
        xTaskResumeAll();
        xQueueOverwrite(xAccelQueue, accel);
        xQueueOverwrite(xGyroQueue, gyro);
        printf("Accelerometer ==> x: %.2f, y: %0.2f, z: %0.2f\n", accel->x, accel->y, accel->z);
        printf("Gyroscope     ==> x: %0.2f, y: %0.2f, z: %0.2f\n\n", gyro->x, gyro->y, gyro->z);
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)(50 / portTICK_PERIOD_MS));
    }
}

void servo_task(void *pvParameters) {
    const uint servo_pin = SERVO_PIN;

    printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    printf("Servo task is started!\n");
    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

    //Servo Conversion Rate      ==>     1000us = 0 Degrees,   1500us = 60 Degrees,   2000us = 120 Degrees.
    const float MIDDLE_MICROS = (MAX_SERVO_MICROS + MIN_SERVO_MICROS) / 2;
    float current_micros = MIDDLE_MICROS;
    const float PROPORTIONAL_GAIN = (float)(MAX_SERVO_MICROS - MIN_SERVO_MICROS - 300) / (MAX_SIDE_DISTANCE_TO_TURN - MIN_SIDE_DISTANCE_TO_TURN) / 2;
    const float DERIVATIVE_GAIN = 0.02f;
    const float INTEGRAL_GAIN = 0.15f;
    const float FULL_RIGHT_DIRECTION = -1*(MAX_SIDE_DISTANCE_TO_TURN - MIN_SIDE_DISTANCE_TO_TURN);
    const float FULL_LEFT_DIRECTION  = (MAX_SIDE_DISTANCE_TO_TURN - MIN_SIDE_DISTANCE_TO_TURN);    
    
    float value_to_turn = 0;
    int front_sensor_distance = 0;
    int right_sensor_distance = 0;
    int left_sensor_distance = 0;
    float prev_proportional_turn = 0;
    float integral = 0.0f;    

    //Initiliaze servo
    setServo(servo_pin, current_micros);

    TickType_t xNextWaitTime;
    absolute_time_t startTime = get_absolute_time();
    xNextWaitTime = xTaskGetTickCount(); 

    while (true) {   
        //Waits until the queue receives data and writes the data to value_to_turn variable
        xQueuePeek(xFrontQueue, &front_sensor_distance, portMAX_DELAY);
        xQueuePeek(xRightQueue, &right_sensor_distance, portMAX_DELAY);
        xQueuePeek(xLeftQueue, &left_sensor_distance, portMAX_DELAY);
        // printf("Left Distance = %d, Front Distance = %d, R, Right Distance = %d\n", left_sensor_distance, front_sensor_distance, right_sensor_distance);

        float proportional_turn = 0;
        if(left_sensor_distance <= MIN_SIDE_DISTANCE_TO_TURN || right_sensor_distance <= MIN_SIDE_DISTANCE_TO_TURN || front_sensor_distance <= MAX_FRONT_DISTANCE_TO_TURN) {
            if(left_sensor_distance < right_sensor_distance){
                proportional_turn = FULL_RIGHT_DIRECTION;
            } else {
                proportional_turn = FULL_LEFT_DIRECTION;
            }
        }
        else if (left_sensor_distance <= MAX_SIDE_DISTANCE_TO_TURN || right_sensor_distance <= MAX_SIDE_DISTANCE_TO_TURN) {
            if(left_sensor_distance >= MAX_SIDE_DISTANCE_TO_TURN)  left_sensor_distance = MAX_SIDE_DISTANCE_TO_TURN;
            if(right_sensor_distance >= MAX_SIDE_DISTANCE_TO_TURN) right_sensor_distance = MAX_SIDE_DISTANCE_TO_TURN;
            proportional_turn = (float)(left_sensor_distance - right_sensor_distance);
        }
        else {
            proportional_turn = 0;
        }

        // get the current time
        absolute_time_t endTime = get_absolute_time(); 

        // convert the time difference between readings from microseconds to seconds by multiplying derivative by 10^6
        float delta_T = (float)(absolute_time_diff_us(startTime, endTime)); 
        float derivative = 1000000*(proportional_turn - prev_proportional_turn) / delta_T;

        // Calculate the integral, and set its bound so that it cannot increase indefinitely
        integral += proportional_turn;
        if(integral >= 150 / INTEGRAL_GAIN) integral = 150 / INTEGRAL_GAIN;
        else if (integral <= -150 / INTEGRAL_GAIN) integral = -150 / INTEGRAL_GAIN; 
    
        // Get the PID value by applying gains to the terms and adding them up
        float proportional_term = (proportional_turn * PROPORTIONAL_GAIN);
        float derivative_term   = (derivative * DERIVATIVE_GAIN);
        float integral_term     = (integral * INTEGRAL_GAIN);
        float value_to_turn = proportional_term + derivative_term + integral_term;

        // Save the current time and current proportional turn to calculate derivative in the next loop
        startTime = endTime;
        prev_proportional_turn = proportional_turn;
        
        // Reverse the turning direction if the motor is going backwards
        if(front_sensor_distance <= MIN_FRONT_DISTANCE) value_to_turn *= -1;
        /*printf("Proportional Term = %f, Integral Term = %f, Derivative Term = %f\n", proportional_term, integral_term, derivative_term);
        printf("\tValue To Turn = %f\n", value_to_turn);*/
        current_micros = MIDDLE_MICROS + value_to_turn;
        if(current_micros <= MIN_SERVO_MICROS) current_micros = MIN_SERVO_MICROS;
        if(current_micros >= MAX_SERVO_MICROS) current_micros = MAX_SERVO_MICROS;

        printf("\tServo Current Micros = %f\n\n", current_micros);
        setMillis(servo_pin, roundToIntervalFloat((current_micros), SERVO_ROUND_INTERVAL));
        
        xQueueOverwrite(xServoQueue, &current_micros);
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)SERVO_UPDATE_PERIOD/portTICK_PERIOD_MS);
    }
}

void motor_task(void *pvParameters) {
    const uint motor_pin = MOTOR_ESC_PIN;

    printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++\n");
    printf("Motor Task is started!\n");
    printf("+++++++++++++++++++++++++++++++++++++++++++++++++++\n\n");

    // Motor Conversion Rate     ==>     1000 = Reverse Max,     1500 = Stop,    2000 = Forward Max.   
    const float CONVERSION_RATE = (float)(MAX_MOTOR_FORWARD_MICROS - MIN_MOTOR_FORWARD_MICROS) / (MAX_FRONT_DISTANCE);
    float current_micros = MOTOR_BRAKE_MICROS;
    int micros_received = 0;
    bool direction = MOTOR_FORWARD_DIRECTION;
    bool brake_condition = true;
    // Initiliaze motor as servo so that we can control it through ESC
    setServo(motor_pin, current_micros);
    sleep_ms(2000);
    TickType_t xNextWaitTime = xTaskGetTickCount();
    while(true) {
        // Wait for the front sensor to send data
        xQueuePeek(xFrontQueue, &micros_received, portMAX_DELAY);
        // Brake early or the car cannot stop
        if(micros_received <= MOTOR_DISTANCE_TO_BRAKE && brake_condition == true){
            setMillis(motor_pin, MOTOR_BRAKE_MICROS);
            vTaskDelay((TickType_t) MOTOR_BRAKE_PERIOD / portTICK_PERIOD_MS);
            brake_condition = false;
        } else if (micros_received > MOTOR_DISTANCE_TO_BRAKE){
            brake_condition = true;
        }

        if(micros_received <= MIN_FRONT_DISTANCE){
            // Set the esc direction change
            if(direction != MOTOR_BACKWARD_DIRECTION) 
                changeMotorDirection(motor_pin, &direction, MOTOR_STATE_CHANGE_INTERVAL);
            
            // Go backwards if front distance is less than 10cm
            current_micros = MOTOR_BACKWARD_MICROS;
        } else {
            // Set the esc direction change 
            if(direction != MOTOR_FORWARD_DIRECTION) 
                changeMotorDirection(motor_pin, &direction, MOTOR_STATE_CHANGE_INTERVAL);
            
            // Go forward if fron distance is more than specified amount
            if(micros_received <= MAX_FRONT_DISTANCE_TO_TURN){
                current_micros = MOTOR_TURNING_MICROS;
            }
            else if(micros_received <= MIN_FRONT_DISTANCE_TO_ACCELERATE) {
                current_micros = MIN_MOTOR_FORWARD_MICROS;
            }
            else if (micros_received >= MIN_FRONT_DISTANCE_TO_ACCELERATE) {
                current_micros = MIN_MOTOR_FORWARD_MICROS + (float)(CONVERSION_RATE * micros_received);
                if(current_micros > MAX_MOTOR_FORWARD_MICROS) current_micros = MAX_MOTOR_FORWARD_MICROS;
            }
        }
        printf("\nReceived Motor Input = %f\n", current_micros);
        setMillis(motor_pin, current_micros);
        xQueueOverwrite(xMotorQueue, &current_micros);
        // Store previous speed for future use
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)MOTOR_UPDATE_PERIOD / portTICK_PERIOD_MS);
    }
}


/*-----------------------------------------------------------*/


int main()
{
    //initiliaze serial communication through USB
    stdio_init_all();
    sleep_ms(150);
    //Create queue for the side sensors
    xDhtQueue   = xQueueCreate(1, sizeof(float));
    xFrontQueue = xQueueCreate(1, sizeof(int));
    xRightQueue = xQueueCreate(1, sizeof(int));
    xLeftQueue  = xQueueCreate(1, sizeof(int));
    xServoQueue = xQueueCreate(1, sizeof(float));
    xMotorQueue = xQueueCreate(1, sizeof(float));
    xAccelQueue = xQueueCreate(1, sizeof(mpu6050_vectorf_t));
    xGyroQueue = xQueueCreate(1, sizeof(mpu6050_vectorf_t));

    // Necessary to check if tasks are created
    BaseType_t xLed_Task_Returned;
    BaseType_t xOled_Screen_Task_Returned;
    BaseType_t xDht_Sensor_Returned;
    BaseType_t xFront_Sensor_Returned;
    BaseType_t xRigth_Sensor_Returned;
    BaseType_t xLeft_Sensor_Returned;
    BaseType_t xMpu6050_Sensor_Returned;
    BaseType_t xServo_Task_Returned;
    BaseType_t xMotor_Task_Returned;

    //Create freeRTOS tasks
    xLed_Task_Returned = xTaskCreate(led_task, 
                "LED_Task", 
                configMINIMAL_STACK_SIZE, 
                NULL, 
                1, 
                &xLed_Task_Handle);
    if(xLed_Task_Returned != pdPASS) {
        printf("Led Task could not be created\n");
        return 0;
    }

    xOled_Screen_Task_Returned = xTaskCreate(oled_screen_task,
                "OLED_Screen_Task",
                configMINIMAL_STACK_SIZE,
                NULL,
                2,
                &xOled_Screen_Task_Handle);
    if(xOled_Screen_Task_Returned != pdPASS) {
        printf("OLED Screen Task could not be created\n");
        return 0;
    }

    xDht_Sensor_Returned = xTaskCreate(dht_sensor_task,
                "DHT_Sensor_Task",
                configMINIMAL_STACK_SIZE,
                NULL,
                2,
                &xDht_Sensor_Handle);
    if(xDht_Sensor_Returned != pdPASS) {
        printf("DHT Sensor Task could not be created\n");
        return 0;
    }

    xFront_Sensor_Returned = xTaskCreate(front_sensor_task, 
                "Front_Servor_Task", 
                configMINIMAL_STACK_SIZE, 
                NULL, 
                6, 
                &xFront_Sensor_Handle);
    if(xLed_Task_Returned != pdPASS) {
        printf("Led Task could not be created\n");
        return 0;
    }
  
    xRigth_Sensor_Returned = xTaskCreate(right_sensor_task, 
                "Right_Servor_Task", 
                configMINIMAL_STACK_SIZE, 
                NULL, 
                5, 
                &xRight_Sensor_Handle);
    if(xRigth_Sensor_Returned != pdPASS) {
        printf("Right Sensor Task could not be created\n");
        return 0;
    }

    xLeft_Sensor_Returned = xTaskCreate(left_sensor_task, 
                "Left_Servor_Task", 
                configMINIMAL_STACK_SIZE, 
                NULL, 
                5, 
                &xLeft_Sensor_Handle);                                
    if(xLeft_Sensor_Returned != pdPASS) {
        printf("Left Sensor Task could not be created\n");
        return 0;
    }

    xMpu6050_Sensor_Returned = xTaskCreate(mpu6050_task,
                "MPU6050_Sensor_Task",
                configMINIMAL_STACK_SIZE * 16,
                NULL,
                4,
                &xMpu6050_Sensor_Handle);
    if(xMpu6050_Sensor_Returned != pdPASS){
        printf("MPU6050 Sensor Task could not be created\n");
        return 0;
    }

    xMotor_Task_Returned = xTaskCreate(motor_task, 
                "Motor_Task", 
                configMINIMAL_STACK_SIZE * 2, 
                NULL, 
                4, 
                &xMotor_Task_Handle);
    if(xMotor_Task_Returned != pdPASS) {
        printf("Motor Task could not be created\n");
        return 0;
    }    

    xServo_Task_Returned = xTaskCreate(servo_task, 
                "Servo_Task", 
                configMINIMAL_STACK_SIZE * 2, 
                NULL, 
                4, 
                &xServo_Task_Handle);  
    if(xServo_Task_Returned != pdPASS) {
        printf("Servo Task could not be created\n");
        return 0;
    }
    printf("All Tasks are successfuly created\n\n");

    vTaskSuspend(xOled_Screen_Task_Handle);
    vTaskSuspend(xMotor_Task_Handle);
    vTaskSuspend(xServo_Task_Handle);
    vTaskSuspend(xFront_Sensor_Handle);
    vTaskSuspend(xLeft_Sensor_Handle);
    vTaskSuspend(xRight_Sensor_Handle);
    vTaskSuspend(xDht_Sensor_Handle);
    vTaskSuspend(xLed_Task_Handle);

    //start the main loop
    vTaskStartScheduler();

    //The code never reaches her
    while(1){};
}


/*--------------------------------------------------------*/


int getMedian(const int* arr, int size) {
    int sortedArray[size];
    for (int i = 0; i < size; i++) {
        sortedArray[i] = arr[i];
    }
    // Insertion Sort
    for (int i = 1; i < size; i++) {
        int key = sortedArray[i];
        int j = i - 1;
        while (j >= 0 && sortedArray[j] > key) {
            sortedArray[j + 1] = sortedArray[j];
            j--;
        }
        sortedArray[j + 1] = key;
    }

    int medianIndex = size / 2;
    if (size % 2 == 0) {
        // Average of two middle elements for even-sized array
        return (sortedArray[medianIndex - 1] + sortedArray[medianIndex]) / 2;
    } else {
        // Middle element for odd-sized array
        return sortedArray[medianIndex];
    }
}

void changeMotorDirection(uint motorPin, bool * direction, int change_interval){
    if(*direction) {
        setMillis(motorPin, MOTOR_BRAKE_MICROS);
        vTaskDelay((TickType_t)(change_interval / 8) * 5 / portTICK_PERIOD_MS);
        setMillis(motorPin, MOTOR_BACKWARD_ACTIVATION_MICROS);
        vTaskDelay((TickType_t)(change_interval / 8) / portTICK_PERIOD_MS);
        setMillis(motorPin, MOTOR_BRAKE_MICROS);
        vTaskDelay((TickType_t)(change_interval / 8) * 2 / portTICK_PERIOD_MS);
        *direction = MOTOR_BACKWARD_DIRECTION;
        return;
    } else {
        setMillis(motorPin, MOTOR_BRAKE_MICROS);
        vTaskDelay((TickType_t)change_interval / portTICK_PERIOD_MS);
        *direction = MOTOR_FORWARD_DIRECTION;
        return;
    }
}

int roundToIntervalFloat(const float number, int round_interval){
    int cast_number = (int)number;
    cast_number = (int)cast_number / round_interval;
    cast_number = cast_number * round_interval;
    return cast_number;
}

int roundToIntervalInt(const int number, int round_interval){
    int cast_number = number;
    cast_number = (int)cast_number / round_interval;
    cast_number = cast_number * round_interval;
    return cast_number;
}