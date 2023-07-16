#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/float.h"
#include "servo.h"
#include "ultrasonic.h"

// #define SINGLE_SENSOR_DEMO 1
// #define SIDE_SENSOR_DEMO 1

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
#define LED_PIN         25

#define RIGHT_TRIG_PIN  16
#define RIGHT_ECHO_PIN  17
#define LEFT_TRIG_PIN   19
#define LEFT_ECHO_PIN   18
#define FRONT_TRIG_PIN  21 
#define FRONT_ECHO_PIN  20

#define MOTOR_ESC_PIN   5
#define SERVO_PIN       2
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
#define SERVO_UPDATE_PERIOD                 10   // Milliseconds
#define MOTOR_UPDATE_PERIOD                 10   // Milliseconds
#define LED_BLINK_PERIOD                    1000 // Milliseconds               
/*------------------------------------------------------------*/


// FreeRTOS queue to send data from side sensors to servo
static QueueHandle_t xLeftQueue      = NULL;
static QueueHandle_t xRightQueue     = NULL;
static QueueHandle_t xFrontQueue     = NULL;

// FreeRTOS task handles
TaskHandle_t xLed_Task_Handle = NULL;
TaskHandle_t xFront_Sensor_Handle = NULL;
TaskHandle_t xLeft_Sensor_Handle = NULL;
TaskHandle_t xRight_Sensor_Handle = NULL;
TaskHandle_t xServo_Task_Handle = NULL;
TaskHandle_t xMotor_Task_Handle = NULL;

// Function definitions
int getMedian(const int* arr, int size);
void changeMotorDirection(uint motorPin, bool * direction, int change_interval);
int roundToIntervalFloat(const float number, int round_interval);
int roundToIntervalInt(const int number, int round_interval);

// FreeRTOS Tasks:
/*-------------------------------------------------------------*/

// Led task that blinks so that we can observe if the board works or not
void led_task() {   
    const uint led_pin = LED_PIN; 
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);
    while (true) {
        gpio_put(led_pin, 1);
        vTaskDelay((TickType_t)LED_BLINK_PERIOD / portTICK_PERIOD_MS);
        gpio_put(led_pin, 0);
        vTaskDelay((TickType_t)LED_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}

void front_sensor_task(void *pvParameters) {
    const uint front_trig_pin = FRONT_TRIG_PIN;
    const uint front_echo_pin = FRONT_ECHO_PIN;

    int front_distance_array[ARRAY_SIZE] = {0};
    int front_distance_count = 0;

    setupUltrasonicPins(front_trig_pin, front_echo_pin);

    for(; front_distance_count < ARRAY_SIZE -1; front_distance_count++){
        int front_distance = getCm(front_trig_pin, front_echo_pin);
        if(front_distance > MAX_FRONT_DISTANCE) front_distance = MAX_FRONT_DISTANCE;
        front_distance_array[front_distance_count] = front_distance;
    }

    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount();
    while(true) {
        int front_distance = getCm(front_trig_pin, front_echo_pin);
        if(front_distance > MAX_FRONT_DISTANCE) front_distance = MAX_FRONT_DISTANCE;
        front_distance_count++;
        if(front_distance_count > ARRAY_SIZE - 1) front_distance_count = 0;

        front_distance_array[front_distance_count] = front_distance;

        #ifdef SIDE_SENSOR_DEMO
        int distance_to_send = 100;
        #else 
        int distance_to_send = getMedian(front_distance_array, ARRAY_SIZE);
        #endif
        
        /*printf("\nMiddle Distance = %d", distance_to_send);*/
        xQueueOverwrite(xFrontQueue, &distance_to_send);

        xTaskDelayUntil(&xNextWaitTime, (TickType_t)FRONT_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
    }
}

void left_sensor_task(void *pvParameters){
    const uint left_trig_pin  = LEFT_TRIG_PIN;
    const uint left_echo_pin  = LEFT_ECHO_PIN;

    int left_distance_count = 0;
    int left_distance_array[ARRAY_SIZE] = {0};
    setupUltrasonicPins(left_trig_pin, left_echo_pin);

    for(; left_distance_count < ARRAY_SIZE - 1; left_distance_count++) {
        int left_distance = getCm(left_trig_pin, left_echo_pin);
        if(left_distance > MAX_SIDE_SENSOR_DISTANCE) left_distance = MAX_SIDE_SENSOR_DISTANCE;
        left_distance_array[left_distance_count] = left_distance;
    }
    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount(); 
    while(true) {
        int left_distance = getCm(left_trig_pin, left_echo_pin);
        if(left_distance > MAX_SIDE_SENSOR_DISTANCE) left_distance = MAX_SIDE_SENSOR_DISTANCE;
        left_distance_count++;
        if(left_distance_count > ARRAY_SIZE-1) left_distance_count = 0;

        left_distance_array[left_distance_count] = left_distance;

        #ifdef FRONT_SENSOR_DEMO
        int distance_to_send = MAX_SIDE_DISTANCE_TO_TURN;
        #else
        int distance_to_send = getMedian(left_distance_array, ARRAY_SIZE);
        #endif
        xQueueOverwrite(xRightQueue, &distance_to_send);
        
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)SIDE_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
    }
}

void right_sensor_task(void *pvParameters){
    const uint right_trig_pin  = RIGHT_TRIG_PIN;
    const uint right_echo_pin  = RIGHT_ECHO_PIN;

    int right_distance_count = 0;
    int right_distance_array[ARRAY_SIZE] = {0};
    setupUltrasonicPins(right_trig_pin, right_echo_pin);

    for(; right_distance_count < ARRAY_SIZE - 1; right_distance_count++) {
        int right_distance = getCm(right_trig_pin, right_echo_pin);
        if(right_distance > MAX_SIDE_SENSOR_DISTANCE) right_distance = MAX_SIDE_SENSOR_DISTANCE;
        right_distance_array[right_distance_count] = right_distance;
    }
    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount(); 
    while(true) {
        int right_distance = getCm(right_trig_pin, right_echo_pin);
        if(right_distance > MAX_SIDE_SENSOR_DISTANCE) right_distance = MAX_SIDE_SENSOR_DISTANCE;
        right_distance_count++;
        if(right_distance_count > ARRAY_SIZE-1) right_distance_count = 0;

        right_distance_array[right_distance_count] = right_distance;

        #ifdef FRONT_SENSOR_DEMO
        int distance_to_send = MAX_SIDE_DISTANCE_TO_TURN;
        #else
        int distance_to_send = getMedian(right_distance_array, ARRAY_SIZE);
        #endif
        xQueueOverwrite(xRightQueue, &distance_to_send);
        
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)SIDE_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
    }
}

void servo_task(void *pvParameters) {
    const uint servo_pin = SERVO_PIN;
    //Servo Conversion Rate      ==>     1000us = 0 Degrees,   1500us = 60 Degrees,   2000us = 120 Degrees.
    const float MIDDLE_MICROS = (MAX_SERVO_MICROS + MIN_SERVO_MICROS) / 2;
    float current_micros = MIDDLE_MICROS;
    const float PROPORTIONAL_GAIN = (float)(MAX_SERVO_MICROS - MIN_SERVO_MICROS - 300) / (MAX_SIDE_DISTANCE_TO_TURN - MIN_SIDE_DISTANCE_TO_TURN) / 2;
    const float DERIVATIVE_GAIN = -0.1f;
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

        current_micros = MIDDLE_MICROS + value_to_turn;
        if(current_micros <= MIN_SERVO_MICROS) current_micros = MIN_SERVO_MICROS;
        if(current_micros >= MAX_SERVO_MICROS) current_micros = MAX_SERVO_MICROS;

        setMillis(servo_pin, roundToIntervalFloat((current_micros), SERVO_ROUND_INTERVAL));

        xTaskDelayUntil(&xNextWaitTime, (TickType_t)SERVO_UPDATE_PERIOD/portTICK_PERIOD_MS);
    }
}

void motor_task(void *pvParameters) {
    const uint motor_pin = MOTOR_ESC_PIN;
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
        /*printf("\nReceived Motor Input = %f\n", current_micros);*/
        setMillis(motor_pin, current_micros);
        // Store previous speed for future use
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)MOTOR_UPDATE_PERIOD / portTICK_PERIOD_MS);
    }
}


/*-----------------------------------------------------------*/


int main()
{
    //initiliaze serial communication through USB
    stdio_init_all();
    sleep_ms(1500);

    //Create queue for the side sensors
    xFrontQueue = xQueueCreate(1, sizeof(int));
    xRightQueue = xQueueCreate(1, sizeof(int));
    xLeftQueue  = xQueueCreate(1, sizeof(int));

    //Create freeRTOS tasks
    xTaskCreate(led_task, 
                "LED_Task", 
                configMINIMAL_STACK_SIZE, 
                NULL, 
                1, 
                &xLed_Task_Handle);

    xTaskCreate(front_sensor_task, 
                "Front_Servor_Task", 
                512, 
                NULL, 
                5, 
                &xFront_Sensor_Handle);

    xTaskCreate(right_sensor_task, 
                "Right_Servor_Task", 
                512, 
                NULL, 
                4, 
                &xRight_Sensor_Handle);

    xTaskCreate(left_sensor_task, 
                "Left_Servor_Task", 
                512, 
                NULL, 
                4, 
                &xLeft_Sensor_Handle);                                

    xTaskCreate(motor_task, 
                "Motor_Task", 
                1024, 
                NULL, 
                3, 
                &xMotor_Task_Handle);

    xTaskCreate(servo_task, 
                "Servo_Task", 
                2048, 
                NULL, 
                3, 
                &xServo_Task_Handle);  

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