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
#define MIN_SIDE_DISTANCE_TO_TURN           20   // CM
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
#define LED_BLINK_PERIOD                    1000 // Milliseconds               
/*------------------------------------------------------------*/


// FreeRTOS queue to send data from side sensors to servo
static QueueHandle_t xSideQueue      = NULL;
static QueueHandle_t xFrontQueue     = NULL;


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



//Front servo task that measures the distance in front and sends it to the DC motor
void front_sensor_task(void *pvParameters) {
    const uint front_trig_pin = FRONT_TRIG_PIN;
    const uint front_echo_pin = FRONT_ECHO_PIN;

    int middle_prev = 0;
    int distance_array[ARRAY_SIZE] = {0};
    int distance_count = 0;

    setupUltrasonicPins(front_trig_pin, front_echo_pin);

    for(; distance_count < ARRAY_SIZE -1; distance_count++){
        int middle_distance = getCm(front_trig_pin, front_echo_pin);
        if(middle_distance > MAX_FRONT_DISTANCE) middle_distance = MAX_FRONT_DISTANCE;
        distance_array[distance_count] = middle_distance;
    }

    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount();
    while(true) {
        int middle_distance = getCm(front_trig_pin, front_echo_pin);
        if(middle_distance > MAX_FRONT_DISTANCE) middle_distance = MAX_FRONT_DISTANCE;
        distance_count++;
        if(distance_count > ARRAY_SIZE - 1) distance_count = 0;

        distance_array[distance_count] = middle_distance;

        #ifdef SIDE_SENSOR_DEMO
        int distance_to_send = 100;
        #else 
        int distance_to_send = getMedian(distance_array, ARRAY_SIZE);
        #endif
        
        printf("\nMiddle Distance = %d", distance_to_send);
        xQueueOverwrite(xFrontQueue, &distance_to_send);

        xTaskDelayUntil(&xNextWaitTime, (TickType_t)FRONT_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
        // middle_prev = middle_distance;
    }
}

// Sensor task that reads from left and right sensors, calculates the amount of turn the servo must make and sends it through a queue.
void side_sensor_task(void *pvParameters) {
    // GPIO Pins of the side sensors
    const uint left_trig_pin  = LEFT_TRIG_PIN;
    const uint left_echo_pin  = LEFT_ECHO_PIN;
    const uint right_trig_pin = RIGHT_TRIG_PIN;
    const uint right_echo_pin = RIGHT_ECHO_PIN;

    const float PROPORTIONAL_GAIN = (float)(MAX_SERVO_MICROS - MIN_SERVO_MICROS) / (MAX_SIDE_DISTANCE_TO_TURN - MIN_SIDE_DISTANCE_TO_TURN) / 2;
    const float DERIVATIVE_GAIN = 0.09f;
    const float INTEGRAL_GAIN = 0.005f;
    const float FULL_RIGHT_DIRECTION = -1*(MAX_SIDE_DISTANCE_TO_TURN - MIN_SIDE_DISTANCE_TO_TURN);
    const float FULL_LEFT_DIRECTION  = (MAX_SIDE_DISTANCE_TO_TURN - MIN_SIDE_DISTANCE_TO_TURN);
    // Initilization of ultrasonic sensors    
    setupUltrasonicPins(left_trig_pin, left_echo_pin);
    setupUltrasonicPins(right_trig_pin, right_echo_pin);
    
    int front_sensor_peeked = 0;
    int motor_direction = MOTOR_FORWARD_DIRECTION;
    int left_array[ARRAY_SIZE] = {0};
    int right_array[ARRAY_SIZE] = {0};
    int array_count = 0;
    float prev_proportional_turn = 0;

    // Read from sensors until the array has only 1 empty slot before the main loop
    for(; array_count < ARRAY_SIZE - 1; array_count++){

        int left_distance  = getCm(left_trig_pin, left_echo_pin);
        if(left_distance > MAX_SIDE_SENSOR_DISTANCE) left_distance = MAX_SIDE_SENSOR_DISTANCE;
        left_array[array_count] = left_distance;

        int right_distance = getCm(right_trig_pin, right_echo_pin);
        if(right_distance > MAX_SIDE_SENSOR_DISTANCE) right_distance = MAX_SIDE_SENSOR_DISTANCE;
        right_array[array_count] = right_distance;
    }

    // Gets current time in terms of freeRTOS ticks
    TickType_t xNextWaitTime;
    absolute_time_t startTime = get_absolute_time();
    xNextWaitTime = xTaskGetTickCount(); 
    while(true) {
        // Read from sensors
        int left_distance  = getCm(left_trig_pin, left_echo_pin);
        if (left_distance >= MAX_SIDE_SENSOR_DISTANCE) left_distance = MAX_SIDE_SENSOR_DISTANCE;
        int right_distance = getCm(right_trig_pin, right_echo_pin);
        if (right_distance >= MAX_SIDE_SENSOR_DISTANCE) right_distance = MAX_SIDE_SENSOR_DISTANCE;

        // Add the current distance measurement to the array by overwriting the most outdated measurement
        array_count++;
        if(array_count > ARRAY_SIZE - 1) array_count = 0;
        left_array[array_count]  = left_distance;
        right_array[array_count] = right_distance;

        // Calculate the median measurements of the arrays
        #ifdef SINGLE_SENSOR_DEMO
        int left_distance_median  = 100;
        int right_distance_median = 100;
        #else 
        int left_distance_median  = getMedian(left_array, ARRAY_SIZE);
        int right_distance_median = getMedian(right_array, ARRAY_SIZE);
        #endif

        printf("\nLeft Distance = %dcm\tRight Distance = %dcm", left_distance, right_distance);
        
        // Check the front distance so that if the front distance is small enough the car can still turn
        int proportional_turn;
        xQueuePeek(xFrontQueue, &front_sensor_peeked, portMAX_DELAY);
        // Check if the car is going forward or backward
        if(front_sensor_peeked <= MIN_FRONT_DISTANCE) motor_direction = MOTOR_BACKWARD_DIRECTION;
        else                                          motor_direction = MOTOR_FORWARD_DIRECTION;

        // Calculate the proportional term
        if(left_distance_median <= MIN_SIDE_DISTANCE_TO_TURN || right_distance_median <= MIN_SIDE_DISTANCE_TO_TURN || front_sensor_peeked <= MAX_FRONT_DISTANCE_TO_TURN) {
            if(left_distance_median < right_distance_median){
                proportional_turn = FULL_RIGHT_DIRECTION;
            } else {
                proportional_turn = FULL_LEFT_DIRECTION;
            }
        }
        else if (left_distance_median <= MAX_SIDE_DISTANCE_TO_TURN || right_distance_median <= MAX_SIDE_DISTANCE_TO_TURN) {
            if(left_distance_median >= MAX_SIDE_DISTANCE_TO_TURN)  left_distance_median = MAX_SIDE_DISTANCE_TO_TURN;
            if(right_distance_median >= MAX_SIDE_DISTANCE_TO_TURN) right_distance_median = MAX_SIDE_DISTANCE_TO_TURN;
            proportional_turn = (left_distance_median - right_distance_median);
            if(proportional_turn >= FULL_RIGHT_DIRECTION) proportional_turn = FULL_RIGHT_DIRECTION;
            if(proportional_turn <= FULL_LEFT_DIRECTION)  proportional_turn = FULL_LEFT_DIRECTION;
        }
        else {
            proportional_turn = 0;
        }
        // get the current time
        absolute_time_t endTime = get_absolute_time(); 
        // convert the time difference between readings from microseconds to seconds
        float delta_T = (float)(absolute_time_diff_us(startTime, endTime) / 1000000); 
        float derivative = (proportional_turn - prev_proportional_turn) / delta_T;
        // The integral term fine tunes the turning angle only during small turn angles
        int integral = 0;
        if((FULL_RIGHT_DIRECTION / 4 <= proportional_turn) && (proportional_turn <= FULL_RIGHT_DIRECTION / 4)){
            integral += proportional_turn;
        }
        // Get the PID value by adding proportional and derivative terms
        float value_to_turn = (float)(proportional_turn * PROPORTIONAL_GAIN) - (derivative * DERIVATIVE_GAIN) + (integral * INTEGRAL_GAIN);

        startTime = endTime;
        prev_proportional_turn = proportional_turn;

        // Reverse the turning direction if the motor is going backwards
        if(motor_direction == MOTOR_BACKWARD_DIRECTION) value_to_turn *= -1;

        printf("\tTurn_Value = %f, Proportional = %f, Derivative = %f, Integral = %f",value_to_turn, proportional_turn*PROPORTIONAL_GAIN, derivative*DERIVATIVE_GAIN, integral*INTEGRAL_GAIN);
        // Send the data to the queue so that the servo task can access it
        xQueueSend(xSideQueue, &value_to_turn, 0U);
        // Delay certain amount of ticks
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)SIDE_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
    }
}

// Motor task that waits for data from the front sensor that controls the robot's speed
void motor_task_exp(void *pvParameters){
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
        // Store previous speed for future use
        vTaskDelay((TickType_t)FRONT_SENSOR_READ_PERIOD / portTICK_PERIOD_MS);
    }
}

void servo_task(void *pvParameters) {
    const uint servo_pin = SERVO_PIN;
    //Servo Conversion Rate      ==>     1000us = 0 Degrees,   1500us = 60 Degrees,   2000us = 120 Degrees.
    const float MIDDLE_MICROS = (MAX_SERVO_MICROS + MIN_SERVO_MICROS) / 2;
    float current_micros = MIDDLE_MICROS;
    float value_to_turn = 0;
    //Initiliaze servo
    setServo(servo_pin, current_micros);

    while (true) {   
        //Waits until the queue receives data and writes the data to value_to_turn variable
        xQueueReceive(xSideQueue, &value_to_turn, portMAX_DELAY);
        printf("\nReceived Servo Input = %f", value_to_turn);

        //Turn the servo according to the data sent from the sensor task
        current_micros = MIDDLE_MICROS + value_to_turn;
        if(current_micros <= MIN_SERVO_MICROS) current_micros = MIN_SERVO_MICROS;
        if(current_micros >= MAX_SERVO_MICROS) current_micros = MAX_SERVO_MICROS;

        setMillis(servo_pin, roundToIntervalFloat((current_micros), SERVO_ROUND_INTERVAL));
    }
}


/*-----------------------------------------------------------*/


int main()
{
    //initiliaze serial communication through USB
    stdio_init_all();
    sleep_ms(1500);

    //Create queue for the side sensors
    xSideQueue  = xQueueCreate(1, sizeof(float));
    xFrontQueue = xQueueCreate(1, sizeof(int));

    //Create freeRTOS tasks
    xTaskCreate(led_task, 
                "LED_Task", 
                configMINIMAL_STACK_SIZE, 
                NULL, 
                1, 
                NULL);

    xTaskCreate(front_sensor_task, 
                "Front_Servor_Task", 
                2048, 
                NULL, 
                5, 
                NULL);

    xTaskCreate(motor_task_exp, 
                "Motor_Task", 
                2048, 
                NULL, 
                3, 
                NULL);

    xTaskCreate(servo_task, 
                "Servo_Task", 
                512, 
                NULL, 
                3, 
                NULL);  

    xTaskCreate(side_sensor_task, 
                "Side_Sensor_Task", 
                4096, 
                NULL, 
                4, 
                NULL);


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