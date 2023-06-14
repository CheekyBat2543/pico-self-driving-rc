#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/float.h"
#include "servo.h"
#include "ultrasonic.h"

#define COS45                               0.70710678118F

#define ARRAY_SIZE                          5

#define MIN_FRONT_DISTANCE                  10
#define MAX_FRONT_DISTANCE                  400

#define MAX_SIDE_SENSOR_DISTANCE            200
#define MIN_SIDE_DISTANCE_TO_TURN           50
#define MAX_SIDE_DISTANCE_TO_TURN           100

#define MIN_SERVO_MICROS                    1000 // Right
#define MAX_SERVO_MICROS                    2000 // Left

#define FORWARD                             1
#define BACKWARD                            0

#define MIN_MOTOR_FORWARD_MICROS            1580
#define MAX_MOTOR_FORWARD_MICROS            1620
#define MOTOR_BRAKE_MICROS                  1500
#define MOTOR_BACKWARD_MICROS               1380
#define MOTOR_FORWARD_ACTIVATION_MICROS     2000
#define MOTOR_BACKWARD_ACTIVATION_MICROS    1000

#define MOTOR_STATE_CHANGE_INTERVAL         5

#define SERVO_ROUND_INTERVAL                50
#define FRONT_READ_FREQUENCY                5
#define SIDE_READ_FREQUENCY                 5

//FreeRTOS queue to send data from side sensors to servo
static QueueHandle_t xSideQueue = NULL;
static QueueHandle_t xMiddleQueue = NULL;

uint map(uint input, uint a, uint b, uint c, uint d) {
    return a + input * (b - a) / (d - c) ; 
}

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

//Led task that blinks so that we can observe if the board works or not
void led_task() {   
    const uint LED_PIN = 25; 
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(100);
        gpio_put(LED_PIN, 0);
        vTaskDelay(100);
    }
}

void changeMotorState(uint motorPin, bool direction){
    if(direction) {
        setMillis(motorPin, MOTOR_FORWARD_ACTIVATION_MICROS);
        vTaskDelay(MOTOR_STATE_CHANGE_INTERVAL);
        setMillis(motorPin, MOTOR_BRAKE_MICROS);
        vTaskDelay(MOTOR_STATE_CHANGE_INTERVAL);
    } else {
        setMillis(motorPin, MOTOR_BACKWARD_ACTIVATION_MICROS);
        vTaskDelay(MOTOR_STATE_CHANGE_INTERVAL);
        setMillis(motorPin, MOTOR_BRAKE_MICROS);
        vTaskDelay(MOTOR_STATE_CHANGE_INTERVAL);
    }
}

//Front servo task that measures the distance in front and sends it to the DC motor
void front_sensor_task(void *pvParameters) {
    const uint M_TRIG_PIN = 16;
    const uint M_ECHO_PIN = 17;

    int middle_prev = 0;
    int distance_array[ARRAY_SIZE] = {0};
    int distance_count = 0;


    setupUltrasonicPins(M_TRIG_PIN, M_ECHO_PIN);

    for(; distance_count < ARRAY_SIZE -1; distance_count++){
        int middle_distance = getCm(M_TRIG_PIN, M_ECHO_PIN);
        if(middle_distance > MAX_FRONT_DISTANCE) middle_distance = MAX_FRONT_DISTANCE;
        distance_array[distance_count] = middle_distance;
    }

    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount();
    while(true) {
        int middle_distance = getCm(M_TRIG_PIN, M_ECHO_PIN);
        if(middle_distance > MAX_FRONT_DISTANCE) middle_distance = MAX_FRONT_DISTANCE;
        distance_count++;
        if(distance_count > ARRAY_SIZE - 1) distance_count = 0;

        distance_array[distance_count] = middle_distance;

        int distance_to_send = getMedian(distance_array, ARRAY_SIZE);

        printf("\nMiddle Distance = %d", distance_to_send);
        xQueueOverwrite(xMiddleQueue, &distance_to_send);

        xTaskDelayUntil(&xNextWaitTime, (TickType_t)FRONT_READ_FREQUENCY);
        // middle_prev = middle_distance;
    }
}

//sensor task that reads from left and right sensors, calculates the amount of turn the servo must make and sends it through a queue.
void side_sensor_task(void *pvParameters) {
    //GPIO Pins of the side sensors
    const uint L_TRIG_PIN = 19;
    const uint L_ECHO_PIN = 18;
    const uint R_TRIG_PIN = 14;
    const uint R_ECHO_PIN = 15;

    const float CONVERSION_RATE = (float)(MAX_SERVO_MICROS - MIN_SERVO_MICROS) / (MAX_SIDE_DISTANCE_TO_TURN - MIN_SIDE_DISTANCE_TO_TURN) / 2;

    //initilization of ultrasonic sensors    
    setupUltrasonicPins(L_TRIG_PIN, L_ECHO_PIN);
    setupUltrasonicPins(R_TRIG_PIN, R_ECHO_PIN);

    int left_prev = 0;
    int right_prev = 0;
    int left_array[ARRAY_SIZE] = {0};
    int right_array[ARRAY_SIZE] = {0};
    int array_count = 0;
    //Read from sensors until the array has only 1 empty slot before the main loop
    for(; array_count < ARRAY_SIZE - 1; array_count++){

        int left_distance  = getCm(L_TRIG_PIN, L_ECHO_PIN);
        if(left_distance > MAX_SIDE_SENSOR_DISTANCE) left_distance = MAX_SIDE_SENSOR_DISTANCE;
        left_array[array_count] = left_distance;

        int right_distance = getCm(R_TRIG_PIN, R_ECHO_PIN);
        if(right_distance > MAX_SIDE_SENSOR_DISTANCE) right_distance = MAX_SIDE_SENSOR_DISTANCE;
        right_array[array_count] = right_distance;
    }

    //gets current time in terms of freeRTOS ticks
    TickType_t xNextWaitTime;
    xNextWaitTime = xTaskGetTickCount(); 

    while(true) {
        //read from sensors
        int left_distance  = getCm(L_TRIG_PIN, L_ECHO_PIN);
        if (left_distance > MAX_SIDE_SENSOR_DISTANCE) left_distance = MAX_SIDE_SENSOR_DISTANCE;
        int right_distance = getCm(R_TRIG_PIN, R_ECHO_PIN);
        if (right_distance > MAX_SIDE_SENSOR_DISTANCE) right_distance = MAX_SIDE_SENSOR_DISTANCE;

        //Add the current distance measurement to the array by overwriting the most outdated measurement
        array_count++;
        if(array_count > ARRAY_SIZE - 1) array_count = 0;
        left_array[array_count]  = left_distance;
        right_array[array_count] = right_distance;

        //Calculate the median measurements of the arrays
        int left_distance_median  = getMedian(left_array, ARRAY_SIZE);
        int right_distance_median = getMedian(right_array, ARRAY_SIZE);
        printf("\nLeft Distance = %dcm\tRight Distance = %dcm", left_distance, right_distance);
        
        //Calculate the turn amount using the conversion rate so that the servo can turn more smoothly
        int value_to_turn; 
        if(left_distance_median <= MIN_SIDE_DISTANCE_TO_TURN || right_distance_median <= MIN_SIDE_DISTANCE_TO_TURN) {
            if(left_distance_median < right_distance_median){
                value_to_turn =  (MAX_SERVO_MICROS - MIN_SERVO_MICROS) / 2;
            } else {
                value_to_turn = -(MAX_SERVO_MICROS - MIN_SERVO_MICROS) / 2;
            }
        }
        else if (left_distance_median <= MAX_SIDE_DISTANCE_TO_TURN || right_distance_median <= MAX_SIDE_DISTANCE_TO_TURN) {
            if(left_distance_median > MAX_SIDE_DISTANCE_TO_TURN)  left_distance_median = MAX_SIDE_DISTANCE_TO_TURN;
            if(right_distance_median > MAX_SIDE_DISTANCE_TO_TURN) right_distance_median = MAX_SIDE_DISTANCE_TO_TURN;
            value_to_turn = (right_distance_median - left_distance_median) * CONVERSION_RATE;
        }
        else {
            value_to_turn = 0;
        }
        printf("\tValue to Turn = %d", value_to_turn);
        //Send the data to the queue so that the servo task can access it
        xQueueSend(xSideQueue, &value_to_turn, 0U);
        //Store the previous distances measured 
        right_prev = right_distance_median;
        left_prev  = left_distance_median;

        //delay certain amount of ticks
        xTaskDelayUntil(&xNextWaitTime, (TickType_t)SIDE_READ_FREQUENCY);
    }
}

//Motor task that waits for data from the front sensor that controls the robot's speed
void motor_task_exp(void *pvParameters){
    const uint MOTOR_PIN = 2;
    //Motor Conversion Rate     ==>     1000 = Reverse Max,     1500 = Stop,    2000 = Forward Max.   
    const float CONVERSION_RATE = (float)(MAX_MOTOR_FORWARD_MICROS - MIN_MOTOR_FORWARD_MICROS) / (MAX_FRONT_DISTANCE);
    float current_micros = MOTOR_BRAKE_MICROS;
    float prev_micros = MOTOR_BRAKE_MICROS;
    int micros_received = 0;
    bool direction = FORWARD;
    //Initiliaze motor as servo so that we can control it through ESC
    setServo(MOTOR_PIN, current_micros);
    while(true) {
        //Wait for the front sensor to send data
        xQueuePeek(xMiddleQueue, &micros_received, portMAX_DELAY);
        if(micros_received <= MIN_FRONT_DISTANCE){
            //set the esc direction change 
            if(direction != BACKWARD) {
                changeMotorState(MOTOR_PIN, BACKWARD);
            }
            //Go backwards if front distance is less than 10cm
            // current_micros = MIDDLE_MICROS - (MIN_FRONT_DISTANCE - micros_received) * 25;
            current_micros = MOTOR_BACKWARD_MICROS;
            direction = BACKWARD;

        } else {
            if(direction != FORWARD) {
                changeMotorState(MOTOR_PIN, FORWARD);
            }
            //Go forward if fron distance is more than 10cm
            current_micros = MIN_MOTOR_FORWARD_MICROS + (CONVERSION_RATE * micros_received);
            if(current_micros > MAX_MOTOR_FORWARD_MICROS) current_micros = MAX_MOTOR_FORWARD_MICROS;
            direction = FORWARD;
        }
        printf("\nReceived Motor Input = %f\n", current_micros);
        setMillis(MOTOR_PIN, prev_micros * 0.8f + current_micros * 0.2f);
        //Store previous speed for future use
        prev_micros = current_micros;
        vTaskDelay(FRONT_READ_FREQUENCY);
    }
}

void servo_task(void *pvParameters) {
    const uint SERVO_PIN = 1;
    //Servo Conversion Rate      ==>     1000us = 0 Degrees,   1500us = 60 Degrees,   2000us = 120 Degrees.
    const float MIDDLE_MICROS = (MAX_SERVO_MICROS + MIN_SERVO_MICROS) / 2;
    float current_micros = MIDDLE_MICROS;
    int value_to_turn = 0;
    int front_sensor_peeked = 0;
    bool direction = FORWARD;
    //Initiliaze servo
    setServo(SERVO_PIN, current_micros);

    while (true) {   
        //Waits until the queue receives data and writes the data to value_to_turn variable
        xQueueReceive(xSideQueue, &value_to_turn, portMAX_DELAY);
        printf("\nReceived Servo Input = %d", value_to_turn);
        //Turn the servo according to the data sent from the sensor task
        // xQueueReceive(xMotorDirection, &direction, portMAX_DELAY);
        xQueuePeek(xMiddleQueue, &front_sensor_peeked, portMAX_DELAY);
        if(front_sensor_peeked >= MIN_FRONT_DISTANCE) direction = FORWARD;
        else                                          direction = BACKWARD;
        if(direction == FORWARD) {
            current_micros = MIDDLE_MICROS - value_to_turn;
            if(current_micros < MIN_SERVO_MICROS) current_micros = MIN_SERVO_MICROS;
        } else {
            current_micros = MIDDLE_MICROS + value_to_turn;
            if(current_micros > MAX_SERVO_MICROS) current_micros = MAX_SERVO_MICROS;
        }
        setMillisRound(SERVO_PIN, current_micros, SERVO_ROUND_INTERVAL);
    }
}

/*-----------------------------------------------------------*/

int main()
{
    //initiliaze serial communication through USB
    stdio_init_all();

    //Create queue for the side sensors
    xSideQueue   = xQueueCreate(1, sizeof(int));
    xMiddleQueue = xQueueCreate(1, sizeof(int)); 
    //Create freeRTOS tasks
    xTaskCreate(led_task, 
                "LED_Task", 
                configMINIMAL_STACK_SIZE, 
                NULL, 
                1, 
                NULL);

    xTaskCreate(front_sensor_task, 
                "Front_Servor_Task", 
                1024, 
                NULL, 
                4, 
                NULL);

    xTaskCreate(motor_task_exp, 
                "Motor_Task", 
                1024, 
                NULL, 
                3, 
                NULL);

    xTaskCreate(side_sensor_task, 
                "Side_Sensor_Task", 
                1024, 
                NULL, 
                4, 
                NULL);

    xTaskCreate(servo_task, 
                "Servo_Task", 
                1024, 
                NULL, 
                3, 
                NULL);    

    sleep_ms(1500);
    //start the main loop
    vTaskStartScheduler();

    //The code never reaches her
    while(1){};
}
