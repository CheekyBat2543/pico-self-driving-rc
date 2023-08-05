#ifndef ultrasonic_h
#define ultrasonic_h
void ultrasonic_setup_pins(int trigPin, int echoPin);
int ultrasonic_get_distance_cm(int trigPin, int echoPin);
int ultrasonic_get_distance_temprerature_compansated_cm(uint trigPin, uint echoPin, float temperature);
int ultrasonic_get_inch(int trigPin, int echoPin);
int ultrasonic_lpf(int sensor_reading, float filterValue, float smoothedValue);
#endif