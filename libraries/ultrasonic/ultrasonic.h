#ifndef ultrasonic_h
#define ultrasonic_h
void setupUltrasonicPins(int trigPin, int echoPin);
int getCm(int trigPin, int echoPin);
int getCm_with_temperature(uint trigPin, uint echoPin, float temperature);
int getInch(int trigPin, int echoPin);
#endif