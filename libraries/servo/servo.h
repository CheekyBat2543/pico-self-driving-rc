#ifndef servo_h
#define servo_h

void setMillis(int servoPin, float millis);
void setServo(int servoPin, float startMillis);
void setMillisRound(int servoPin, float startMillis, int roundInterval);
#endif