# MSK-SERVO57C for ESP32 (ESP-IDF) 

## Notes
* Declare how many motors you want to use by changing `MKS_MOTOR_N` in `mks_servo.c`.
* Datasheet claims that maximum speed is 1600 but in reality it is 1279 (see `MKS_MAX_SPEED` in `mks_servo.h`).

## To do list:
* debug and test

## Sources
https://github.com/makerbase-mks/MKS-SERVO57C