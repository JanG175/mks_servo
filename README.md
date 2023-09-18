# MSK-SERVO57C for ESP32 (ESP-IDF) 

## Notes
* Uncomment `#define MKS_STEP_MODE_ENABLE 1` in `mks_servo.h` in config section to enable step mode.
* Uncomment `#define MKS_PC_RETURN 1` in `mks_servo.h` in config section to enable mks servo return message.
* Declare how many motors you want to use by changing `MKS_MOTOR_N` in `mks_servo.h`.
* Datasheet claims that maximum speed is 1600 but in reality it is 1279 (see `MKS_MAX_SPEED` in `mks_servo.h`).

## To do list:
* debug and test

## Sources
https://github.com/makerbase-mks/MKS-SERVO57C