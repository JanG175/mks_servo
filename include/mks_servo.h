/**
 * @file mks_servo.h
 * @author JanG175
 * @brief MKS SERVO SERIAL PROTOCOL LIBRARY
 * 
 * @copyright All rigths reserved (R) 2023
 */

#include <stdio.h>
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// MKS HEAD
#define UPLINK_HEAD             0xFA
#define DOWNLINK_HEAD           0xFB

#define BROADCAST_ADDRESS       0x00

// MKS read parameters commands
#define ENCODER_READ            0x30
#define PULSE_READ              0x33
#define SHAFT_ERROR_READ        0x39
#define ENABLE_READ             0x3A
#define GO_TO_ZERO_READ         0x3B
#define RELEASE_SHAFT           0x3D
#define SHAFT_READ              0x3E

// MKS parameters commands
#define CAL_FUNC                0x80
#define MODE_FUNC               0x82
#define MA_FUNC                 0x83
#define MSTEP_FUNC              0x84
#define EN_FUNC                 0x85
#define DIR_FUNC                0x86
#define PROTECT_FUNC            0x88
#define MPLYER_FUNC             0x89
#define UART_BAUD_FUNC          0x8A
#define UART_ADDR_FUNC          0x8B
#define UART_RSP_FUNC           0x8C
#define RESTORE_FUNC            0x3F

// MKS serial control commands
#define CR_UART_QUERY           0xF1
#define CR_UART_ENABLE          0xF3
#define CR_UART_MOTOR_RUN       0xFD
#define CR_UART_SAVE_CLEAR      0xFF

// CR_UART status
#define CR_UART_STATUS_SAVE     0xC8
#define CR_UART_STATUS_CLEAR    0xCA

// MKS baud rates
#define BAUDRATE_9600           0x01
#define BAUDRATE_19200          0x02
#define BAUDRATE_25000          0x03
#define BAUDRATE_38400          0x04
#define BAUDRATE_57600          0x05
#define BAUDRATE_115200         0x06
#define BAUDRATE_256000         0x07

// MKS configs
#define TIMER_GROUP TIMER_GROUP_0
#define TIMER_ID TIMER_0

#define UART_TIMEOUT_MS (100 / portTICK_PERIOD_MS)
#define UART_MAX_REPEAT 10

#define CW_DIR 0
#define CCW_DIR 1

#define FULL_ROT 200

typedef struct mks_config_t
{
    uart_port_t uart;
    uint32_t baudrate;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    gpio_num_t* step_pin;
    gpio_num_t* dir_pin;
    gpio_num_t* en_pin;
} mks_config_t;

typedef struct cb_arg_t
{
    gpio_num_t step_pin;
    uint32_t motor_num;
} cb_arg_t;


void mks_servo_init(mks_config_t mks_config);

void mks_servo_deinit(mks_config_t mks_config);

void mks_servo_enable(mks_config_t mks_config, uint32_t motor_num, uint32_t enable);

void mks_servo_set_dir(mks_config_t mks_config, uint32_t motor_num, uint32_t cw);

void mks_servo_set_period(uint32_t motor_num, uint32_t period_us);

void mks_servo_start(mks_config_t mks_config, uint32_t motor_num, uint32_t start);

void mks_servo_step_move(mks_config_t mks_config, int64_t* steps, uint32_t* period_us);

float mks_servo_uart_read_encoder(mks_config_t mks_config, uint8_t address);

int32_t mks_servo_uart_read_pulses(mks_config_t mks_config, uint8_t address);

int16_t mks_servo_uart_read_motor_shaft_error(mks_config_t mks_config, uint8_t address);

uint8_t mks_servo_uart_read_enable(mks_config_t mks_config, uint8_t address);

uint8_t mks_servo_uart_read_go_to_zero_status(mks_config_t mks_config, uint8_t address);

uint8_t mks_servo_uart_release_protection_state(mks_config_t mks_config, uint8_t address);

uint8_t mks_servo_uart_read_protection_state(mks_config_t mks_config, uint8_t address);

uint8_t mks_servo_uart_calibrate_encoder(mks_config_t mks_config, uint8_t address);

uint8_t mks_servo_uart_set_work_mode(mks_config_t mks_config, uint8_t address, uint8_t mode);

uint8_t mks_servo_uart_set_current(mks_config_t mks_config, uint8_t address, uint8_t ma);

uint8_t mks_servo_uart_set_mstep(mks_config_t mks_config, uint8_t address, uint8_t mstep);

uint8_t mks_servo_uart_set_enable(mks_config_t mks_config, uint8_t address, uint8_t enable);

uint8_t mks_servo_uart_set_dir(mks_config_t mks_config, uint8_t address, uint8_t dir);

uint8_t mks_servo_uart_set_shaft_protection(mks_config_t mks_config, uint8_t address, uint8_t enable);

uint8_t mks_servo_uart_set_mplyer(mks_config_t mks_config, uint8_t address, uint8_t enable);

uint8_t mks_servo_uart_set_baud_rate(mks_config_t mks_config, uint8_t address, uint8_t baud);

uint8_t mks_servo_uart_set_slave_address(mks_config_t mks_config, uint8_t address, uint8_t new_address);

uint8_t mks_servo_uart_set_slave_respond(mks_config_t mks_config, uint8_t address, uint8_t enable);

uint8_t mks_servo_uart_restore(mks_config_t mks_config, uint8_t address);

uint8_t mks_servo_uart_cr_query(mks_config_t mks_config, uint8_t address);

uint8_t mks_servo_uart_cr_enable(mks_config_t mks_config, uint8_t address, uint8_t enable);

uint8_t mks_servo_uart_cr_run_w_speed(mks_config_t mks_config, uint8_t address, int16_t speed, uint8_t accel);

uint8_t mks_servo_uart_cr_save_params(mks_config_t mks_config, uint8_t address);

uint8_t mks_servo_uart_cr_clear_params(mks_config_t mks_config, uint8_t address);

uint8_t mks_servo_uart_cr_set_pos(mks_config_t mks_config, uint8_t address, int16_t speed, uint8_t accel, uint32_t pulses);