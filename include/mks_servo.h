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

// MKS config
#define MKS_STEP_MODE_ENABLE 1 // uncomment to enable step mode
// #define MKS_PC_RETURN 1 // uncommnent if you want to return message to PC

// MKS HEAD
#define MKS_UPLINK_HEAD             0xFA
#define MKS_DOWNLINK_HEAD           0xFB

#define MKS_BROADCAST_ADDRESS       0x00

// MKS read parameters commands
#define MKS_ENCODER_READ            0x30
#define MKS_PULSE_READ              0x33
#define MKS_SHAFT_ERROR_READ        0x39
#define MKS_ENABLE_READ             0x3A
#define MKS_GO_TO_ZERO_READ         0x3B
#define MKS_RELEASE_SHAFT           0x3D
#define MKS_SHAFT_READ              0x3E

// MKS parameters commands
#define MKS_CAL_FUNC                0x80
#define MKS_MODE_FUNC               0x82
#define MKS_MA_FUNC                 0x83
#define MKS_MSTEP_FUNC              0x84
#define MKS_EN_FUNC                 0x85
#define MKS_DIR_FUNC                0x86
#define MKS_PROTECT_FUNC            0x88
#define MKS_MPLYER_FUNC             0x89
#define MKS_UART_BAUD_FUNC          0x8A
#define MKS_UART_ADDR_FUNC          0x8B
#define MKS_UART_RSP_FUNC           0x8C
#define MKS_RESTORE_FUNC            0x3F

// MKS serial control commands
#define MKS_CR_UART_QUERY           0xF1
#define MKS_CR_UART_ENABLE          0xF3
#define MKS_CR_UART_MOTOR_TURN      0xF6
#define MKS_CR_UART_MOTOR_RUN       0xFD
#define MKS_CR_UART_SAVE_CLEAR      0xFF

// CR_UART status
#define MKS_CR_UART_STATUS_SAVE     0xC8
#define MKS_CR_UART_STATUS_CLEAR    0xCA

// MKS baud rates
#define MKS_BAUDRATE_9600           0x01
#define MKS_BAUDRATE_19200          0x02
#define MKS_BAUDRATE_25000          0x03
#define MKS_BAUDRATE_38400          0x04
#define MKS_BAUDRATE_57600          0x05
#define MKS_BAUDRATE_115200         0x06
#define MKS_BAUDRATE_256000         0x07

// MKS constants
#define MKS_UART_TIMEOUT_MS         (100 / portTICK_PERIOD_MS)
#define MKS_UART_MAX_REPEAT         10

#define MKS_CW_DIR                  0
#define MKS_CCW_DIR                 1
#define MKS_FULL_ROT                200
#define MKS_MAX_SPEED               1279

typedef struct mks_conf_t
{
    uart_port_t uart;
    uint32_t baudrate;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
#ifdef MKS_STEP_MODE_ENABLE
    uint8_t motor_num;
    gpio_num_t* step_pin;
    gpio_num_t* dir_pin;
    gpio_num_t* en_pin;
#endif // MKS_STEP_MODE_ENABLE
} mks_conf_t;

typedef struct mks_cb_arg_t
{
    gpio_num_t step_pin;
    uint8_t motor_num;
} mks_cb_arg_t;


void mks_servo_init(mks_conf_t mks_conf);

void mks_servo_deinit(mks_conf_t mks_conf);

#ifdef MKS_STEP_MODE_ENABLE

void mks_servo_enable(mks_conf_t mks_conf, uint8_t motor_num, bool enable);

void mks_servo_set_dir(mks_conf_t mks_conf, uint8_t motor_num, uint8_t dir);

void mks_servo_set_period(uint8_t motor_num, uint64_t period_us);

void mks_servo_start(mks_conf_t mks_conf, uint8_t motor_num, bool start);

void mks_servo_step_move(mks_conf_t mks_conf, uint8_t motor_num, uint64_t steps, int64_t period_us);

#endif // MKS_STEP_MODE_ENABLE

float mks_servo_uart_read_encoder(mks_conf_t mks_conf, uint8_t address);

int32_t mks_servo_uart_read_pulses(mks_conf_t mks_conf, uint8_t address);

int16_t mks_servo_uart_read_motor_shaft_error(mks_conf_t mks_conf, uint8_t address);

uint8_t mks_servo_uart_read_enable(mks_conf_t mks_conf, uint8_t address);

uint8_t mks_servo_uart_read_go_to_zero_status(mks_conf_t mks_conf, uint8_t address);

uint8_t mks_servo_uart_release_protection_state(mks_conf_t mks_conf, uint8_t address);

uint8_t mks_servo_uart_read_protection_state(mks_conf_t mks_conf, uint8_t address);

uint8_t mks_servo_uart_calibrate_encoder(mks_conf_t mks_conf, uint8_t address);

uint8_t mks_servo_uart_set_work_mode(mks_conf_t mks_conf, uint8_t address, uint8_t mode);

uint8_t mks_servo_uart_set_current(mks_conf_t mks_conf, uint8_t address, uint8_t ma);

uint8_t mks_servo_uart_set_mstep(mks_conf_t mks_conf, uint8_t address, uint8_t mstep);

uint8_t mks_servo_uart_set_enable(mks_conf_t mks_conf, uint8_t address, uint8_t enable);

uint8_t mks_servo_uart_set_dir(mks_conf_t mks_conf, uint8_t address, uint8_t dir);

uint8_t mks_servo_uart_set_shaft_protection(mks_conf_t mks_conf, uint8_t address, uint8_t enable);

uint8_t mks_servo_uart_set_mplyer(mks_conf_t mks_conf, uint8_t address, uint8_t enable);

uint8_t mks_servo_uart_set_baud_rate(mks_conf_t mks_conf, uint8_t address, uint8_t baud);

uint8_t mks_servo_uart_set_slave_address(mks_conf_t mks_conf, uint8_t address, uint8_t new_address);

uint8_t mks_servo_uart_set_slave_respond(mks_conf_t mks_conf, uint8_t address, uint8_t enable);

uint8_t mks_servo_uart_restore(mks_conf_t mks_conf, uint8_t address);

uint8_t mks_servo_uart_cr_query(mks_conf_t mks_conf, uint8_t address);

uint8_t mks_servo_uart_cr_enable(mks_conf_t mks_conf, uint8_t address, uint8_t enable);

uint8_t mks_servo_uart_cr_run_w_speed(mks_conf_t mks_conf, uint8_t address, int16_t speed, uint8_t accel);

uint8_t mks_servo_uart_cr_save_params(mks_conf_t mks_conf, uint8_t address);

uint8_t mks_servo_uart_cr_clear_params(mks_conf_t mks_conf, uint8_t address);

uint8_t mks_servo_uart_cr_set_pos(mks_conf_t mks_conf, uint8_t address, int16_t speed, uint8_t accel, uint32_t pulses);