/**
 * @file mks_servo.c
 * @author JanG175
 * @brief MKS SERVO SERIAL PROTOCOL LIBRARY
 * 
 * @copyright All rigths reserved (R) 2023
 */

#include <stdio.h>
#include "mks_servo.h"

#define MKS_SERVO_N 1 // declare how many motors do you want to use


static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

static gptimer_handle_t gptimer[MKS_SERVO_N];
static int64_t steps_left[MKS_SERVO_N];
static const uint32_t timer_N = MKS_SERVO_N; // number of timers

static const char* TAG = "mks_servo";


/**
 * @brief calculate CRC for MKS UART communication
 * 
 * @param datagram pointer to datagram
 * @param len length of datagram
 * 
 * @return calculated CRC
 */
static uint8_t mks_servo_uart_calc_CRC(uint8_t* datagram, uint32_t len)
{
    uint64_t crc = 0;

    for (uint32_t i = 0; i < len - 1; i++)
        crc += datagram[i];

    return (uint8_t)(crc & 0xFF);
}


/**
 * @brief check if CRC is correct
 * 
 * @param datagram pointer to datagram
 * @param len length of datagram
 * 
 * @return true - if CRC is correct; false - otherwise
 */
static bool mks_servo_uart_check_CRC(uint8_t* datagram, uint32_t len)
{
    uint8_t crc = mks_servo_uart_calc_CRC(datagram, len);

    if (crc == datagram[len - 1])
        return true;
    else
        return false;
}


/**
 * @brief write datagram to register via UART
 * 
 * @param mks_config struct with MKS connection parameters
 * @param datagram pointer to datagram
 * @param len length of datagram
 */
static void mks_servo_uart_send(mks_conf_t mks_config, uint8_t* datagram, uint8_t len)
{
    uart_write_bytes(mks_config.uart, (const uint8_t*)datagram, len);
    ESP_ERROR_CHECK(uart_wait_tx_done(mks_config.uart, MKS_UART_TIMEOUT_MS));
}


/**
 * @brief read datagram from register via UART
 * 
 * @param mks_config struct with MKS connection parameters
 * @param datagram pointer to datagram
 * @param len length of datagram
 */
static void mks_servo_uart_recv(mks_conf_t mks_config, uint8_t* datagram, uint8_t len)
{
    uint32_t buf = 0;
    uint8_t data[len];

    buf = uart_read_bytes(mks_config.uart, data, len, MKS_UART_TIMEOUT_MS);
    uart_flush(mks_config.uart);

    if (buf == len)
    {
        for (int i = 0; i < buf; i++)
        {
            datagram[i] = data[i];
        }
    }
    else
    {
        ESP_LOGE(TAG, "UART read error");

        for (int i = 0; i < len; i++)
            datagram[i] = 0;
    }
}


/**
 * @brief callback function for timers
 * 
 * @param timer timer handle
 * @param edata event data
 * @param user_ctx user context
 * @return true - if high priority task was woken up; false - otherwise
 */
static bool clk_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;

    mks_cb_arg_t cb_arg = *(mks_cb_arg_t*)user_ctx;
    gpio_num_t step_pin = cb_arg.step_pin;
    uint32_t motor_num = cb_arg.motor_num;

    if (steps_left[motor_num] > 0)
    {
        if (gpio_get_level(step_pin) == 0)
            gpio_set_level(step_pin, 1);
        else
            gpio_set_level(step_pin, 0);

        portENTER_CRITICAL(&spinlock);
        steps_left[motor_num]--;
        portEXIT_CRITICAL(&spinlock);
    }
    else
        ESP_ERROR_CHECK(gptimer_stop(timer));

    return (high_task_awoken == pdTRUE);
}


/**
 * @brief initialize MKS UART and timers
 * 
 * @param mks_config struct with MKS connection parameters
 */
void mks_servo_init(mks_conf_t mks_config)
{
    // configure UART
    uart_config_t uart_config = {
        .baud_rate = mks_config.baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT
    };

    if (uart_is_driver_installed(mks_config.uart) == true)
        uart_driver_delete(mks_config.uart);

    ESP_ERROR_CHECK(uart_driver_install(mks_config.uart, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(mks_config.uart, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(mks_config.uart, mks_config.tx_pin, mks_config.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    mks_cb_arg_t* cb_arg = malloc(sizeof(mks_cb_arg_t) * timer_N); // allocate memory for callback arguments

    for (uint32_t i = 0; i < timer_N; i++)
    {
        // configure step and dir pins

        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
        io_conf.pin_bit_mask = ((1 << mks_config.step_pin[i]) | (1 << mks_config.dir_pin[i]) | (1 << mks_config.en_pin[i]));
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);

        mks_servo_enable(mks_config, i, 0);
        mks_servo_set_dir(mks_config, i, MKS_CW_DIR);

        // configure timers

        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000 // 1 us
        };

        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer[i]));

        gptimer_alarm_config_t alarm_config = {
            .alarm_count = 1000000, // 1 s
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true
        };

        ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer[i], &alarm_config));

        gptimer_event_callbacks_t timer_cbs = {
            .on_alarm = clk_timer_callback
        };

        cb_arg[i].step_pin = mks_config.step_pin[i];
        cb_arg[i].motor_num = i;

        ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer[i], &timer_cbs, (void*)&cb_arg[i]));

        ESP_ERROR_CHECK(gptimer_enable(gptimer[i]));

        mks_servo_enable(mks_config, i, 1); // enable motor
    }
}


// deinit MKS UART and timers
void mks_servo_deinit(mks_conf_t mks_config)
{
    for (uint32_t i = 0; i < timer_N; i++)
    {
        ESP_ERROR_CHECK(gptimer_disable(gptimer[i]));
        ESP_ERROR_CHECK(gptimer_del_timer(gptimer[i]));
    }

    ESP_ERROR_CHECK(uart_driver_delete(mks_config.uart));
}


/**
 * @brief set enable pin
 * 
 * @param mks_config struct with MKS connection parameters
 * @param motor_num motor number
 * @param enable 0 - disable, 1 - enable
 */
void mks_servo_enable(mks_conf_t mks_config, uint32_t motor_num, uint32_t enable)
{
    if (enable == 0)
        gpio_set_level(mks_config.en_pin[motor_num], 0);
    else if (enable == 1)
        gpio_set_level(mks_config.en_pin[motor_num], 1);
}


/**
 * @brief set direction pin
 * 
 * @param mks_config struct with MKS connection parameters
 * @param motor_num motor number
 * @param dir direction (CW_DIR or CCW_DIR)
 */
void mks_servo_set_dir(mks_conf_t mks_config, uint32_t motor_num, uint32_t dir)
{
    if (dir == MKS_CW_DIR)
        gpio_set_level(mks_config.dir_pin[motor_num], 0);
    else if (dir == MKS_CCW_DIR)
        gpio_set_level(mks_config.dir_pin[motor_num], 1);
}


/**
 * @brief set period for step signal
 * 
 * @param motor_num motor number
 * @param period_us period in us
 */
void mks_servo_set_period(uint32_t motor_num, uint32_t period_us)
{
    if (period_us < 200)
        ESP_LOGW(TAG, "low period value - motor may not work properly");

    period_us = period_us / 2; // 1 period = 2 gpio switches

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = period_us,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer[motor_num], &alarm_config));
}


/**
 * @brief start/stop step signal
 * 
 * @param mks_config struct with MKS connection parameters
 * @param motor_num motor number
 * @param start 0 - stop, 1 - start
 */
void mks_servo_start(mks_conf_t mks_config, uint32_t motor_num, uint32_t start)
{
    if (start == 0)
    {
        ESP_ERROR_CHECK(gptimer_stop(gptimer[motor_num]));
        gpio_set_level(mks_config.step_pin[motor_num], 0);
    }
    else if (start == 1)
    {
        ESP_ERROR_CHECK(gptimer_start(gptimer[motor_num]));
    }
}


/**
 * @brief move all motors by desired number of steps with desired period and direction (sign in steps variable)
 * 
 * @param mks_config struct with MKS connection parameters
 * @param steps array of steps for each motor
 * @param period_us array of periods for each motor
 */
void mks_servo_step_move(mks_conf_t mks_config, int64_t* steps, uint32_t* period_us)
{
    for (uint32_t motor_num = 0; motor_num < timer_N; motor_num++)
    {
        // set direction
        if (steps[motor_num] < 0)
        {
            mks_servo_set_dir(mks_config, motor_num, 0);
            steps[motor_num] = -steps[motor_num];
        }
        else
            mks_servo_set_dir(mks_config, motor_num, 1);

        // set period
        mks_servo_set_period(motor_num, period_us[motor_num]);

        // set number of steps
        portENTER_CRITICAL(&spinlock);
        steps_left[motor_num] = 2 * steps[motor_num];
        portEXIT_CRITICAL(&spinlock);
    }

    // start all motors one by one
    for (uint32_t motor_num = 0; motor_num < timer_N; motor_num++)
        mks_servo_start(mks_config, motor_num, 1);

    bool end_wait = false;

    // wait until all motors stop
    while (end_wait == false)
    {
        int64_t steps_left_status = 0;

        for (uint32_t motor_num = 0; motor_num < timer_N; motor_num++)
            steps_left_status = steps_left_status + steps_left[motor_num];

        if (steps_left_status <= 0)
            end_wait = true;

        vTaskDelay(1);
    }

    // // for debug
    // for (uint32_t motor_num = 0; motor_num < timer_N; motor_num++)
    //     ESP_LOGI(TAG, "%lu: %lld", motor_num, steps_left[motor_num] / 2);
}


/**
 * @brief read encoder value
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * 
 * @return encoder value
 */
float mks_servo_uart_read_encoder(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];
    uint8_t len_r = 10;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_ENCODER_READ;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    int32_t carry = response[3] << 24 | response[4] << 16 | response[5] << 8 | response[6];
    uint16_t value = response[7] << 8 | response[8];

    float encoder_value = (float)carry * 360.0f + (float)value / 65535.0f * 360.0f;

    return encoder_value;
}


/**
 * @brief read the number of pulses received.
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return number of pulses
 */
int32_t mks_servo_uart_read_pulses(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];
    uint8_t len_r = 8;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_PULSE_READ;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    int32_t pulses = response[3] << 24 | response[4] << 16 | response[5] << 8 | response[6];

    return pulses;
}


/**
 * @brief read the error of the motor shaft angle
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * 
 * @return error of the motor shaft angle
 */
int16_t mks_servo_uart_read_motor_shaft_error(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];
    uint8_t len_r = 6;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_SHAFT_ERROR_READ;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    int16_t shaft_error = response[3] << 8 | response[4];

    return shaft_error;
}


/**
 * @brief read enable status
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return enable status
 */
uint8_t mks_servo_uart_read_enable(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_ENABLE_READ;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief read the go back to zero status when power on
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return go back to zero status (0 - going to zero, 1 - success, 2 - fail)
 */
uint8_t mks_servo_uart_read_go_to_zero_status(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_GO_TO_ZERO_READ;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief release the motor shaft locked-rotor protection state
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_release_protection_state(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_RELEASE_SHAFT;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief read the motor shaft protection state
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return motor shaft protection state (0 - not protected, 1 - protected)
 */
uint8_t mks_servo_uart_read_protection_state(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_SHAFT_READ;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief calibrate the encoder
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return 0 if calibrating, 1 if success, 2 if fail
 */
uint8_t mks_servo_uart_calibrate_encoder(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_CAL_FUNC;
    datagram[3] = 0x00;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief set the work mode
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param mode work mode (0 - CR_OPEN, 1 - CR_CLOSE, 2 - CR_vFOC, 3 - CR_UART)
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_work_mode(mks_conf_t mks_config, uint8_t address, uint8_t mode)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_MODE_FUNC;
    datagram[3] = mode;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief set the current limit
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param ma current limit in mA (ma * 400 mA)
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_current(mks_conf_t mks_config, uint8_t address, uint8_t ma)
{
    if (ma > 0x0D)
    {
        ESP_LOGW(TAG, "current limit is out of range");
        ma = 0x0D;
    }

    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_MA_FUNC;
    datagram[3] = ma;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief set subdivision
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param mstep microstep value
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_mstep(mks_conf_t mks_config, uint8_t address, uint8_t mstep)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_MSTEP_FUNC;
    datagram[3] = mstep;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief set the active of the enable pin
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param enable enable (0 - active low, 1 - active high, 2 - active always)
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_enable(mks_conf_t mks_config, uint8_t address, uint8_t enable)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_EN_FUNC;
    datagram[3] = enable;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief set the direction of motor rotation
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param dir direction (0 - CW, 1 - CCW)
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_dir(mks_conf_t mks_config, uint8_t address, uint8_t dir)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_DIR_FUNC;
    datagram[3] = dir;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief set the motor shaft locked-rotor protection function
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param enable enable (0 - disabled protection, 1 - enabled protection)
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_shaft_protection(mks_conf_t mks_config, uint8_t address, uint8_t enable)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_PROTECT_FUNC;
    datagram[3] = enable;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief set the subdivision interpolation function
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param enable enable (0 - Mplyer disabled, 1 - Mplyer enabled)
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_mplyer(mks_conf_t mks_config, uint8_t address, uint8_t enable)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_MPLYER_FUNC;
    datagram[3] = enable;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief set the baud rate
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param baud baud rate value
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_baud_rate(mks_conf_t mks_config, uint8_t address, uint8_t baud)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_UART_BAUD_FUNC;
    datagram[3] = baud;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief set the slave address
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param new_address new slave address
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_slave_address(mks_conf_t mks_config, uint8_t address, uint8_t new_address)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_UART_ADDR_FUNC;
    datagram[3] = new_address;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief set the slave respond
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param enable 1 - enable slave respond, 0 - disable slave respond
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_set_slave_respond(mks_conf_t mks_config, uint8_t address, uint8_t enable)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_UART_RSP_FUNC;
    datagram[3] = enable;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief restore the default parameters
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return 1 if success, 0 if fail
 */
uint8_t mks_servo_uart_restore(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_RESTORE_FUNC;
    datagram[3] = (uint8_t)NULL;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief query the motor status
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return 0 - run fail, 1 - run starting, 2 - run complete
 */
uint8_t mks_servo_uart_cr_query(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_CR_UART_QUERY;
    datagram[3] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief enable the motor
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param enable 1 - enable motor, 0 - disable motor
 * @return 0 - set fail, 1 - set success
 */
uint8_t mks_servo_uart_cr_enable(mks_conf_t mks_config, uint8_t address, uint8_t enable)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_CR_UART_ENABLE;
    datagram[3] = enable;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief run the motor in speed mode
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param speed speed value with direction sign (-1600 - 1600)
 * @param accel acceleration value (0 - 32)
 * @return 0 - set fail, 1 - set success
 */
uint8_t mks_servo_uart_cr_run_w_speed(mks_conf_t mks_config, uint8_t address, int16_t speed, uint8_t accel)
{
    if (speed > 1600)
    {
        ESP_LOGW(TAG, "speed is too high, set to 1600");
        speed = 1600;
    }
    else if (speed < -1600)
    {
        ESP_LOGW(TAG, "speed is too high, set to -1600");
        speed = -1600;
    }

    if (speed < 0)
    {
        speed = -speed;
        speed = speed | (1 << 15);
    }

    if (accel > 32)
    {
        ESP_LOGW(TAG, "accel is too high, set to 32");
        accel = 32;
    }

    uint8_t len_w = 7;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_CR_UART_MOTOR_RUN;
    datagram[3] = speed >> 8;
    datagram[4] = speed;
    datagram[5] = accel;
    datagram[6] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief save the parameter in speed mode
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return 0 - set fail, 1 - set success
 */
uint8_t mks_servo_uart_cr_save_params(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_CR_UART_SAVE_CLEAR;
    datagram[3] = MKS_CR_UART_STATUS_SAVE;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief clear the parameter in speed mode
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @return 0 - set fail, 1 - set success
 */
uint8_t mks_servo_uart_cr_clear_params(mks_conf_t mks_config, uint8_t address)
{
    uint8_t len_w = 5;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_CR_UART_SAVE_CLEAR;
    datagram[3] = MKS_CR_UART_STATUS_CLEAR;
    datagram[4] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}


/**
 * @brief run the motor in position mode
 * 
 * @param mks_config struct with MKS configs
 * @param address MKS slave address
 * @param speed speed value with direction sign (-1600 - 1600)
 * @param accel acceleration value (0 - 32)
 * @param pulses number of pulses
 * @return 0 - set fail, 1 - set success
 */
uint8_t mks_servo_uart_cr_set_pos(mks_conf_t mks_config, uint8_t address, int16_t speed, uint8_t accel, uint32_t pulses)
{
        if (speed > 1600)
    {
        ESP_LOGW(TAG, "speed is too high, set to 1600");
        speed = 1600;
    }
    else if (speed < -1600)
    {
        ESP_LOGW(TAG, "speed is too high, set to -1600");
        speed = -1600;
    }

    if (speed < 0)
    {
        speed = -speed;
        speed = speed | (1 << 15);
    }

    if (accel > 32)
    {
        ESP_LOGW(TAG, "accel is too high, set to 32");
        accel = 32;
    }

    uint8_t len_w = 11;
    uint8_t datagram[len_w];
    uint8_t len_r = 5;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = MKS_UPLINK_HEAD;
    datagram[1] = address;
    datagram[2] = MKS_CR_UART_MOTOR_RUN;
    datagram[3] = (speed >> 8) & 0xFF;
    datagram[4] = speed & 0xFF;
    datagram[5] = accel;
    datagram[6] = (pulses >> 24) & 0xFF;
    datagram[7] = (pulses >> 16) & 0xFF;
    datagram[8] = (pulses >> 8) & 0xFF;
    datagram[9] = pulses & 0xFF;
    datagram[10] = mks_servo_uart_calc_CRC(datagram, len_w);

    mks_servo_uart_send(mks_config, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        mks_servo_uart_recv(mks_config, response, len_r);
        cnt++;
    } while ((response[0] != MKS_DOWNLINK_HEAD || response[1] != address) && (cnt < MKS_UART_MAX_REPEAT));

    if (cnt >= MKS_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");
        abort();
    }

    return response[3];
}